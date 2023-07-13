use core::ops::Bound::{Included, Unbounded};
use std::{cmp::max, collections::BTreeMap, iter};

use enumflags2::BitFlags;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::{
  dds::ddsdata::DDSData,
  discovery::sedp_messages::DiscoveredWriterData,
  messages::submessages::submessages::{DATAFRAG_Flags, DataFrag},
  rtps::fragment_assembler::FragmentAssembler,
  structure::{
    guid::{EntityId, GUID},
    locator::Locator,
    sequence_number::{FragmentNumber, SequenceNumber},
    time::Timestamp,
  },
};

#[derive(Debug)] // these are not cloneable, because contained data may be large
pub(crate) struct RtpsWriterProxy {
  /// Identifies the remote matched Writer
  pub remote_writer_guid: GUID,

  /// List of unicast (address, port) combinations that can be used to send
  /// messages to the matched Writer or Writers. The list may be empty.
  pub unicast_locator_list: Vec<Locator>,

  /// List of multicast (address, port) combinations that can be used to send
  /// messages to the matched Writer or Writers. The list may be empty.
  pub multicast_locator_list: Vec<Locator>,

  /// Identifies the group to which the matched Reader belongs
  pub remote_group_entity_id: EntityId,

  // See RTPS Spec v2.5 Section 8.4.10.4 on how the WriterProxy is supposed to
  // operate.
  // And 8.4.10.5 on statuses of the (cache) changes received from a writer.
  // The changes are identified by sequence numbers.
  // Any sequence number is at any moment in one of the following states:
  // unknown, missing, received, not_available.
  //
  // Unknown means that we have no information of that change. This is the initial state for
  // everyone. Missing means that writer claims to have it and reader may request it with
  // ACKNACK. Received means that the reader has received the change via DATA (or DATAFRAGs).
  // Not_available means that writer has informed via GAP message that change is not available.

  // Unknown can transition to Missing (via HEARTBEAT), Received (DATA), or not_available (GAP).
  // Missing can transition to Received or Not_available.
  // Received cannot transition to anything.
  // Not_available cannot transition to anything.

  // We keep a map "changes" and a sequence number counters "ack_base", to keep track of these.

  // changes.get(sn) is interpreted as follows:
  // * Some(Some(timestamp)) = received at timestamp
  // * Some(None) = not_available
  // * None = any state, see below:
  //
  // All changes below ack_base are either received or not_available.
  // All changes above hb_last are unknown (if they are not in "changes" map)
  // All changes between ack_base and hb_last (inclusive) are missing.

  // Timestamps are stored, because they are used as keys into the DDS Cache.
  changes: BTreeMap<SequenceNumber, Option<Timestamp>>,

  // The changes map is cleaned on heartbeat messages. The changes no longer available are dropped.
  pub received_heartbeat_count: i32,

  pub sent_ack_nack_count: i32,

  ack_base: SequenceNumber, // We can ACK everything before this number.
  // ack_base can be increased from N-1 to N, if we receive DATA with SequenceNumber N-1
  // heartbeat(first,last) => ack_base can be increased to first.
  // GAP is treated like receiving a message.

  // These are used for quick tracking of
  last_received_sequence_number: SequenceNumber,
  last_received_timestamp: Timestamp,

  fragment_assembler: Option<FragmentAssembler>,
}

impl RtpsWriterProxy {
  pub fn new(
    remote_writer_guid: GUID,
    unicast_locator_list: Vec<Locator>,
    multicast_locator_list: Vec<Locator>,
    remote_group_entity_id: EntityId,
  ) -> Self {
    Self {
      remote_writer_guid,
      unicast_locator_list,
      multicast_locator_list,
      remote_group_entity_id,
      changes: BTreeMap::new(),
      received_heartbeat_count: 0,
      sent_ack_nack_count: 0,
      // Sequence numbering must start at 1.
      // Therefore, we can ACK all sequence numbers below 1 even before receiving anything.
      ack_base: SequenceNumber::new(1),
      last_received_sequence_number: SequenceNumber::new(0),
      last_received_timestamp: Timestamp::INVALID,
      fragment_assembler: None,
    }
  }

  pub fn next_ack_nack_sequence_number(&mut self) -> i32 {
    let c = self.sent_ack_nack_count;
    self.sent_ack_nack_count += 1;
    c
  }

  // Returns a bound, below which everything can be acknowledged, i.e.
  // is received either as DATA or GAP
  pub fn all_ackable_before(&self) -> SequenceNumber {
    self.ack_base
  }

  pub fn update_contents(&mut self, other: Self) {
    self.unicast_locator_list = other.unicast_locator_list;
    self.multicast_locator_list = other.multicast_locator_list;
    self.remote_group_entity_id = other.remote_group_entity_id;
  }

  // This is used to check for DEADLINE policy
  pub fn last_change_timestamp(&self) -> Option<Timestamp> {
    if self.last_received_sequence_number > SequenceNumber::new(0) {
      Some(self.last_received_timestamp)
    } else {
      None
    }
  }

  // Check if we no samples in the received state.
  pub fn no_changes_received(&self) -> bool {
    self.ack_base == SequenceNumber::new(0) && self.changes.is_empty()
  }

  // Given an availability range from a HEARTBEAT, find out what we are missing.
  //
  // Note: Heartbeat gives bounds only. Some samples within that range may
  // have been received already, or not really available, i.e. there may be GAPs
  // in the range.
  pub fn missing_seqnums(
    &self,
    hb_first_sn: SequenceNumber,
    hb_last_sn: SequenceNumber,
  ) -> Vec<SequenceNumber> {
    // Need to verify first <= last, or BTreeMap::range will crash
    if hb_first_sn > hb_last_sn {
      if hb_first_sn > hb_last_sn + SequenceNumber::from(1) {
        warn!(
          "Negative range of missing_seqnums first={:?} last={:?}",
          hb_first_sn, hb_last_sn
        );
      } else {
        // first == last+1
        // This is normal. See RTPS 2.5 Spec Section "8.3.8.6.3 Validity"
        // It means nothing is available. Since nothing is available, nothing is
        // missing.
      }
      return vec![];
    }

    let mut missing_seqnums = Vec::with_capacity(32); // out of hat value

    let relevant_interval = SequenceNumber::range_inclusive(
      max(hb_first_sn, self.ack_base), // ignore those that we already have
      hb_last_sn,
    );

    // iterator over known Received and Not_available changes.
    let known =
      // again check for negative intervals, or BTreeMap::range will crash
      if relevant_interval.begin() <= relevant_interval.end() {
        self.changes
          .range( relevant_interval )
          .map(|e| *e.0)
          .collect()
      } else { vec![] };
    let mut known_iter = known.iter();
    let mut known_head = known_iter.next();

    // Iterate over all SequenceNumbers (indices) in the advertised range.
    for s in relevant_interval {
      match known_head {
        None => missing_seqnums.push(s), // no known changes left => s is missing
        Some(known_sn) => {
          // there are known changes left
          if *known_sn == s {
            // and the index sequence matches it => not missing
            // => advance to next known change and continue iteration
            known_head = known_iter.next();
          } else {
            // but it is not yet this index s => s is missing
            missing_seqnums.push(s);
          }
        }
      }
    }

    missing_seqnums
  }

  // Check if we have already received this sequence number
  // or it has been marked as not_available
  pub fn should_ignore_change(&self, seqnum: SequenceNumber) -> bool {
    seqnum < self.ack_base || self.changes.contains_key(&seqnum)
  }

  // This is used to mark DATA as received.
  pub fn received_changes_add(&mut self, seq_num: SequenceNumber, receive_timestamp: Timestamp) {
    self.changes.insert(seq_num, Some(receive_timestamp));

    // Update deadline tracker
    if seq_num > self.last_received_sequence_number {
      self.last_received_sequence_number = seq_num;
      self.last_received_timestamp = receive_timestamp;
    }

    // We get to advance ack_base if it was equal to seq_num
    // If ack_base < seq_num, we are still missing seq_num-1 or others below
    // If ack_base > seq_num, this is either a duplicate or ack_base was wrong.
    // Remember, ack_base is the SN one past the last received/irrelevant SN.
    if seq_num == self.ack_base {
      self.advance_ack_base();
    }
  }

  // Used to add individual irrelevant changes from GAP message
  pub fn set_irrelevant_change(&mut self, seq_num: SequenceNumber) {
    // If sequence number is still in the relevant range,
    // insert not_available marker
    if seq_num >= self.ack_base {
      self.changes.insert(seq_num, None);
    }

    if seq_num == self.ack_base {
      // ack_base can be advanced
      self.advance_ack_base();
    }
  }

  // Used to add range of irrelevant changes from GAP message
  pub fn irrelevant_changes_range(
    &mut self,
    remove_from: SequenceNumber,
    remove_until_before: SequenceNumber,
  ) {
    // check sanity
    if remove_from > remove_until_before {
      error!(
        "irrelevant_changes_range: negative range: remove_from={:?} remove_until_before={:?}",
        remove_from, remove_until_before
      );
      return;
    }
    // now remove_from <= remove_until_before, i.e. at least zero to remove
    //
    // Two cases here:
    // If remove_from <= self.ack_base, then we may proceed by moving
    // ack_base to remove_until_before and clearing "changes" before that.
    //
    // Else (remove_from > self.ack_base), which means we must insert not_available
    // markers to "changes".
    //
    if remove_from <= self.ack_base {
      let mut removed_and_after = self.changes.split_off(&remove_from);
      let mut after = removed_and_after.split_off(&remove_until_before);
      // let removed = removed_and_after;
      self.changes.append(&mut after);

      self.ack_base = max(remove_until_before, self.ack_base);
      debug!(
        "ack_base increased to {:?} by irrelevant_changes_range {:?} to {:?}. writer={:?}",
        self.ack_base, remove_from, remove_until_before, self.remote_writer_guid
      );
    } else {
      // TODO: This potentially generates a very large BTreeMap
      for na in
        SequenceNumber::range_inclusive(remove_from, remove_until_before - SequenceNumber::new(1))
      {
        self.changes.insert(na, None);
      }
    }
  }

  // Used to mark messages irrelevant because of a HEARTBEAT message.
  //
  // smallest_seqnum is the lowest key to be retained
  pub fn irrelevant_changes_up_to(&mut self, smallest_seqnum: SequenceNumber) {
    self.irrelevant_changes_range(SequenceNumber::new(0), smallest_seqnum);
  }

  fn discovered_or_default(drd: &[Locator], default: &[Locator]) -> Vec<Locator> {
    if drd.is_empty() {
      default.to_vec()
    } else {
      drd.to_vec()
    }
  }

  pub fn from_discovered_writer_data(
    discovered_writer_data: &DiscoveredWriterData,
    default_unicast_locators: &[Locator],
    default_multicast_locators: &[Locator],
  ) -> RtpsWriterProxy {
    let unicast_locator_list = Self::discovered_or_default(
      &discovered_writer_data.writer_proxy.unicast_locator_list,
      default_unicast_locators,
    );
    let multicast_locator_list = Self::discovered_or_default(
      &discovered_writer_data.writer_proxy.multicast_locator_list,
      default_multicast_locators,
    );

    RtpsWriterProxy {
      remote_writer_guid: discovered_writer_data.writer_proxy.remote_writer_guid,
      remote_group_entity_id: EntityId::UNKNOWN,
      unicast_locator_list,
      multicast_locator_list,
      changes: BTreeMap::new(),
      received_heartbeat_count: 0,
      sent_ack_nack_count: 0,
      ack_base: SequenceNumber::default(),
      last_received_sequence_number: SequenceNumber::new(0),
      last_received_timestamp: Timestamp::INVALID,
      fragment_assembler: None,
    }
  } // fn

  pub fn handle_datafrag(
    &mut self,
    datafrag: &DataFrag,
    flags: BitFlags<DATAFRAG_Flags>,
  ) -> Option<DDSData> {
    if let Some(ref mut fa) = self.fragment_assembler {
      fa.new_datafrag(datafrag, flags)
    } else {
      let mut fa = FragmentAssembler::new(datafrag.fragment_size);
      // TODO: Test that the fragment size is not zero
      let ret = fa.new_datafrag(datafrag, flags);
      self.fragment_assembler = Some(fa);
      ret
    }
  } // fn

  pub fn missing_frags_for<'a>(
    &'a self,
    seq: SequenceNumber,
  ) -> Box<dyn 'a + Iterator<Item = FragmentNumber>> {
    if let Some(ref fa) = self.fragment_assembler {
      fa.missing_frags_for(seq)
    } else {
      Box::new(iter::empty())
    }
  }
  pub fn is_partially_received(&self, seq: SequenceNumber) -> bool {
    if let Some(ref fa) = self.fragment_assembler {
      fa.is_partially_received(seq)
    } else {
      false
    }
  }

  // Advance ack_base as far as possible
  // This function should be called after the writer proxy has modified its
  // changes cache (for instance added a new received change) such that ack_base
  // could be advanced
  fn advance_ack_base(&mut self) {
    // Start searching from current ack_base
    let mut test_sn = self.ack_base;

    for (&sn, _what) in self.changes.range((Included(&self.ack_base), Unbounded)) {
      if sn == test_sn {
        // test_sn found from changes, ack_base can be set to test_sn + 1
        test_sn = test_sn + SequenceNumber::new(1);
      } else {
        // test_sn not found from changes, stop here
        break;
      }

      // The changes cache contains a string of consecutive sequence numbers from
      // ack_base-1 up to test_sn (excluded), so ack_base can be set to test_sn
      self.ack_base = test_sn;
    }
  }
} // impl
