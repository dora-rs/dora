use std::collections::{BTreeMap, BTreeSet};

use bit_vec::BitVec;
#[allow(unused_imports)]
use log::{debug, error, trace, warn};

use crate::{
  dds::{participant::DomainParticipant, qos::QosPolicies},
  discovery::sedp_messages::DiscoveredReaderData,
  messages::submessages::submessage::AckSubmessage,
  network::constant::*,
  structure::{
    guid::{EntityId, GUID},
    locator::Locator,
    sequence_number::{FragmentNumber, FragmentNumberSet, SequenceNumber},
  },
};
use super::reader::ReaderIngredients;

#[derive(Debug, PartialEq, Eq, Clone)]
/// ReaderProxy class represents the information an RTPS StatefulWriter
/// maintains on each matched RTPS Reader
pub(crate) struct RtpsReaderProxy {
  ///Identifies the remote matched RTPS Reader that is represented by the
  /// ReaderProxy
  pub remote_reader_guid: GUID,
  /// Identifies the group to which the matched Reader belongs
  pub remote_group_entity_id: EntityId,
  /// List of unicast locators (transport, address, port combinations) that can
  /// be used to send messages to the matched RTPS Reader. The list may be empty
  pub unicast_locator_list: Vec<Locator>,
  /// List of multicast locators (transport, address, port combinations) that
  /// can be used to send messages to the matched RTPS Reader. The list may be
  /// empty
  pub multicast_locator_list: Vec<Locator>,

  /// Specifies whether the remote matched RTPS Reader expects in-line QoS to be
  /// sent along with any data.
  pub expects_in_line_qos: bool,
  /// Specifies whether the remote Reader is responsive to the Writer
  pub is_active: bool,

  // Reader has positively acked all SequenceNumbers _before_ this.
  // This is directly the same as readerSNState.base in ACKNACK submessage.
  pub all_acked_before: SequenceNumber,

  // List of SequenceNumbers to be sent to Reader. Both unsent and requested by ACKNACK.
  // TODO: Can we make this private?
  pub unsent_changes: BTreeSet<SequenceNumber>,

  // true = send repair data messages due to NACKs, buffer messages by DataWriter
  // false = send data messages directly from DataWriter
  pub repair_mode: bool,
  pub qos: QosPolicies,
  pub frags_requested: BTreeMap<SequenceNumber, BitVec>,
}

impl RtpsReaderProxy {
  pub fn new(remote_reader_guid: GUID, qos: QosPolicies) -> Self {
    Self {
      remote_reader_guid,
      remote_group_entity_id: EntityId::UNKNOWN,
      unicast_locator_list: Vec::default(),
      multicast_locator_list: Vec::default(),
      expects_in_line_qos: false,
      is_active: true,
      all_acked_before: SequenceNumber::zero(),
      unsent_changes: BTreeSet::new(),
      repair_mode: false,
      qos,
      frags_requested: BTreeMap::new(),
    }
  }

  pub fn qos(&self) -> &QosPolicies {
    &self.qos
  }

  pub fn from_reader(reader: &ReaderIngredients, domain_participant: &DomainParticipant) -> Self {
    let mut self_locators = domain_participant.self_locators(); // This clones a map of locator lists.
    let unicast_locator_list = self_locators
      .remove(&USER_TRAFFIC_LISTENER_TOKEN)
      .unwrap_or_default();
    let multicast_locator_list = self_locators
      .remove(&USER_TRAFFIC_MUL_LISTENER_TOKEN)
      .unwrap_or_default();

    Self {
      remote_reader_guid: reader.guid,
      remote_group_entity_id: EntityId::UNKNOWN, //TODO
      unicast_locator_list,
      multicast_locator_list,
      expects_in_line_qos: false,
      is_active: true,
      all_acked_before: SequenceNumber::zero(),
      unsent_changes: BTreeSet::new(),
      repair_mode: false,
      qos: reader.qos_policy.clone(),
      frags_requested: BTreeMap::new(),
    }
  }

  fn discovered_or_default(drd: &[Locator], default: &[Locator]) -> Vec<Locator> {
    if drd.is_empty() {
      default.to_vec()
    } else {
      drd.to_vec()
    }
  }

  pub fn from_discovered_reader_data(
    discovered_reader_data: &DiscoveredReaderData,
    default_unicast_locators: &[Locator],
    default_multicast_locators: &[Locator],
  ) -> Self {
    let unicast_locator_list = Self::discovered_or_default(
      &discovered_reader_data.reader_proxy.unicast_locator_list,
      default_unicast_locators,
    );
    let multicast_locator_list = Self::discovered_or_default(
      &discovered_reader_data.reader_proxy.multicast_locator_list,
      default_multicast_locators,
    );

    Self {
      remote_reader_guid: discovered_reader_data.reader_proxy.remote_reader_guid,
      remote_group_entity_id: EntityId::UNKNOWN, //TODO
      unicast_locator_list,
      multicast_locator_list,
      expects_in_line_qos: discovered_reader_data.reader_proxy.expects_inline_qos,
      is_active: true,
      all_acked_before: SequenceNumber::zero(),
      unsent_changes: BTreeSet::new(),
      repair_mode: false,
      qos: discovered_reader_data.subscription_topic_data.qos(),
      frags_requested: BTreeMap::new(),
    }
  }

  pub fn handle_ack_nack(
    &mut self,
    ack_submessage: &AckSubmessage,
    last_available: SequenceNumber,
  ) {
    match ack_submessage {
      AckSubmessage::AckNack(acknack) => {
        self.all_acked_before = acknack.reader_sn_state.base();
        // clean up unsent_changes:
        // The handy split_off function "Returns everything after the given key,
        // including the key."
        self.unsent_changes = self.unsent_changes.split_off(&self.all_acked_before);

        // Insert the requested changes.
        for nack_sn in acknack.reader_sn_state.iter() {
          self.unsent_changes.insert(nack_sn);
        }
        // sanity check
        if let Some(&high) = self.unsent_changes.iter().next_back() {
          if high > last_available {
            warn!(
              "ReaderProxy {:?} asks for {:?} but I have only up to {:?}. ACKNACK = {:?}",
              self.remote_reader_guid, self.unsent_changes, last_available, acknack
            );
          }
        }
      }

      AckSubmessage::NackFrag(_nack_frag) => {
        // TODO
        error!("NACKFRAG not implemented");
      }
    }
  }

  /// this should be called everytime a new CacheChange is set to RTPS writer
  /// HistoryCache
  pub fn notify_new_cache_change(&mut self, sequence_number: SequenceNumber) {
    if sequence_number == SequenceNumber::from(0) {
      error!(
        "new cache change with {:?}! bad! my GUID = {:?}",
        sequence_number, self.remote_reader_guid
      );
    }
    self.unsent_changes.insert(sequence_number);
  }

  pub fn acked_up_to_before(&self) -> SequenceNumber {
    self.all_acked_before
  }

  pub fn mark_frags_requested(&mut self, seq_num: SequenceNumber, frag_nums: &FragmentNumberSet) {
    let req_set = self
      .frags_requested
      .entry(seq_num)
      .or_insert_with(|| BitVec::with_capacity(64)); // default capacity out of hat

    if let Some(max_fn_requested) = req_set.iter().next_back() {
      // allocate more space if needed
      let max_fn_requested = usize::from(max_fn_requested);
      if max_fn_requested > req_set.len() {
        let growth_need = max_fn_requested - req_set.len();
        req_set.grow(growth_need, false);
      }
      for f in frag_nums.iter() {
        // -1 because FragmentNumbers start at 1
        req_set.set(usize::from(f) - 1, true);
      }
    } else {
      warn!(
        "mark_frags_requested: Empty set in NackFrag??? reader={:?} SN={:?}",
        self.remote_reader_guid, seq_num
      );
    }
  }

  // This just removes the FragmentNumber entry from the set.
  pub fn mark_frag_sent(&mut self, seq_num: SequenceNumber, frag_num: &FragmentNumber) {
    let mut frag_map_emptied = false;
    if let Some(frag_map) = self.frags_requested.get_mut(&seq_num) {
      // -1 because FragmentNumbers start at 1
      frag_map.set(usize::from(*frag_num) - 1, false);
      frag_map_emptied = frag_map.none();
    }
    if frag_map_emptied {
      self.frags_requested.remove(&seq_num);
    }
  }

  // Note: The current implementation produces an iterator that iterates only
  // over one fragmented sample, but the upper layer should detect that
  // there are still other fragmented samples requested (if any)
  // and continue sending.
  pub fn frags_requested_iterator(&self) -> FragBitVecIterator {
    match self.frags_requested.iter().next() {
      None => FragBitVecIterator::new(SequenceNumber::default(), BitVec::new()), // empty iterator
      Some((sn, bv)) => FragBitVecIterator::new(*sn, bv.clone()),
    }
  }

  pub fn repair_frags_requested(&self) -> bool {
    self.frags_requested.values().any(|rf| rf.any())
  }
}

pub struct FragBitVecIterator {
  sequence_number: SequenceNumber,
  frag_count: FragmentNumber,
  bitvec: BitVec,
}

impl FragBitVecIterator {
  pub fn new(sequence_number: SequenceNumber, bv: BitVec) -> FragBitVecIterator {
    FragBitVecIterator {
      sequence_number,
      frag_count: FragmentNumber::new(1),
      bitvec: bv,
    }
  }
}

impl Iterator for FragBitVecIterator {
  type Item = (SequenceNumber, FragmentNumber);

  fn next(&mut self) -> Option<Self::Item> {
    // f indexes from 1, like FragmentNumber
    let mut f = u32::from(self.frag_count);
    while (f as usize) <= self.bitvec.len() && self.bitvec.get((f - 1) as usize) == Some(false) {
      f += 1;
    }
    if (f as usize) > self.bitvec.len() {
      None
    } else {
      self.frag_count = FragmentNumber::new(f + 1);
      Some((self.sequence_number, FragmentNumber::new(f)))
    }
  }
}

// pub enum ChangeForReaderStatusKind {
//   UNSENT,
//   NACKNOWLEDGED,
//   REQUESTED,
//   ACKNOWLEDGED,
//   UNDERWAY,
// }

// ///The RTPS ChangeForReader is an association class that maintains
// information of a CacheChange in the RTPS ///Writer HistoryCache as it
// pertains to the RTPS Reader represented by the ReaderProxy pub struct
// RTPSChangeForReader {   ///Indicates the status of a CacheChange relative to
// the RTPS Reader represented by the ReaderProxy.   pub kind:
// ChangeForReaderStatusKind,   ///Indicates whether the change is relevant to
// the RTPS Reader represented by the ReaderProxy.   pub is_relevant: bool,
// }

// impl RTPSChangeForReader {
//   pub fn new() -> RTPSChangeForReader {
//     RTPSChangeForReader {
//       kind: ChangeForReaderStatusKind::UNSENT,
//       is_relevant: true,
//     }
//   }
// }
