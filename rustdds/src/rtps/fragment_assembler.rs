use std::{collections::BTreeMap, convert::TryInto, fmt, iter};

use bit_vec::BitVec;
use enumflags2::BitFlags;
use bytes::BytesMut;
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::{
  dds::ddsdata::DDSData,
  messages::submessages::{
    elements::serialized_payload::SerializedPayload,
    submessages::{DATAFRAG_Flags, DataFrag},
  },
  structure::{
    cache_change::ChangeKind,
    sequence_number::{FragmentNumber, SequenceNumber},
    time::Timestamp,
  },
};

// This is for the assembly of a single object
struct AssemblyBuffer {
  buffer_bytes: BytesMut,
  #[allow(dead_code)] // This module is still WiP
  fragment_count: usize,
  received_bitmap: BitVec,

  #[allow(dead_code)] // This module is still WiP
  created_time: Timestamp,
  modified_time: Timestamp,
}

impl AssemblyBuffer {
  pub fn new(datafrag: &DataFrag) -> Self {
    let data_size: usize = datafrag.data_size.try_into().unwrap();
    // We have unwrap here, but it will succeed as long as usize >= u32.
    let fragment_size: u16 = datafrag.fragment_size;
    debug!(
      "new AssemblyBuffer data_size={} frag_size={}",
      data_size, fragment_size
    );

    assert!(fragment_size as usize <= data_size); // This is validated at DataFrag deserializer
    assert!(fragment_size > 0); // This is validated at DataFrag deserializer
                                // Note: Technically RTPS spec allows fragment_size == 0.

    let mut buffer_bytes = BytesMut::with_capacity(data_size);
    buffer_bytes.resize(data_size, 0); // TODO: Can we replace this with faster (and unsafer) .set_len and live with
                                       // uninitialized data?

    let fragment_count = usize::from(datafrag.total_number_of_fragments());

    let now = Timestamp::now();

    Self {
      buffer_bytes,
      fragment_count,
      received_bitmap: BitVec::from_elem(fragment_count, false),
      created_time: now,
      modified_time: now,
    }
  }

  pub fn insert_frags(&mut self, datafrag: &DataFrag, frag_size: u16) {
    // TODO: Sanity checks? E.g. datafrag.fragment_size == frag_size
    let frag_size = usize::from(frag_size); // - payload_header;
    let frags_in_subm = usize::from(datafrag.fragments_in_submessage);
    let fragment_starting_num: usize = u32::from(datafrag.fragment_starting_num)
      .try_into()
      .unwrap();
    let start_frag_from_0 = fragment_starting_num - 1; // number of first fragment in this DataFrag, indexing from 0

    debug!(
      "insert_frags: datafrag.writer_sn = {:?}, frag_size = {:?}, datafrag.fragment_size = {:?}, datafrag.fragment_starting_num = {:?}, \
      datafrag.fragments_in_submessage = {:?}, datafrag.data_size = {:?}",
      datafrag.writer_sn, frag_size, datafrag.fragment_size, datafrag.fragment_starting_num,
      datafrag.fragments_in_submessage, datafrag.data_size
    );

    // unwrap: u32 should fit into usize
    let from_byte = start_frag_from_0 * frag_size;

    // Last fragment might be smaller than fragment size
    // Copy reported number of fragments, or as much data as there is, whichever
    // ends first.
    // And clamp to assembly buffer length to avoid buffer overrun.
    let to_before_byte = std::cmp::min(
      from_byte + std::cmp::min(frags_in_subm * frag_size, datafrag.serialized_payload.len()),
      self.buffer_bytes.len(),
    );
    let payload_size = to_before_byte - from_byte;

    // sanity check data size
    // Last fragment may be smaller than frags_in_subm * frag_size
    if fragment_starting_num < self.fragment_count
      && datafrag.serialized_payload.len() < frags_in_subm * frag_size
    {
      error!("Received DATAFRAG too small. fragment_starting_num={} out of fragment_count={}, frags_in_subm={}, frag_size={} but payload length ={}",
        fragment_starting_num, self.fragment_count, frags_in_subm, frag_size, datafrag.serialized_payload.len(), );
    }

    debug!(
      "insert_frags: from_byte = {:?}, to_before_byte = {:?}",
      from_byte, to_before_byte
    );

    debug!(
      "insert_frags: dataFrag.serializedPayload.len = {:?}",
      datafrag.serialized_payload.len()
    );

    self.buffer_bytes.as_mut()[from_byte..to_before_byte]
      .copy_from_slice(&datafrag.serialized_payload[..payload_size]);

    for f in 0..frags_in_subm {
      self.received_bitmap.set(start_frag_from_0 + f, true);
    }
    self.modified_time = Timestamp::now();
  }

  pub fn is_complete(&self) -> bool {
    self.received_bitmap.all() // return if all are received
  }
}

// Assembles fragments from a single (remote) Writer
// So there is only one sequence of SNs
pub(crate) struct FragmentAssembler {
  fragment_size: u16, // number of bytes per fragment. Each writer must select one constant value.
  assembly_buffers: BTreeMap<SequenceNumber, AssemblyBuffer>,
}

impl fmt::Debug for FragmentAssembler {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    f.debug_struct("FragmentAssembler - fields omitted")
      // insert field printing here, if you really need it.
      .finish()
  }
}

impl FragmentAssembler {
  pub fn new(fragment_size: u16) -> Self {
    debug!("new FragmentAssembler. frag_size = {}", fragment_size);
    Self {
      fragment_size,
      assembly_buffers: BTreeMap::new(),
    }
  }

  // Returns completed DDSData, when complete, and disposes the assembly buffer.
  pub fn new_datafrag(
    &mut self,
    datafrag: &DataFrag,
    flags: BitFlags<DATAFRAG_Flags>,
  ) -> Option<DDSData> {
    let writer_sn = datafrag.writer_sn;
    let frag_size = self.fragment_size;

    let abuf = self
      .assembly_buffers
      .entry(datafrag.writer_sn)
      .or_insert_with(|| AssemblyBuffer::new(datafrag));

    abuf.insert_frags(datafrag, frag_size);

    if abuf.is_complete() {
      debug!("new_datafrag: COMPLETED FRAGMENT");
      if let Some(abuf) = self.assembly_buffers.remove(&writer_sn) {
        // Return what we have assembled.
        let ser_data_or_key = SerializedPayload::from_bytes(&abuf.buffer_bytes.freeze())
          .map_or_else(
            |e| {
              error!("Deserializing SerializedPayload from DATAFRAG: {:?}", &e);
              None
            },
            Some,
          )?;
        let ddsdata = if flags.contains(DATAFRAG_Flags::Key) {
          DDSData::new_disposed_by_key(ChangeKind::NotAliveDisposed, ser_data_or_key)
        } else {
          // it is data
          DDSData::new(ser_data_or_key)
        };
        Some(ddsdata) // completed data from fragments
      } else {
        error!("Assembly buffer mysteriously lost");
        None
      }
    } else {
      debug!("new_dataFrag: FRAGMENT NOT COMPLETED YET");
      None
    }
  }

  // pub fn partially_received_sequence_numbers_iterator(&self) -> Box<dyn
  // Iterator<Item=SequenceNumber>> {   // Since we should only know about SNs
  // via DATAFRAG messages   // and AssemblyBuffers are removed immediately on
  // completion,   // the list should be just the list of current
  // AssemblyBuffers   self.assembly_buffers.keys()
  // }

  pub fn is_partially_received(&self, sn: SequenceNumber) -> bool {
    self.assembly_buffers.contains_key(&sn)
    // assembly buffers map contains a key (SN) if and only if we have some
    // frags but not all
  }

  pub fn missing_frags_for(
    &self,
    seq: SequenceNumber,
  ) -> Box<dyn '_ + Iterator<Item = FragmentNumber>> {
    match self.assembly_buffers.get(&seq) {
      None => Box::new(iter::empty()),
      Some(ab) => {
        let iter = (0..ab.fragment_count)
          .filter(move |f| !ab.received_bitmap.get(*f).unwrap_or(true))
          .map(|f| FragmentNumber::new((f + 1).try_into().unwrap()));
        Box::new(iter)
      }
    }
  }
}
