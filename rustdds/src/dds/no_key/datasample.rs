use crate::{
  dds::{
    no_key::wrappers::NoKeyWrapper,
    sampleinfo::SampleInfo,
    with_key,
    with_key::{
      datasample::{DataSample as WithKeyDataSample, Sample},
      datawriter::WriteOptions,
    },
  },
  rpc::SampleIdentity,
  structure::{
    cache_change::CacheChange, guid::GUID, sequence_number::SequenceNumber, time::Timestamp,
  },
};

/// A data sample and its associated [metadata](`SampleInfo`) received from a
/// NO_KEY Topic.
///
/// See DDS spec version 1.4 Section 2.2.2.5.4
///
/// Note that [`no_key::DataSample`](crate::no_key::DataSample) and
/// [`with_key::DataSample`](crate::with_key::DataSample) are two different
/// structs.
#[derive(PartialEq, Eq, Debug)]
pub struct DataSample<D> {
  pub(crate) sample_info: SampleInfo, // TODO: Can we somehow make this lazily evaluated?

  pub(crate) value: D,
}

impl<D> DataSample<D> {
  pub(crate) fn from_with_key(keyed: WithKeyDataSample<NoKeyWrapper<D>>) -> Option<Self> {
    match keyed.value {
      Sample::Value(kv) => Some(Self {
        sample_info: keyed.sample_info,
        value: kv.d,
      }),
      Sample::Dispose(_) => None,
    }
  }

  pub(crate) fn from_with_key_ref(
    keyed: WithKeyDataSample<&NoKeyWrapper<D>>,
  ) -> Option<DataSample<&D>> {
    match keyed.value {
      Sample::Value(kv) => Some(DataSample::<&D> {
        sample_info: keyed.sample_info,
        value: &kv.d,
      }),
      Sample::Dispose(_) => None,
    }
  }

  pub fn value(&self) -> &D {
    &self.value
  }

  pub fn into_value(self) -> D {
    self.value
  }

  pub fn sample_info(&self) -> &SampleInfo {
    &self.sample_info
  }

  pub fn sample_info_mut(&mut self) -> &mut SampleInfo {
    &mut self.sample_info
  }
} // impl

// This structure is used to communicate just deserialized samples
// from SimpleDatareader to DataReader
#[derive(Debug, Clone)]
pub struct DeserializedCacheChange<D> {
  pub receive_instant: Timestamp, /* 8 bytes, to be used as unique key in internal data
                                   * structures */
  pub writer_guid: GUID,               // 8 bytes
  pub sequence_number: SequenceNumber, // 8 bytes
  pub write_options: WriteOptions,     // 16 bytes

  // the data sample itself is stored here
  pub sample: D, /* TODO: make this a Box<> for easier detaching an
                  * reattaching to somewhere else */
}

impl<D> DeserializedCacheChange<D> {
  pub fn new(receive_instant: Timestamp, cc: &CacheChange, deserialized: D) -> Self {
    DeserializedCacheChange {
      receive_instant,
      writer_guid: cc.writer_guid,
      sequence_number: cc.sequence_number,
      write_options: cc.write_options.clone(),
      sample: deserialized,
    }
  }

  pub(crate) fn from_keyed(
    kdcc: with_key::datasample::DeserializedCacheChange<NoKeyWrapper<D>>,
  ) -> Option<Self> {
    match kdcc.sample {
      Sample::Value(sample) => Some(DeserializedCacheChange {
        receive_instant: kdcc.receive_instant,
        writer_guid: kdcc.writer_guid,
        sequence_number: kdcc.sequence_number,
        write_options: kdcc.write_options,
        sample: sample.d,
      }),
      Sample::Dispose(_key) => None,
    }
  }

  pub fn into_value(self) -> D {
    self.sample
  }

  pub fn source_timestamp(&self) -> Option<Timestamp> {
    self.write_options.source_timestamp
  }

  pub fn writer_guid(&self) -> GUID {
    self.writer_guid
  }

  pub fn related_sample_identity(&self) -> Option<SampleIdentity> {
    self.write_options.related_sample_identity
  }
}
