use crate::{
  dds::{key::*, sampleinfo::*, with_key::datawriter::WriteOptions},
  structure::{
    cache_change::CacheChange, guid::GUID, sequence_number::SequenceNumber, time::Timestamp,
  },
};

/// A data sample received from a WITH_KEY Topic without the associated
/// metadata.
///
/// Replaces the use of `valid_data` flag in SampleInfo of DataSample from the
/// DDS spec.
///
/// Implements the methods `value`, `map_value`, `map_dispose` `unwrap` and
/// `as_ref` that correspond to methods of [`Result`](std::result::Result),
/// which had been previously used for this purpose.
#[derive(Clone, PartialEq, Debug)]
pub enum Sample<D, K> {
  Value(D),
  Dispose(K),
}

impl<D, K> Sample<D, K> {
  pub fn value(self) -> Option<D> {
    match self {
      Sample::Value(d) => Some(d),
      Sample::Dispose(_) => None,
    }
  }

  pub fn map_value<D2, F: FnOnce(D) -> D2>(self, op: F) -> Sample<D2, K> {
    match self {
      Sample::Value(d) => Sample::Value(op(d)),
      Sample::Dispose(k) => Sample::Dispose(k),
    }
  }

  pub fn map_dispose<K2, F: FnOnce(K) -> K2>(self, op: F) -> Sample<D, K2> {
    match self {
      Sample::Value(d) => Sample::Value(d),
      Sample::Dispose(k) => Sample::Dispose(op(k)),
    }
  }

  pub fn unwrap(self) -> D {
    match self {
      Sample::Value(d) => d,
      Sample::Dispose(_k) => panic!("Unwrap called on a Sample with no data"),
    }
  }

  pub const fn as_ref(&self) -> Sample<&D, &K> {
    match *self {
      Sample::Value(ref d) => Sample::Value(d),
      Sample::Dispose(ref k) => Sample::Dispose(k),
    }
  }
}

/// A data sample and its associated [metadata](`SampleInfo`) received from a
/// WITH_KEY Topic.
///
/// Note that [`no_key::DataSample`](crate::no_key::DataSample) and
/// [`with_key::DataSample`](crate::with_key::DataSample) are two different
/// structs.
///
/// We are using [`Sample`](crate::with_key::Sample) to replace the `valid_data`
/// flag from the DDS spec, because when `valid_data = false`, the application
/// should not be able to access any data.
///
/// Sample usage:
/// * `Sample::Value(d)` means `valid_data == true` and there is a sample `d`.
/// * `Sample::Dispose(k)` means `valid_data == false`, no sample exists, but
///   only a Key `k` and instance_state has changed.
///
/// See also DDS spec v1.4 Section 2.2.2.5.4.
#[derive(PartialEq, Debug)]
pub struct DataSample<D: Keyed> {
  pub(crate) sample_info: SampleInfo, // TODO: Can we somehow make this lazily evaluated?

  pub(crate) value: Sample<D, D::K>,
}

impl<D> DataSample<D>
where
  D: Keyed,
{
  pub(crate) fn new(sample_info: SampleInfo, value: Sample<D, D::K>) -> Self {
    Self { sample_info, value }
  }

  // convenience shorthand to get the key directly, without digging out the
  // "value"
  pub fn key(&self) -> D::K {
    match &self.value {
      Sample::Value(d) => d.key(),
      Sample::Dispose(k) => k.clone(),
    }
  } // fn

  pub fn value(&self) -> &Sample<D, D::K> {
    &self.value
  }

  pub fn into_value(self) -> Sample<D, D::K> {
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
pub struct DeserializedCacheChange<D: Keyed> {
  pub(crate) receive_instant: Timestamp, /* 8 bytes, to be used as unique key in internal data
                                          * structures */
  pub(crate) writer_guid: GUID,               // 8 bytes
  pub(crate) sequence_number: SequenceNumber, // 8 bytes
  pub(crate) write_options: WriteOptions,     // 16 bytes

  // the data sample (or key) itself is stored here
  pub(crate) sample: Sample<D, D::K>, /* TODO: make this a Box<> for easier detaching an
                                       * reattaching to somewhere else */
}

impl<D: Keyed> DeserializedCacheChange<D> {
  pub fn new(receive_instant: Timestamp, cc: &CacheChange, deserialized: Sample<D, D::K>) -> Self {
    DeserializedCacheChange {
      receive_instant,
      writer_guid: cc.writer_guid,
      sequence_number: cc.sequence_number,
      write_options: cc.write_options.clone(),
      sample: deserialized,
    }
  }
}
