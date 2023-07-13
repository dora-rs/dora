use enumflags2::{bitflags, BitFlags};

use crate::{
  dds::with_key::datawriter::WriteOptions,
  structure::{guid::GUID, rpc::SampleIdentity, sequence_number::SequenceNumber, time::Timestamp},
};

//use std::num::Zero; // unstable

/// indicates whether or not the corresponding data sample has already
/// been read by this `DataReader`.
///
/// > For each sample received, the middleware internally maintains a
/// > sample_state relative to each DataReader. The sample_state can either be
/// > READ or NOT_READ.
///
/// > The sample_state will, in general, be different for each sample in the
/// > collection returned by [`read()`](crate::with_key::DataReader::read())
/// > or [`take()`](crate::with_key::DataReader::take()).
///
/// See DDS spec v1.4 Section 2.2.2.5.4 and Section "2.2.2.5.1.2 Interpretation
/// of the SampleInfo sample_state".
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[bitflags]
#[repr(u32)] // DDS Spec 1.4 section 2.3.3 DCPS PSM : IDL defines these as "unsigned long",
             // so u32
pub enum SampleState {
  /// > indicates that the DataReader has already accessed that sample by means
  /// of `read()` ... or corresponding iterator.
  Read = 0b0001,
  /// > indicates that the DataReader has not accessed that sample before
  NotRead = 0b0010,
}

impl SampleState {
  /// Set that contains all possible states
  pub fn any() -> BitFlags<Self> {
    BitFlags::<Self>::all()
  }
}

/// Indicates if this data *instance* has been seen (viewed).
///
/// > For each instance (identified by the [key](crate::Key)), the middleware
/// > internally maintains a view_state relative to each DataReader. The
/// > view_state can either be NEW or NOT_NEW.
///
/// > The view_state available in the SampleInfo is a snapshot of the view_state
/// > of the instance relative to the DataReader used to access the samples at
/// > the time the collection was obtained (i.e., at the time read or take was
/// > called). The view_state is therefore the same for all samples in the
/// > returned collection that refer to the same instance.
/// > Once an instance has been detected as not having any "live" writers and
/// > all the samples associated with the instance are ‘taken’ from the
/// > DataReader, the middleware can reclaim all local resources regarding the
/// > instance. Future samples will be treated as 'never seen'.
///
/// Cf. [`SampleState`]
///
/// See DDS spec v.14 Section 2.2.2.5.1.8 Interpretation of the SampleInfo
/// view_state
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u32)]
#[bitflags]
pub enum ViewState {
  /// > indicates that either this is the first time that the DataReader has
  /// > ever accessed samples of that instance, or else that the DataReader
  /// > has accessed previous samples of the instance, but the instance has
  /// > since been reborn (i.e., become not-alive and then alive again).
  New = 0b0001,
  /// > indicates that the DataReader has already accessed samples of the same
  /// > instance and that the instance has not been reborn since
  NotNew = 0b0010,
}
impl ViewState {
  /// Set that contains all possible states
  pub fn any() -> BitFlags<Self> {
    BitFlags::<Self>::all()
  }
}

/// Is this data instance alive or not and why.
///
/// > The instance_state available in the SampleInfo is a snapshot of the
/// > instance_state of the instance at the time the collection was obtained
/// > (i.e., at the time read or take was called). The instance_state is
/// > therefore be the same for all samples in the returned collection that
/// > refer to the same instance.
///
/// DDS spec v1.4 Section "2.2.2.5.1.3 Interpretation of the SampleInfo
/// instance_state"
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u32)]
#[bitflags]
pub enum InstanceState {
  /// > indicates that (a) samples have been received for the instance, (b)
  /// > there are live DataWriter entities writing the instance, and (c) the
  /// > instance has not been explicitly disposed (or else more samples have
  /// > been received after it was disposed).
  Alive = 0b0001,

  /// > indicates the instance was explicitly disposed by a DataWriter by means
  /// > of the dispose operation.
  NotAliveDisposed = 0b0010,

  /// > indicates the instance has been declared as not-alive by the DataReader
  /// > because it  detected that there are no live DataWriter entities writing
  /// > that instance
  NotAliveNoWriters = 0b0100,
}

impl InstanceState {
  /// Set that contains all possible states
  pub fn any() -> BitFlags<Self> {
    BitFlags::<Self>::all()
  }
  /// Set that contains both not_alive states.
  pub fn not_alive() -> BitFlags<Self> {
    Self::NotAliveDisposed | Self::NotAliveNoWriters
  }
}

/// A double counter for counting how many times an instance as become Alive.
///
/// > For each instance the middleware internally maintains two counts: the
/// > disposed_generation_count and no_writers_generation_count, relative to
/// > each DataReader:
/// > * The disposed_generation_count and
/// > no_writers_generation_count are initialized to zero when the DataReader
/// > first detects the presence of a never-seen-before instance.
/// > * The disposed_generation_count is incremented each time the
/// > instance_state of the corresponding instance changes
/// > from NOT_ALIVE_DISPOSED to ALIVE.
/// > * The no_writers_generation_count is incremented each time the
/// > instance_state of the corresponding instance changes
/// > from NOT_ALIVE_NO_WRITERS to ALIVE.
/// > The disposed_generation_count and no_writers_generation_count available in
/// > the SampleInfo capture a snapshot of the corresponding counters at the
/// > time the sample was received.
///
/// See DDS spec v1.4 Section "2.2.2.5.1.5 Interpretation of the SampleInfo
/// disposed_generation_count and no_writers_generation_count"
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotAliveGenerationCounts {
  pub disposed_generation_count: i32,
  pub no_writers_generation_count: i32,
}

impl NotAliveGenerationCounts {
  /// Initial count value
  pub fn zero() -> Self {
    Self {
      disposed_generation_count: 0,
      no_writers_generation_count: 0,
    }
  }

  /// Marker value for "never accessed"
  pub fn sub_zero() -> Self {
    Self {
      disposed_generation_count: -1,
      no_writers_generation_count: -1,
    }
  }

  pub fn total(&self) -> i32 {
    self.disposed_generation_count + self.no_writers_generation_count
  }
}

/// SampleInfo is metadata attached to each received data sample.
/// It exists only at the receiving end of DDS, and is (mostly) generated by
/// DDS.
///
/// Some of the SampleInfo field description texts are quoted from the DDS
/// Specification.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SampleInfo {
  /// sample_state indicates whether or not the corresponding data sample has
  /// already been read through this DataReader.
  pub(crate) sample_state: SampleState,

  /// Indicates that either this is the first time that the DataReader has ever
  /// accessed samples of that instance, or else that the DataReader has
  /// accessed previous samples of the instance, but the instance has since
  /// been reborn (i.e., become not-alive and then alive again).
  pub(crate) view_state: ViewState,

  /// Indicates whether this instance is alive or not.
  /// The instance can be not alive either because some writer has actively
  /// disposed it, or there are no writers alive.
  pub(crate) instance_state: InstanceState,

  /// For each instance the middleware internally maintains these counts
  /// relative to each DataReader. The counts capture snapshots of the
  /// corresponding counters at the time the sample was received.
  pub(crate) generation_counts: NotAliveGenerationCounts,

  /// The ranks are are computed based solely on the actual samples in the
  /// ordered collection returned by the read or take.
  /// The sample_rank indicates the number of samples of the same instance that
  /// follow the current one in the collection.
  pub(crate) sample_rank: i32,

  /// The generation_rank indicates the difference in generations between the
  /// samples S and the Most Recent Sample of the same instance that appears In
  /// the returned Collection (MRSIC). It counts the number of times the
  /// instance transitioned from not-alive to alive in the time from the
  /// reception of the S to the  reception of MRSIC. The generation rank is
  /// computed with: generation_rank =
  /// (MRSIC.disposed_generation_count + MRSIC.no_writers_generation_count)
  ///   -- (S.disposed_generation_count + S.no_writers_generation_count)
  pub(crate) generation_rank: i32,

  /// The absolute_generation_rank indicates the difference in "generations"
  /// between sample S and the Most Recent Sample of the instance that the
  /// middlware has received (MRS). It counts the number of times the instance
  /// transitioned from not-alive to alive in the time from the reception of the
  /// S to the time when the read or take was called. absolute_generation_rank =
  /// (MRS.disposed_generation_count + MRS.no_writers_generation_count)
  ///   -- (S.disposed_generation_count + S.no_writers_generation_count)
  pub(crate) absolute_generation_rank: i32,

  pub(crate) write_options: WriteOptions,

  /// publication_handle identifies the DataWriter that modified
  /// the instance (i.e. wrote this sample)
  pub(crate) publication_handle: GUID,
  pub(crate) sequence_number: SequenceNumber,
}

#[allow(clippy::new_without_default)]
impl SampleInfo {
  /// Source timestamp is the timestamp that was supplied by the DataWriter
  /// that sent this sample. It is optional to timestamp samples when writing.
  pub fn source_timestamp(&self) -> Option<Timestamp> {
    self.write_options.source_timestamp
  }

  pub fn sample_state(&self) -> SampleState {
    self.sample_state
  }

  // pub fn set_sample_state(&mut self, sample_state: SampleState) {
  //   self.sample_state = sample_state
  // }

  pub fn view_state(&self) -> ViewState {
    self.view_state
  }

  // pub fn set_view_state(&mut self, view_state: ViewState) {
  //   self.view_state = view_state;
  // }

  pub fn instance_state(&self) -> InstanceState {
    self.instance_state
  }

  // pub fn set_instance_state(&mut self, instance_state: InstanceState) {
  //   self.instance_state = instance_state;
  // }

  pub fn disposed_generation_count(&self) -> i32 {
    self.generation_counts.disposed_generation_count
  }

  pub fn no_writers_generation_count(&self) -> i32 {
    self.generation_counts.no_writers_generation_count
  }

  pub fn sample_rank(&self) -> i32 {
    self.sample_rank
  }

  pub fn generation_rank(&self) -> i32 {
    self.generation_rank
  }

  pub fn absolute_generation_rank(&self) -> i32 {
    self.absolute_generation_rank
  }

  /// publication_handle identifies the DataWriter that modified
  /// the instance (i.e. wrote this sample)
  pub fn publication_handle(&self) -> GUID {
    self.publication_handle
  }

  pub fn writer_guid(&self) -> GUID {
    self.publication_handle
  }

  pub fn related_sample_identity(&self) -> Option<SampleIdentity> {
    self.write_options.related_sample_identity
  }

  pub fn sample_identity(&self) -> SampleIdentity {
    SampleIdentity {
      writer_guid: self.publication_handle,
      sequence_number: self.sequence_number,
    }
  }

  // pub fn set_publication_handle(&mut self, publication_handle: GUID) {
  //   self.publication_handle = publication_handle
  // }
}
