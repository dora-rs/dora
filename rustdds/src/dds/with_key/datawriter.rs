use std::{
  marker::PhantomData,
  pin::Pin,
  sync::{
    atomic::{AtomicI64, Ordering},
    Arc, Mutex,
  },
  task::{Context, Poll, Waker},
  time::{Duration, Instant},
};

use futures::{Future, Stream};
use mio_06::{self, Evented, Events, PollOpt, Ready, Token};
use mio_extras::channel::{self as mio_channel, SendError, TrySendError};
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::{
  dds::{
    adapters::with_key::SerializerAdapter,
    dds_entity::DDSEntity,
    ddsdata::DDSData,
    helpers::*,
    key,
    pubsub::Publisher,
    qos::{
      policy::{Liveliness, Reliability},
      HasQoSPolicy, QosPolicies,
    },
    result::{CreateResult, WriteError, WriteResult},
    statusevents::*,
    topic::Topic,
  },
  discovery::{discovery::DiscoveryCommand, sedp_messages::SubscriptionBuiltinTopicData},
  messages::submessages::elements::serialized_payload::SerializedPayload,
  rtps::writer::WriterCommand,
  serialization::CDRSerializerAdapter,
  structure::{
    cache_change::ChangeKind, duration, entity::RTPSEntity, guid::GUID, rpc::SampleIdentity,
    sequence_number::SequenceNumber, time::Timestamp,
  },
  Key, Keyed, TopicDescription,
};

// TODO: Move the write options and the builder type to some lower-level module
// to avoid circular dependencies.
#[derive(Debug, Default)]
pub struct WriteOptionsBuilder {
  related_sample_identity: Option<SampleIdentity>,
  source_timestamp: Option<Timestamp>,
}

impl WriteOptionsBuilder {
  pub fn new() -> Self {
    Self::default()
  }

  pub fn build(self) -> WriteOptions {
    WriteOptions {
      related_sample_identity: self.related_sample_identity,
      source_timestamp: self.source_timestamp,
    }
  }

  #[must_use]
  pub fn related_sample_identity(mut self, related_sample_identity: SampleIdentity) -> Self {
    self.related_sample_identity = Some(related_sample_identity);
    self
  }

  #[must_use]
  pub fn related_sample_identity_opt(
    mut self,
    related_sample_identity_opt: Option<SampleIdentity>,
  ) -> Self {
    self.related_sample_identity = related_sample_identity_opt;
    self
  }

  #[must_use]
  pub fn source_timestamp(mut self, source_timestamp: Timestamp) -> Self {
    self.source_timestamp = Some(source_timestamp);
    self
  }
}

/// Type to be used with write_with_options.
/// Use WriteOptionsBuilder to construct this.
#[derive(Clone, Eq, PartialEq, Ord, PartialOrd, Debug, Default)]
pub struct WriteOptions {
  pub(crate) related_sample_identity: Option<SampleIdentity>,
  pub(crate) source_timestamp: Option<Timestamp>,
  // future extension room fo other fields.
}

impl WriteOptions {
  pub fn related_sample_identity(&self) -> Option<SampleIdentity> {
    self.related_sample_identity
  }

  pub fn source_timestamp(&self) -> Option<Timestamp> {
    self.source_timestamp
  }
}

impl From<Option<Timestamp>> for WriteOptions {
  fn from(source_timestamp: Option<Timestamp>) -> Self {
    Self {
      related_sample_identity: None,
      source_timestamp,
    }
  }
}

/// Simplified type for CDR encoding
pub type DataWriterCdr<D> = DataWriter<D, CDRSerializerAdapter<D>>;

/// DDS DataWriter for keyed topics
///
/// # Examples
///
/// ```
/// use serde::{Serialize, Deserialize};
/// use rustdds::*;
/// use rustdds::with_key::DataWriter;
/// use rustdds::serialization::CDRSerializerAdapter;
///
/// let domain_participant = DomainParticipant::new(0).unwrap();
/// let qos = QosPolicyBuilder::new().build();
/// let publisher = domain_participant.create_publisher(&qos).unwrap();
///
/// #[derive(Serialize, Deserialize, Debug)]
/// struct SomeType { a: i32 }
/// impl Keyed for SomeType {
///   type K = i32;
///
///   fn key(&self) -> Self::K {
///     self.a
///   }
/// }
///
/// // WithKey is important
/// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
/// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(&topic, None);
/// ```
pub struct DataWriter<D: Keyed, SA: SerializerAdapter<D> = CDRSerializerAdapter<D>> {
  data_phantom: PhantomData<D>,
  ser_phantom: PhantomData<SA>,
  my_publisher: Publisher,
  my_topic: Topic,
  qos_policy: QosPolicies,
  my_guid: GUID,
  cc_upload: mio_channel::SyncSender<WriterCommand>,
  cc_upload_waker: Arc<Mutex<Option<Waker>>>,
  discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
  status_receiver: StatusReceiver<DataWriterStatus>,
  available_sequence_number: AtomicI64,
}

impl<D, SA> Drop for DataWriter<D, SA>
where
  D: Keyed,
  SA: SerializerAdapter<D>,
{
  fn drop(&mut self) {
    // Tell Publisher to drop the corresponding RTPS Writer
    self.my_publisher.remove_writer(self.my_guid);

    // Notify Discovery that we are no longer
    match self
      .discovery_command
      .send(DiscoveryCommand::RemoveLocalWriter { guid: self.guid() })
    {
      Ok(_) => {}

      // This is fairly normal at shutdown, as the other end is down already.
      Err(SendError::Disconnected(_cmd)) => {
        debug!("Failed to send REMOVE_LOCAL_WRITER DiscoveryCommand: Disconnected.");
      }
      // other errors must be taken more seriously
      Err(e) => error!(
        "Failed to send REMOVE_LOCAL_WRITER DiscoveryCommand. {:?}",
        e
      ),
    }
  }
}

impl<D, SA> DataWriter<D, SA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  SA: SerializerAdapter<D>,
{
  #[allow(clippy::too_many_arguments)]
  pub(crate) fn new(
    publisher: Publisher,
    topic: Topic,
    qos: QosPolicies,
    guid: GUID,
    cc_upload: mio_channel::SyncSender<WriterCommand>,
    cc_upload_waker: Arc<Mutex<Option<Waker>>>,
    discovery_command: mio_channel::SyncSender<DiscoveryCommand>,
    status_receiver_rec: StatusChannelReceiver<DataWriterStatus>,
  ) -> CreateResult<Self> {
    if let Some(lv) = qos.liveliness {
      match lv {
        Liveliness::Automatic { .. } | Liveliness::ManualByTopic { .. } => (),
        Liveliness::ManualByParticipant { .. } => {
          if let Err(e) = discovery_command.send(DiscoveryCommand::ManualAssertLiveliness) {
            error!("Failed to send DiscoveryCommand - Refresh. {e:?}");
          }
        }
      }
    };
    Ok(Self {
      data_phantom: PhantomData,
      ser_phantom: PhantomData,
      my_publisher: publisher,
      my_topic: topic,
      qos_policy: qos,
      my_guid: guid,
      cc_upload,
      cc_upload_waker,
      discovery_command,
      status_receiver: StatusReceiver::new(status_receiver_rec),
      available_sequence_number: AtomicI64::new(1), // valid numbering starts from 1
    })
  }

  fn next_sequence_number(&self) -> SequenceNumber {
    SequenceNumber::from(
      self
        .available_sequence_number
        .fetch_add(1, Ordering::Relaxed),
    )
  }

  fn undo_sequence_number(&self) {
    self
      .available_sequence_number
      .fetch_sub(1, Ordering::Relaxed);
  }

  // This one function provides both get_matched_subscriptions and
  // get_matched_subscription_data TODO: Maybe we could return references to the
  // subscription data to avoid copying? But then what if the result set changes
  // while the application processes it?

  /// Manually refreshes liveliness if QoS allows it
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// data_writer.refresh_manual_liveliness();
  /// ```

  // TODO: What is this function? To what part of DDS spec does it correspond to?
  pub fn refresh_manual_liveliness(&self) {
    if let Some(lv) = self.qos().liveliness {
      match lv {
        Liveliness::Automatic { .. } | Liveliness::ManualByTopic { .. } => (),
        Liveliness::ManualByParticipant { .. } => {
          if let Err(e) = self
            .discovery_command
            .send(DiscoveryCommand::ManualAssertLiveliness)
          {
            error!("Failed to send DiscoveryCommand - Refresh. {e:?}");
          }
        }
      }
    };
  }

  /// Writes single data instance to a topic.
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// let some_data = SomeType { a: 1 };
  /// data_writer.write(some_data, None).unwrap();
  /// ```
  pub fn write(&self, data: D, source_timestamp: Option<Timestamp>) -> WriteResult<(), D> {
    self.write_with_options(data, WriteOptions::from(source_timestamp))?;
    Ok(())
  }

  pub fn write_with_options(
    &self,
    data: D,
    write_options: WriteOptions,
  ) -> WriteResult<SampleIdentity, D> {
    // serialize
    let send_buffer = match SA::to_bytes(&data) {
      Ok(b) => b,
      Err(e) => {
        return Err(WriteError::Serialization {
          reason: format!("{e}"),
          data,
        })
      }
    };

    let ddsdata = DDSData::new(SerializedPayload::new_from_bytes(
      SA::output_encoding(),
      send_buffer,
    ));
    let sequence_number = self.next_sequence_number();
    let writer_command = WriterCommand::DDSData {
      ddsdata,
      write_options,
      sequence_number,
    };

    let timeout = self.qos().reliable_max_blocking_time();

    match try_send_timeout(&self.cc_upload, writer_command, timeout) {
      Ok(_) => {
        self.refresh_manual_liveliness();
        Ok(SampleIdentity {
          writer_guid: self.my_guid,
          sequence_number,
        })
      }
      Err(TrySendError::Full(_writer_command)) => {
        warn!(
          "Write timed out: topic={:?}  timeout={:?}",
          self.my_topic.name(),
          timeout,
        );
        self.undo_sequence_number();
        Err(WriteError::WouldBlock { data })
      }
      Err(TrySendError::Disconnected(_)) => {
        self.undo_sequence_number();
        Err(WriteError::Poisoned {
          reason: "Cannot send to Writer".to_string(),
          data,
        })
      }
      Err(TrySendError::Io(e)) => {
        self.undo_sequence_number();
        Err(e.into())
      }
    }
  }

  /// This operation blocks the calling thread until either all data written by
  /// the reliable DataWriter entities is acknowledged by all
  /// matched reliable DataReader entities, or else the duration specified by
  /// the `max_wait` parameter elapses, whichever happens first.
  ///
  /// See DDS Spec 1.4 Section 2.2.2.4.1.12 wait_for_acknowledgments.
  ///
  /// If this DataWriter is not set to Reliable, or there are no matched
  /// DataReaders with Reliable QoS, the call succeeds immediately.
  ///
  /// Return values
  /// * `Ok(true)` - all acknowledged
  /// * `Ok(false)`- timed out waiting for acknowledgments
  /// * `Err(_)` - something went wrong
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// let some_data = SomeType { a: 1 };
  /// data_writer.write(some_data, None).unwrap();
  /// data_writer.wait_for_acknowledgments(std::time::Duration::from_millis(100));
  /// ```
  pub fn wait_for_acknowledgments(&self, max_wait: Duration) -> WriteResult<bool, ()> {
    match &self.qos_policy.reliability {
      None | Some(Reliability::BestEffort) => Ok(true),
      Some(Reliability::Reliable { .. }) => {
        let (acked_sender, acked_receiver) = sync_status_channel::<()>(1)?;
        let poll = mio_06::Poll::new()?;
        poll.register(
          acked_receiver.as_evented(),
          Token(0),
          Ready::readable(),
          PollOpt::edge(),
        )?;
        self
          .cc_upload
          .try_send(WriterCommand::WaitForAcknowledgments {
            all_acked: acked_sender,
          })
          .unwrap_or_else(|e| {
            warn!("wait_for_acknowledgments: cannot initiate waiting. This will timeout. {e}");
          });

        let mut events = Events::with_capacity(1);
        poll.poll(&mut events, Some(max_wait))?;
        if let Some(_event) = events.iter().next() {
          match acked_receiver.try_recv() {
            Ok(_) => Ok(true), // got token
            Err(e) => {
              warn!("wait_for_acknowledgments - Spurious poll event? - {e}");
              Ok(false) // TODO: We could also loop here
            }
          }
        } else {
          // no token, so presumably timed out
          Ok(false)
        }
      }
    } // match
  }

  /*
  /// Gets mio Receiver for all status changes
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(topic, None).unwrap();
  ///
  /// // Some status has changed
  ///
  /// while let Ok(sc) = data_writer.get_status_listener().try_recv() {
  ///   // do something
  /// }
  /// ```
  pub fn get_status_listener(&self) -> &Receiver<StatusChange> {
    match self
      .cc_upload
      .try_send(WriterCommand::ResetOfferedDeadlineMissedStatus {
        writer_guid: self.guid(),
      }) {
      Ok(_) => (),
      Err(e) => error!("Unable to send ResetOfferedDeadlineMissedStatus. {e:?}"),
    };
    &self.status_receiver
  }

  /// Unimplemented. <b>Do not use</b>.
  ///
  /// # Examples
  ///
  /// ```no_run
  // TODO: enable when functional
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(topic, None).unwrap();
  ///
  /// // Liveliness lost status has changed
  ///
  /// if let Ok(lls) = data_writer.get_liveliness_lost_status() {
  ///   // do something
  /// }
  /// ```
  pub fn get_liveliness_lost_status(&self) -> Result<LivelinessLostStatus> {
    todo!()
  }

  /// Should get latest offered deadline missed status. <b>Do not use yet</b> use `get_status_lister` instead for the moment.
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(topic, None).unwrap();
  ///
  /// // Deadline missed status has changed
  ///
  /// if let Ok(odms) = data_writer.get_offered_deadline_missed_status() {
  ///   // do something
  /// }
  /// ```
  pub fn get_offered_deadline_missed_status(&self) -> Result<OfferedDeadlineMissedStatus> {
    let mut fstatus = OfferedDeadlineMissedStatus::new();
    while let Ok(status) = self.status_receiver.try_recv() {
      match status {
        StatusChange::OfferedDeadlineMissedStatus(status) => fstatus = status,
  // TODO: possibly save old statuses
        _ => (),
      }
    }

    match self
      .cc_upload
      .try_send(WriterCommand::ResetOfferedDeadlineMissedStatus {
        writer_guid: self.guid(),
      }) {
      Ok(_) => (),
      Err(e) => error!("Unable to send ResetOfferedDeadlineMissedStatus. {e:?}"),
    };

    Ok(fstatus)
  }

  /// Unimplemented. <b>Do not use</b>.
  ///
  /// # Examples
  ///
  /// ```no_run
  // TODO: enable when functional
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(topic, None).unwrap();
  ///
  /// // Liveliness lost status has changed
  ///
  /// if let Ok(oiqs) = data_writer.get_offered_incompatible_qos_status() {
  ///   // do something
  /// }
  /// ```
  pub fn get_offered_incompatible_qos_status(&self) -> Result<OfferedIncompatibleQosStatus> {
    todo!()
  }

  /// Unimplemented. <b>Do not use</b>.
  ///
  /// # Examples
  ///
  /// ```no_run
  // TODO: enable when functional
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(topic, None).unwrap();
  ///
  /// // Liveliness lost status has changed
  ///
  /// if let Ok(pms) = data_writer.get_publication_matched_status() {
  ///   // do something
  /// }
  /// ```
  pub fn get_publication_matched_status(&self) -> Result<PublicationMatchedStatus> {
    todo!()
  }

  */

  /// Topic assigned to this DataWriter
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// assert_eq!(data_writer.topic(), &topic);
  /// ```
  pub fn topic(&self) -> &Topic {
    &self.my_topic
  }

  /// Publisher assigned to this DataWriter
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// assert_eq!(data_writer.publisher(), &publisher);
  pub fn publisher(&self) -> &Publisher {
    &self.my_publisher
  }

  /// Manually asserts liveliness (use this instead of refresh) according to QoS
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// data_writer.assert_liveliness().unwrap();
  /// ```

  // TODO: This cannot really fail, so could change type to () (alternatively,
  // make send error visible) TODO: Better make send failure visible, so
  // application can see if Discovery has failed.
  pub fn assert_liveliness(&self) -> WriteResult<(), ()> {
    self.refresh_manual_liveliness();

    match self.qos().liveliness {
      Some(Liveliness::ManualByTopic { lease_duration: _ }) => {
        self
          .discovery_command
          .send(DiscoveryCommand::AssertTopicLiveliness {
            writer_guid: self.guid(),
            manual_assertion: true, // by definition of this function
          })
          .unwrap_or_else(|e| error!("assert_liveness - Failed to send DiscoveryCommand. {e:?}"));
      }
      _other => (),
    }
    Ok(())
  }

  /// Unimplemented. <b>Do not use</b>.
  ///
  /// # Examples
  ///
  /// ```no_run
  // TODO: enable when available
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32 }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(),
  /// "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType,
  /// CDRSerializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// for sub in data_writer.get_matched_subscriptions().iter() {
  ///   // do something
  /// }
  pub fn get_matched_subscriptions(&self) -> Vec<SubscriptionBuiltinTopicData> {
    todo!()
  }

  /// Disposes data instance with specified key
  ///
  /// # Arguments
  ///
  /// * `key` - Key of the instance
  /// * `source_timestamp` - DDS source timestamp (None uses now as time as
  ///   specified in DDS spec)
  ///
  /// # Examples
  ///
  /// ```
  /// # use serde::{Serialize, Deserialize};
  /// # use rustdds::*;
  /// # use rustdds::with_key::DataWriter;
  /// # use rustdds::serialization::CDRSerializerAdapter;
  /// #
  /// let domain_participant = DomainParticipant::new(0).unwrap();
  /// let qos = QosPolicyBuilder::new().build();
  /// let publisher = domain_participant.create_publisher(&qos).unwrap();
  ///
  /// #[derive(Serialize, Deserialize, Debug)]
  /// struct SomeType { a: i32, val: usize }
  /// impl Keyed for SomeType {
  ///   type K = i32;
  ///
  ///   fn key(&self) -> Self::K {
  ///     self.a
  ///   }
  /// }
  ///
  /// // WithKey is important
  /// let topic = domain_participant.create_topic("some_topic".to_string(), "SomeType".to_string(), &qos, TopicKind::WithKey).unwrap();
  /// let data_writer = publisher.create_datawriter::<SomeType, CDRSerializerAdapter<_>>(&topic, None).unwrap();
  ///
  /// let some_data_1_1 = SomeType { a: 1, val: 3};
  /// let some_data_1_2 = SomeType { a: 1, val: 4};
  /// // different key
  /// let some_data_2_1 = SomeType { a: 2, val: 5};
  /// let some_data_2_2 = SomeType { a: 2, val: 6};
  ///
  /// data_writer.write(some_data_1_1, None).unwrap();
  /// data_writer.write(some_data_1_2, None).unwrap();
  /// data_writer.write(some_data_2_1, None).unwrap();
  /// data_writer.write(some_data_2_2, None).unwrap();
  ///
  /// // disposes both some_data_1_1 and some_data_1_2. They are no longer offered by this writer to this topic.
  /// data_writer.dispose(&1, None).unwrap();
  /// ```
  pub fn dispose(
    &self,
    key: &<D as Keyed>::K,
    source_timestamp: Option<Timestamp>,
  ) -> WriteResult<(), ()> {
    let send_buffer = SA::key_to_bytes(key).map_err(|e| WriteError::Serialization {
      reason: format!("{e}"),
      data: (),
    })?; // serialize key

    let ddsdata = DDSData::new_disposed_by_key(
      ChangeKind::NotAliveDisposed,
      SerializedPayload::new_from_bytes(SA::output_encoding(), send_buffer),
    );
    self
      .cc_upload
      .send(WriterCommand::DDSData {
        ddsdata,
        write_options: WriteOptions::from(source_timestamp),
        sequence_number: self.next_sequence_number(),
      })
      .map_err(|e| {
        self.undo_sequence_number();
        WriteError::Serialization {
          reason: format!("{e}"),
          data: (),
        }
      })?;

    self.refresh_manual_liveliness();
    Ok(())
  }

  pub fn as_async_event_stream(&self) -> StatusReceiverStream<DataWriterStatus> {
    self.status_receiver.as_async_stream()
  }
}

impl<D, SA> StatusEvented<DataWriterStatus> for DataWriter<D, SA>
where
  D: Keyed,
  SA: SerializerAdapter<D>,
{
  fn as_status_evented(&mut self) -> &dyn Evented {
    self.status_receiver.as_status_evented()
  }

  fn as_status_source(&mut self) -> &mut dyn mio_08::event::Source {
    self.status_receiver.as_status_source()
  }

  fn try_recv_status(&self) -> Option<DataWriterStatus> {
    self.status_receiver.try_recv_status()
  }
}

impl<D, SA> RTPSEntity for DataWriter<D, SA>
where
  D: Keyed,
  SA: SerializerAdapter<D>,
{
  fn guid(&self) -> GUID {
    self.my_guid
  }
}

impl<D, SA> HasQoSPolicy for DataWriter<D, SA>
where
  D: Keyed,
  SA: SerializerAdapter<D>,
{
  fn qos(&self) -> QosPolicies {
    self.qos_policy.clone()
  }
}

impl<D, SA> DDSEntity for DataWriter<D, SA>
where
  D: Keyed,
  SA: SerializerAdapter<D>,
{
}

//-------------------------------------------------------------------------------
// async writing implementation
//

// A future for an asynchronous write operation
pub struct AsyncWrite<'a, D, SA>
where
  D: Keyed,
  <D as key::Keyed>::K: Key,
  SA: SerializerAdapter<D>,
{
  writer: &'a DataWriter<D, SA>,
  writer_command: Option<WriterCommand>,
  sequence_number: SequenceNumber,
  timeout: Option<duration::Duration>,
  timeout_instant: Instant,
  sample: Option<D>,
}

// This is required, because AsyncWrite contains "D".
// TODO: Is it ok to promise Unpin here?
impl<'a, D, SA> Unpin for AsyncWrite<'a, D, SA>
where
  D: Keyed,
  <D as key::Keyed>::K: Key,
  SA: SerializerAdapter<D>,
{
}

impl<'a, D, SA> Future for AsyncWrite<'a, D, SA>
where
  D: Keyed,
  <D as key::Keyed>::K: Key,
  SA: SerializerAdapter<D>,
{
  type Output = WriteResult<SampleIdentity, D>;

  fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
    match self.writer_command.take() {
      Some(wc) => {
        match self.writer.cc_upload.try_send(wc) {
          Ok(()) => {
            self.writer.refresh_manual_liveliness();
            Poll::Ready(Ok(SampleIdentity {
              writer_guid: self.writer.my_guid,
              sequence_number: self.sequence_number,
            }))
          }
          Err(TrySendError::Full(wc)) => {
            *self.writer.cc_upload_waker.lock().unwrap() = Some(cx.waker().clone());
            if Instant::now() < self.timeout_instant {
              // Put our command back
              self.writer_command = Some(wc);
              Poll::Pending
            } else {
              // TODO: unwrap
              Poll::Ready(Err(WriteError::WouldBlock {
                data: self.sample.take().unwrap(),
              }))
            }
          }
          Err(other_err) => {
            warn!(
              "Failed to write new data: topic={:?}  reason={:?}  timeout={:?}",
              self.writer.my_topic.name(),
              other_err,
              self.timeout
            );
            // TODO: Is this (undo) the right thing to do, if there are
            // several futures in progress? (Can this result in confused numbering?)
            self.writer.undo_sequence_number();
            Poll::Ready(Err(WriteError::Poisoned {
              reason: format!("{other_err}"),
              data: self.sample.take().unwrap(),
            }))
          }
        }
      }
      None => {
        // the dog ate my homework
        // this should not happen
        Poll::Ready(Err(WriteError::Internal {
          reason: "someone stole my WriterCommand".to_owned(),
        }))
      }
    }
  }
}

// A future for an asynchronous operation of waiting for acknowledgements.
// Handles both the sending of the WaitForAcknowledgments command and
// the waiting for the acknowledgements.
pub enum AsyncWaitForAcknowledgments<'a, D, SA>
where
  D: Keyed,
  SA: SerializerAdapter<D>,
{
  Waiting {
    ack_wait_receiver: StatusChannelReceiver<()>,
  },
  Done,
  WaitingSendCommand {
    writer: &'a DataWriter<D, SA>,
    ack_wait_receiver: StatusChannelReceiver<()>,
    ack_wait_sender: StatusChannelSender<()>,
  },
  Fail(WriteError<()>),
}

impl<'a, D, SA> Future for AsyncWaitForAcknowledgments<'a, D, SA>
where
  D: Keyed,
  <D as key::Keyed>::K: Key,
  SA: SerializerAdapter<D>,
{
  type Output = WriteResult<bool, ()>;

  fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
    match *self {
      AsyncWaitForAcknowledgments::Done => Poll::Ready(Ok(true)),
      AsyncWaitForAcknowledgments::Fail(_) => {
        let mut dummy = AsyncWaitForAcknowledgments::Done;
        core::mem::swap(&mut dummy, &mut self);
        match dummy {
          AsyncWaitForAcknowledgments::Fail(e) => Poll::Ready(Err(e)),
          _ => unreachable!(),
        }
      }
      AsyncWaitForAcknowledgments::Waiting {
        ref ack_wait_receiver,
      } => {
        match Pin::new(&mut ack_wait_receiver.as_async_stream()).poll_next(cx) {
          Poll::Pending => Poll::Pending,

          // this should not really happen, but let's judge that as a "no"
          Poll::Ready(None) => Poll::Ready(Ok(false)),

          Poll::Ready(Some(Err(_read_error)))
            // RecvError means the sending side has disconnected.
            // We assume this would only be because the event loop thread is dead.
            => Poll::Ready(Err(WriteError::Poisoned{ reason: "RecvError".to_string(), data:()})),

          Poll::Ready(Some(Ok(()))) => Poll::Ready(Ok(true)),
          // There is no timeout support here, so we never really
          // return Ok(false)
        }
      }
      AsyncWaitForAcknowledgments::WaitingSendCommand { .. } => {
        let mut dummy = AsyncWaitForAcknowledgments::Done;
        core::mem::swap(&mut dummy, &mut self);
        let (writer, ack_wait_receiver, ack_wait_sender) = match dummy {
          AsyncWaitForAcknowledgments::WaitingSendCommand {
            writer,
            ack_wait_receiver,
            ack_wait_sender,
          } => (writer, ack_wait_receiver, ack_wait_sender),
          _ => unreachable!(),
        };

        match writer
          .cc_upload
          .try_send(WriterCommand::WaitForAcknowledgments {
            all_acked: ack_wait_sender,
          }) {
          Ok(()) => {
            *self = AsyncWaitForAcknowledgments::Waiting { ack_wait_receiver };
            Poll::Pending
          }

          Err(TrySendError::Full(WriterCommand::WaitForAcknowledgments {
            all_acked: ack_wait_sender,
          })) => {
            *self = AsyncWaitForAcknowledgments::WaitingSendCommand {
              writer,
              ack_wait_receiver,
              ack_wait_sender,
            };
            Poll::Pending
          }
          Err(TrySendError::Full(_other_writer_command)) =>
          // We are sending WaitForAcknowledgments, so the channel
          // should return only that, if any.
          {
            unreachable!()
          }
          Err(e) => Poll::Ready(Err(WriteError::Poisoned {
            reason: format!("{e}"),
            data: (),
          })),
        }
      }
    }
  }
}

impl<D, SA> DataWriter<D, SA>
where
  D: Keyed,
  <D as Keyed>::K: Key,
  SA: SerializerAdapter<D>,
{
  pub async fn async_write(
    &self,
    data: D,
    source_timestamp: Option<Timestamp>,
  ) -> WriteResult<(), D> {
    match self
      .async_write_with_options(data, WriteOptions::from(source_timestamp))
      .await
    {
      Ok(_sample_identity) => Ok(()),
      Err(e) => Err(e),
    }
  }

  pub async fn async_write_with_options(
    &self,
    data: D,
    write_options: WriteOptions,
  ) -> WriteResult<SampleIdentity, D> {
    // Construct a future for an async write operation and await for its completion

    let send_buffer = match SA::to_bytes(&data) {
      Ok(s) => s,
      Err(e) => {
        return Err(WriteError::Serialization {
          reason: format!("{e}"),
          data,
        })
      }
    };

    let dds_data = DDSData::new(SerializedPayload::new_from_bytes(
      SA::output_encoding(),
      send_buffer,
    ));
    let sequence_number = self.next_sequence_number();
    let writer_command = WriterCommand::DDSData {
      ddsdata: dds_data,
      write_options,
      sequence_number,
    };

    let timeout = self.qos().reliable_max_blocking_time();

    let write_future = AsyncWrite {
      writer: self,
      writer_command: Some(writer_command),
      sequence_number,
      timeout,
      timeout_instant: std::time::Instant::now()
        + timeout
          .map(|t| t.to_std())
          .unwrap_or(crate::dds::helpers::TIMEOUT_FALLBACK.to_std()),
      sample: Some(data),
    };
    write_future.await
  }

  /// Like the synchronous version.
  /// But there is no timeout. Use asyncs to bring your own timeout.
  pub async fn async_wait_for_acknowledgments(&self) -> WriteResult<bool, ()> {
    match &self.qos_policy.reliability {
      None | Some(Reliability::BestEffort) => Ok(true),
      Some(Reliability::Reliable { .. }) => {
        // Construct a future for an async operation to first send the
        // WaitForAcknowledgments command and then wait for the
        // acknowledgements. Await for this future to complete.

        let (ack_wait_sender, ack_wait_receiver) = sync_status_channel::<()>(1).unwrap(); // TODO: remove unwrap

        let async_ack_wait = AsyncWaitForAcknowledgments::WaitingSendCommand {
          writer: self,
          ack_wait_receiver,
          ack_wait_sender,
        };
        async_ack_wait.await
      }
    }
  }
} // impl

#[cfg(test)]
mod tests {
  use std::thread;

  use byteorder::LittleEndian;
  use log::info;

  use super::*;
  use crate::{
    dds::{key::Keyed, participant::DomainParticipant},
    serialization::cdr_serializer::CDRSerializerAdapter,
    structure::topic_kind::TopicKind,
    test::random_data::*,
  };

  #[test]
  fn dw_write_test() {
    let domain_participant = DomainParticipant::new(0).expect("Publisher creation failed!");
    let qos = QosPolicies::qos_none();
    let _default_dw_qos = QosPolicies::qos_none();
    let publisher = domain_participant
      .create_publisher(&qos)
      .expect("Failed to create publisher");
    let topic = domain_participant
      .create_topic(
        "Aasii".to_string(),
        "Huh?".to_string(),
        &qos,
        TopicKind::WithKey,
      )
      .expect("Failed to create topic");

    let data_writer: DataWriter<RandomData, CDRSerializerAdapter<RandomData, LittleEndian>> =
      publisher
        .create_datawriter(&topic, None)
        .expect("Failed to create datawriter");

    let mut data = RandomData {
      a: 4,
      b: "Fobar".to_string(),
    };

    data_writer
      .write(data.clone(), None)
      .expect("Unable to write data");

    data.a = 5;
    let timestamp = Timestamp::now();
    data_writer
      .write(data, Some(timestamp))
      .expect("Unable to write data with timestamp");

    // TODO: verify that data is sent/written correctly
    // TODO: write also with timestamp
  }

  #[test]
  fn dw_dispose_test() {
    let domain_participant = DomainParticipant::new(0).expect("Publisher creation failed!");
    let qos = QosPolicies::qos_none();
    let publisher = domain_participant
      .create_publisher(&qos)
      .expect("Failed to create publisher");
    let topic = domain_participant
      .create_topic(
        "Aasii".to_string(),
        "Huh?".to_string(),
        &qos,
        TopicKind::WithKey,
      )
      .expect("Failed to create topic");

    let data_writer: DataWriter<RandomData, CDRSerializerAdapter<RandomData, LittleEndian>> =
      publisher
        .create_datawriter(&topic, None)
        .expect("Failed to create datawriter");

    let data = RandomData {
      a: 4,
      b: "Fobar".to_string(),
    };

    let key = &data.key().hash_key();
    info!("key: {:?}", key);

    data_writer
      .write(data.clone(), None)
      .expect("Unable to write data");

    thread::sleep(Duration::from_millis(100));
    data_writer
      .dispose(&data.key(), None)
      .expect("Unable to dispose data");

    // TODO: verify that dispose is sent correctly
  }

  #[test]
  fn dw_wait_for_ack_test() {
    let domain_participant = DomainParticipant::new(0).expect("Participant creation failed!");
    let qos = QosPolicies::qos_none();
    let publisher = domain_participant
      .create_publisher(&qos)
      .expect("Failed to create publisher");
    let topic = domain_participant
      .create_topic(
        "Aasii".to_string(),
        "Huh?".to_string(),
        &qos,
        TopicKind::WithKey,
      )
      .expect("Failed to create topic");

    let data_writer: DataWriter<RandomData, CDRSerializerAdapter<RandomData, LittleEndian>> =
      publisher
        .create_datawriter(&topic, None)
        .expect("Failed to create datawriter");

    let data = RandomData {
      a: 4,
      b: "Fobar".to_string(),
    };

    data_writer.write(data, None).expect("Unable to write data");

    let res = data_writer
      .wait_for_acknowledgments(Duration::from_secs(2))
      .unwrap();
    assert!(res); // we should get "true" immediately, because we have
                  // no Reliable QoS
  }
}
