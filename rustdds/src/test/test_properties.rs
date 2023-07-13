use crate::{
  messages::submessages::{
    elements::serialized_payload::SerializedPayload,
    submessages::{Data, RepresentationIdentifier},
  },
  structure::{guid::EntityId, sequence_number::SequenceNumber},
};
/*
use mio_extras::channel as mio_channel;
use crate::dds::interfaces::{IDataReader};
use crate::dds::traits::key::Keyed;
use crate::dds::traits::key::Key;
use crate::dds::traits::serde_adapters::DeserializerAdapter;
use serde::de::DeserializeOwned;
*/

// Different properties that do not belong to the actual library but make
// testing easier.

impl Default for SerializedPayload {
  fn default() -> Self {
    Self::new(RepresentationIdentifier::CDR_LE, b"fake data".to_vec())
  }
}

impl Default for Data {
  fn default() -> Self {
    Self {
      reader_id: EntityId::default(),
      writer_id: EntityId::default(),
      writer_sn: SequenceNumber::default(),
      inline_qos: None,
      serialized_payload: Some(SerializedPayload::default()),
    }
  }
}

/*
trait TestingTrait {
  fn TEST_FUNCTION_set_status_change_receiver(&mut self, receiver : mio_channel::Receiver<StatusChange>);
  fn TEST_FUNCTION_get_requested_deadline_missed_status(&mut self)-> Result<Option<RequestedDeadlineMissedStatus>>;
  fn TEST_FUNCTION_set_reader_commander(&mut self, sender : mio_channel::SyncSender<ReaderCommand> );
}


impl<'a, D: 'static, SA> DataReader<'a, D, SA>
where
  D: DeserializeOwned + Keyed,
  <D as Keyed>::K: Key,
  SA: DeserializerAdapter<D>,
  {

  fn TEST_FUNCTION_set_status_change_receiver(&mut self, receiver : mio_channel::Receiver<StatusChange>){
    self.status_receiver = receiver;
  }

  fn TEST_FUNCTION_get_requested_deadline_missed_status(&mut self)-> Result<Option<RequestedDeadlineMissedStatus>>{
    self.get_requested_deadline_missed_status()
  }

  fn TEST_FUNCTION_set_reader_commander(&mut self, sender : mio_channel::SyncSender<ReaderCommand> ){
    self.reader_command = sender;
  }

}
*/
