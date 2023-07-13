use crate::{
  dds::{ddsdata::DDSData, with_key::datawriter::WriteOptions},
  structure::{guid::GUID, sequence_number::SequenceNumber},
};

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Copy, Clone)]
pub enum ChangeKind {
  Alive,
  NotAliveDisposed,
  NotAliveUnregistered,
}

#[derive(Debug, Clone)]
pub struct CacheChange {
  pub writer_guid: GUID,
  pub sequence_number: SequenceNumber,
  pub write_options: WriteOptions,
  pub data_value: DDSData,
}

#[cfg(test)]
impl PartialEq for CacheChange {
  fn eq(&self, other: &Self) -> bool {
    self.writer_guid == other.writer_guid
      && self.sequence_number == other.sequence_number
      && self.write_options == other.write_options
      && self.data_value == other.data_value
  }
}

impl CacheChange {
  pub fn new(
    writer_guid: GUID,
    sequence_number: SequenceNumber,
    write_options: WriteOptions,
    data_value: DDSData,
  ) -> Self {
    Self {
      writer_guid,
      sequence_number,
      write_options,
      data_value,
    }
  }

  // Not needed?
  // pub fn change_kind(&self) -> ChangeKind {
  //   self.data_value.change_kind()
  // }
}
