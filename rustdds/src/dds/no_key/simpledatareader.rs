use std::{io, task::Waker};

use futures::stream::{FusedStream, Stream, StreamExt};
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use mio_06::{self, Evented};
use mio_08;

use crate::{
  dds::{
    adapters::no_key::*, no_key::datasample::DeserializedCacheChange, qos::*, result::ReadResult,
    statusevents::*, with_key,
  },
  serialization::CDRDeserializerAdapter,
  structure::entity::RTPSEntity,
  GUID,
};
use super::wrappers::{DAWrapper, NoKeyWrapper};

/// SimpleDataReaders can only do "take" semantics and does not have
/// any deduplication or other DataSampleCache functionality.
pub struct SimpleDataReader<D, DA: DeserializerAdapter<D> = CDRDeserializerAdapter<D>> {
  keyed_simpledatareader: with_key::SimpleDataReader<NoKeyWrapper<D>, DAWrapper<DA>>,
}

/// Simplified type for CDR encoding
pub type SimpleDataReaderCdr<D> = SimpleDataReader<D, CDRDeserializerAdapter<D>>;

impl<D: 'static, DA> SimpleDataReader<D, DA>
where
  DA: DeserializerAdapter<D> + 'static,
{
  // TODO: Make it possible to construct SimpleDataReader (particularly, no_key
  // version) from the public API. That is, From a Subscriber object like a
  // normal Datareader. This is to be then used from the ros2-client package.
  pub(crate) fn from_keyed(
    keyed_simpledatareader: with_key::SimpleDataReader<NoKeyWrapper<D>, DAWrapper<DA>>,
  ) -> Self {
    Self {
      keyed_simpledatareader,
    }
  }

  pub fn set_waker(&self, w: Option<Waker>) {
    self.keyed_simpledatareader.set_waker(w);
  }

  pub fn drain_read_notifications(&self) {
    self.keyed_simpledatareader.drain_read_notifications();
  }

  pub fn try_take_one(&self) -> ReadResult<Option<DeserializedCacheChange<D>>> {
    match self.keyed_simpledatareader.try_take_one() {
      Err(e) => Err(e),
      Ok(None) => Ok(None),
      Ok(Some(kdcc)) => match DeserializedCacheChange::<D>::from_keyed(kdcc) {
        Some(dcc) => Ok(Some(dcc)),
        None => Ok(None),
      },
    }
  }

  pub fn qos(&self) -> &QosPolicies {
    self.keyed_simpledatareader.qos()
  }

  pub fn guid(&self) -> GUID {
    self.keyed_simpledatareader.guid()
  }

  pub fn as_async_stream(
    &self,
  ) -> impl Stream<Item = ReadResult<DeserializedCacheChange<D>>> + FusedStream + '_ {
    self
      .keyed_simpledatareader
      .as_async_stream()
      .filter_map(move |r| async {
        // This is Stream::filter_map, so returning None means just skipping Item.
        match r {
          Err(e) => Some(Err(e)),
          Ok(kdcc) => match DeserializedCacheChange::<D>::from_keyed(kdcc) {
            None => {
              info!("Got dispose from no_key topic.");
              None
            }
            Some(dcc) => Some(Ok(dcc)),
          },
        }
      })
  }

  pub fn as_simple_data_reader_event_stream(
    &self,
  ) -> impl Stream<Item = ReadResult<DataReaderStatus>> + '_ {
    self
      .keyed_simpledatareader
      .as_simple_data_reader_event_stream()
  }
}

// This is  not part of DDS spec. We implement mio Eventd so that the
// application can asynchronously poll DataReader(s).
impl<D, DA> Evented for SimpleDataReader<D, DA>
where
  DA: DeserializerAdapter<D>,
{
  // We just delegate all the operations to notification_receiver, since it
  // already implements Evented
  fn register(
    &self,
    poll: &mio_06::Poll,
    token: mio_06::Token,
    interest: mio_06::Ready,
    opts: mio_06::PollOpt,
  ) -> io::Result<()> {
    self
      .keyed_simpledatareader
      .register(poll, token, interest, opts)
  }

  fn reregister(
    &self,
    poll: &mio_06::Poll,
    token: mio_06::Token,
    interest: mio_06::Ready,
    opts: mio_06::PollOpt,
  ) -> io::Result<()> {
    self
      .keyed_simpledatareader
      .reregister(poll, token, interest, opts)
  }

  fn deregister(&self, poll: &mio_06::Poll) -> io::Result<()> {
    self.keyed_simpledatareader.deregister(poll)
  }
}

impl<D, DA> mio_08::event::Source for SimpleDataReader<D, DA>
where
  DA: DeserializerAdapter<D>,
{
  fn register(
    &mut self,
    registry: &mio_08::Registry,
    token: mio_08::Token,
    interests: mio_08::Interest,
  ) -> io::Result<()> {
    mio_08::event::Source::register(&mut self.keyed_simpledatareader, registry, token, interests)
    // self.keyed_simpledatareader.register(registry, token, interests)
  }

  fn reregister(
    &mut self,
    registry: &mio_08::Registry,
    token: mio_08::Token,
    interests: mio_08::Interest,
  ) -> io::Result<()> {
    mio_08::event::Source::reregister(&mut self.keyed_simpledatareader, registry, token, interests)
    // self.keyed_simpledatareader.reregister(registry, token, interests)
  }

  fn deregister(&mut self, registry: &mio_08::Registry) -> io::Result<()> {
    mio_08::event::Source::deregister(&mut self.keyed_simpledatareader, registry)
    // self.keyed_simpledatareader.deregister(registry)
  }
}

impl<D, DA> StatusEvented<DataReaderStatus> for SimpleDataReader<D, DA>
where
  DA: DeserializerAdapter<D>,
{
  fn as_status_evented(&mut self) -> &dyn Evented {
    self.keyed_simpledatareader.as_status_evented()
  }

  fn as_status_source(&mut self) -> &mut dyn mio_08::event::Source {
    self.keyed_simpledatareader.as_status_source()
  }

  fn try_recv_status(&self) -> Option<DataReaderStatus> {
    self.keyed_simpledatareader.try_recv_status()
  }
}

impl<D, DA> RTPSEntity for SimpleDataReader<D, DA>
where
  D: 'static,
  DA: DeserializerAdapter<D>,
{
  fn guid(&self) -> GUID {
    self.keyed_simpledatareader.guid()
  }
}

// ----------------------------------------------
// ----------------------------------------------
