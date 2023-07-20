use std::{io, marker::PhantomData, sync::atomic};

use mio::{Evented, Poll, PollOpt, Ready, Token};
#[allow(unused_imports)]
use log::{debug, error, info, warn};
use futures::{pin_mut, stream::FusedStream, Stream, StreamExt};
use rustdds::{rpc::*, *};

use crate::{message::Message, node::Node, pubsub::MessageInfo, DdsError};

pub mod request_id;
mod wrappers;

pub use request_id::*;
use wrappers::*;

// --------------------------------------------
// --------------------------------------------

/// Service trait pairs the Request and Response types together.
/// Additonally, it ensures that Response and Request are Messages
/// (Serializable), and we have a means to name the types.
pub trait Service {
  type Request: Message;
  type Response: Message;
  fn request_type_name(&self) -> &str;
  fn response_type_name(&self) -> &str;
}

// --------------------------------------------
// --------------------------------------------

/// AService is a means of constructing a descriptor for a Service on the fly.
/// This allows generic code to construct a Service from the types of
/// request and response.
pub struct AService<Q, S>
where
  Q: Message,
  S: Message,
{
  q: PhantomData<Q>,
  s: PhantomData<S>,
  req_type_name: String,
  resp_type_name: String,
}

impl<Q, S> AService<Q, S>
where
  Q: Message,
  S: Message,
{
  pub fn new(req_type_name: String, resp_type_name: String) -> Self {
    Self {
      req_type_name,
      resp_type_name,
      q: PhantomData,
      s: PhantomData,
    }
  }
}

impl<Q, S> Service for AService<Q, S>
where
  Q: Message,
  S: Message,
{
  type Request = Q;
  type Response = S;

  fn request_type_name(&self) -> &str {
    &self.req_type_name
  }

  fn response_type_name(&self) -> &str {
    &self.resp_type_name
  }
}

// --------------------------------------------
// --------------------------------------------

/// There are different and incompatible ways to map Services onto DDS Topics.
/// The mapping used by ROS2 depends on the DDS implementation used and its
/// configuration. For details, see OMG Specification
/// [RPC over DDS](https://www.omg.org/spec/DDS-RPC/1.0/About-DDS-RPC/) Section "7.2.4 Basic and Enhanced Service Mapping for RPC over DDS"
/// RPC over DDS" . which defines Service Mappings "Basic" and "Enhanced"
/// ServiceMapping::Cyclone reporesents a third mapping used by RMW for
/// CycloneDDS.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ServiceMapping {
  /// "Basic" service mapping from RPC over DDS specification.
  /// * RTI Connext with `RMW_CONNEXT_REQUEST_REPLY_MAPPING=basic`, but this is
  ///   not tested, so may not work.
  Basic,

  /// "Enhanced" service mapping from RPC over DDS specification.
  /// * ROS2 Foxy with eProsima DDS,
  /// * ROS2 Galactic with RTI Connext (rmw_connextdds, not rmw_connext_cpp) -
  ///   set environment variable `RMW_CONNEXT_REQUEST_REPLY_MAPPING=extended`
  ///   before running ROS2 executable.
  Enhanced,

  /// CycloneDDS-specific service mapping.
  /// Specification for this mapping is unknown, technical details are
  /// reverse-engineered from ROS2 sources.
  /// * ROS2 Galactic with CycloneDDS - Seems to work on the same host only, not
  ///   over actual network.
  Cyclone,
}

// --------------------------------------------
// --------------------------------------------
/// Server end of a ROS2 Service
pub struct Server<S>
where
  S: Service,
  S::Request: Message,
  S::Response: Message,
{
  service_mapping: ServiceMapping,
  request_receiver: SimpleDataReaderR,
  response_sender: DataWriterR<ResponseWrapper<S::Response>>,
}

impl<S> Server<S>
where
  S: 'static + Service,
{
  pub(crate) fn new(
    service_mapping: ServiceMapping,
    node: &mut Node,
    request_topic: &Topic,
    response_topic: &Topic,
    qos_request: Option<QosPolicies>,
    qos_response: Option<QosPolicies>,
  ) -> dds::CreateResult<Self> {
    let request_receiver =
      node.create_simpledatareader::<RequestWrapper<S::Request>>(request_topic, qos_request)?;
    let response_sender =
      node.create_datawriter
      ::<ResponseWrapper<S::Response>, ServiceSerializerAdapter<ResponseWrapper<S::Response>>>(
        response_topic, qos_response)?;

    debug!(
      "Created new Server: requests={} response={}",
      request_topic.name(),
      response_topic.name()
    );

    Ok(Server::<S> {
      service_mapping,
      request_receiver,
      response_sender,
    })
  }

  /// Receive a request from Client.
  /// Returns `Ok(None)` if no new requests have arrived.
  pub fn receive_request<'de>(&self) -> dds::ReadResult<Option<(RmwRequestId, S::Request)>>
  where
    S::Request: Deserialize<'de>,
  {
    self.request_receiver.drain_read_notifications();
    let dcc_rw: Option<no_key::DeserializedCacheChange<RequestWrapper<S::Request>>> =
      self.request_receiver.try_take_one()?;

    match dcc_rw {
      None => Ok(None),
      Some(dcc) => {
        let mi = MessageInfo::from(&dcc);
        let req_wrapper = dcc.into_value();
        let (ri, req) = req_wrapper.unwrap(self.service_mapping, &mi)?;
        Ok(Some((ri, req)))
      }
    } // match
  }

  /// Send response to request by Client.
  /// rmw_req_id identifies request being responded.
  pub fn send_response(
    &self,
    rmw_req_id: RmwRequestId,
    response: &S::Response,
  ) -> Result<(), DdsError<()>> {
    let resp_wrapper: ResponseWrapper<<S as Service>::Response> =
      ResponseWrapper::<S::Response>::new(
        self.service_mapping,
        rmw_req_id,
        RepresentationIdentifier::CDR_LE,
        response,
      )?;
    let write_opts = WriteOptionsBuilder::new()
      .source_timestamp(Timestamp::now()) // always add source timestamp
      .related_sample_identity(SampleIdentity::from(rmw_req_id))
      // TODO: Check if this is right. Cyclone mapping does not send
      // Related Sample Identity in
      // WriteOptions (QoS ParameterList), but within data payload.
      // But maybe it is not harmful to send it in both?
      .build();
    self
      .response_sender
      .write_with_options(resp_wrapper, write_opts)
      .map(|_| ()) // lose SampleIdentity result
      .map_err(|err| err.forget_data())?;
    Ok(())
  }

  /// The request_id must be sent back with the response to identify which
  /// request and response belong together.
  pub async fn async_receive_request(&self) -> dds::ReadResult<(RmwRequestId, S::Request)> {
    let dcc_stream = self.request_receiver.as_async_stream();
    pin_mut!(dcc_stream);

    match dcc_stream.next().await {
      None => Err(dds::ReadError::Internal {
        reason: "SimpleDataReader value stream unexpectedly ended!".to_string(),
      }),
      // This should never occur, because topic do not "end".
      Some(Err(e)) => Err(e),
      Some(Ok(dcc)) => {
        let mi = MessageInfo::from(&dcc);
        let req_wrapper = dcc.into_value();
        let (ri, req) = req_wrapper.unwrap(self.service_mapping, &mi)?;
        Ok((ri, req))
      }
    } // match
  }

  /// Returns a never-ending stream of (request_id, request)
  /// The request_id must be sent back with the response to identify which
  /// request and response belong together.
  pub fn receive_request_stream(
    &self,
  ) -> impl Stream<Item = dds::ReadResult<(RmwRequestId, S::Request)>> + FusedStream + '_ {
    Box::pin(self.request_receiver.as_async_stream()).then(
      move |dcc_r| async move {
        match dcc_r {
          Err(e) => Err(e),
          Ok(dcc) => {
            let mi = MessageInfo::from(&dcc);
            let req_wrapper = dcc.into_value();
            req_wrapper.unwrap(self.service_mapping, &mi)
          }
        } // match
      }, // async
    )
  }

  /// Asynchronous response sending
  pub async fn async_send_response(
    &self,
    rmw_req_id: RmwRequestId,
    response: &S::Response,
  ) -> Result<(), DdsError<()>> {
    let resp_wrapper = ResponseWrapper::<S::Response>::new(
      self.service_mapping,
      rmw_req_id,
      RepresentationIdentifier::CDR_LE,
      response,
    )?;
    let write_opts = WriteOptionsBuilder::new()
      .source_timestamp(Timestamp::now()) // always add source timestamp
      .related_sample_identity(SampleIdentity::from(rmw_req_id))
      // TODO: Check if this is right. Cyclone mapping does not send
      // Related Sample Identity in
      // WriteOptions (QoS ParameterList), but within data payload.
      // But maybe it is not harmful to send it in both?
      .build();
    let _ = self
      .response_sender
      .async_write_with_options(resp_wrapper, write_opts)
      .await; // lose SampleIdentity result
    Ok(())
  }
}

impl<S> Evented for Server<S>
where
  S: 'static + Service,
{
  fn register(&self, poll: &Poll, token: Token, interest: Ready, opts: PollOpt) -> io::Result<()> {
    self.request_receiver.register(poll, token, interest, opts)
  }

  fn reregister(
    &self,
    poll: &Poll,
    token: Token,
    interest: Ready,
    opts: PollOpt,
  ) -> io::Result<()> {
    self
      .request_receiver
      .reregister(poll, token, interest, opts)
  }

  fn deregister(&self, poll: &Poll) -> io::Result<()> {
    self.request_receiver.deregister(poll)
  }
}

/// Client end of a ROS2 Service
pub struct Client<S>
where
  S: Service,
  S::Request: Message,
  S::Response: Message,
{
  service_mapping: ServiceMapping,
  request_sender: DataWriterR<RequestWrapper<S::Request>>,
  response_receiver: SimpleDataReaderR,
  sequence_number_gen: atomic::AtomicI64, // used by basic and cyclone
  client_guid: GUID,                      // used by the Cyclone ServiceMapping
}

impl<S> Client<S>
where
  S: 'static + Service,
{
  pub(crate) fn new(
    service_mapping: ServiceMapping,
    node: &mut Node,
    request_topic: &Topic,
    response_topic: &Topic,
    qos_request: Option<QosPolicies>,
    qos_response: Option<QosPolicies>,
  ) -> dds::CreateResult<Self> {
    let request_sender = node.create_datawriter(request_topic, qos_request)?;
    let response_receiver = node.create_simpledatareader(response_topic, qos_response)?;

    debug!(
      "Created new Client: request={} response={}",
      request_topic.name(),
      response_topic.name()
    );
    let client_guid = request_sender.guid();
    Ok(Client::<S> {
      service_mapping,
      request_sender,
      response_receiver,
      sequence_number_gen: atomic::AtomicI64::new(SequenceNumber::default().into()),
      client_guid,
    })
  }

  /// Send a request to Service Server.
  /// The returned `RmwRequestId` is a token to identify the correct response.
  pub fn send_request(&self, request: &S::Request) -> Result<RmwRequestId, DdsError<()>> {
    self.increment_sequence_number();
    let gen_rmw_req_id = RmwRequestId {
      writer_guid: self.client_guid,
      sequence_number: self.sequence_number(),
    };
    let req_wrapper = RequestWrapper::<S::Request>::new(
      self.service_mapping,
      gen_rmw_req_id,
      RepresentationIdentifier::CDR_LE,
      request,
    )?;
    let write_opts_builder = WriteOptionsBuilder::new().source_timestamp(Timestamp::now()); // always add source timestamp

    let write_opts_builder = if self.service_mapping == ServiceMapping::Enhanced {
      write_opts_builder
    } else {
      write_opts_builder.related_sample_identity(SampleIdentity::from(gen_rmw_req_id))
    };
    let sent_rmw_req_id = self
      .request_sender
      .write_with_options(req_wrapper, write_opts_builder.build())
      .map(RmwRequestId::from)
      .map_err(|err| DdsError::from(err).map_write_error_data(|_| ()))?;

    match self.service_mapping {
      ServiceMapping::Enhanced => Ok(sent_rmw_req_id),
      ServiceMapping::Basic | ServiceMapping::Cyclone => Ok(gen_rmw_req_id),
    }
  }

  /// Receive a response from Server
  /// Returns `Ok(None)` if no new responses have arrived.
  /// Note: The response may to someone else's request. Check received
  /// `RmWRequestId` against the one you got when sending request to identify
  /// the correct response. In case you receive someone else's response,
  /// please do receive again.
  pub fn receive_response(&self) -> dds::ReadResult<Option<(RmwRequestId, S::Response)>> {
    self.response_receiver.drain_read_notifications();
    let dcc_rw: Option<no_key::DeserializedCacheChange<ResponseWrapper<S::Response>>> =
      self.response_receiver.try_take_one()?;

    match dcc_rw {
      None => Ok(None),
      Some(dcc) => {
        let mi = MessageInfo::from(&dcc);
        let res_wrapper = dcc.into_value();
        let (ri, res) = res_wrapper.unwrap(self.service_mapping, mi, self.client_guid)?;
        Ok(Some((ri, res)))
      }
    } // match
  }

  /// Send a request to Service Server asynchronously.
  /// The returned `RmwRequestId` is a token to identify the correct response.
  pub async fn async_send_request(
    &self,
    request: &S::Request,
  ) -> Result<RmwRequestId, DdsError<()>> {
    let gen_rmw_req_id =
      // we do the req_id generation in an async block so that we do not generate
      // multiple sequence numbers if there are multiple polls to this function
      async {
        self.increment_sequence_number();
         RmwRequestId {
          writer_guid: self.client_guid,
          sequence_number: self.sequence_number(),
        }
      }.await;

    let req_wrapper = RequestWrapper::<S::Request>::new(
      self.service_mapping,
      gen_rmw_req_id,
      RepresentationIdentifier::CDR_LE,
      request,
    )?;
    let write_opts_builder = WriteOptionsBuilder::new().source_timestamp(Timestamp::now()); // always add source timestamp

    let write_opts_builder = if self.service_mapping == ServiceMapping::Enhanced {
      write_opts_builder
    } else {
      write_opts_builder.related_sample_identity(SampleIdentity::from(gen_rmw_req_id))
    };
    let sent_rmw_req_id = self
      .request_sender
      .async_write_with_options(req_wrapper, write_opts_builder.build())
      .await
      .map(RmwRequestId::from)
      .map_err(|err| DdsError::from(err).map_write_error_data(|_| ()))?;

    let req_id = match self.service_mapping {
      ServiceMapping::Enhanced => sent_rmw_req_id,
      ServiceMapping::Basic | ServiceMapping::Cyclone => gen_rmw_req_id,
    };
    debug!(
      "Sent Request {:?} to {:?}",
      req_id,
      self.request_sender.topic().name()
    );
    Ok(req_id)
  }

  /// Receive a response from Server
  /// The returned Future does not complete until the response has been
  /// received.
  pub async fn async_receive_response(
    &self,
    request_id: RmwRequestId,
  ) -> dds::ReadResult<S::Response> {
    let dcc_stream = self.response_receiver.as_async_stream();
    pin_mut!(dcc_stream);

    loop {
      match dcc_stream.next().await {
        None => {
          return Err(dds::ReadError::Internal {
            reason: "SimpleDataReader value stream unexpectedly ended!".to_string(),
          })
        }
        // This should never occur, because topic do not "end".
        Some(Err(e)) => return Err(e),
        Some(Ok(dcc)) => {
          let mi = MessageInfo::from(&dcc);
          let (req_id, response) =
            dcc
              .into_value()
              .unwrap(self.service_mapping, mi, self.client_guid)?;
          if req_id == request_id {
            return Ok(response);
          } else {
            debug!(
              "Received response for someone else. expected={:?}  received={:?}",
              request_id, req_id
            );
            continue; //
          }
        }
      }
    } // loop
  }

  pub async fn async_call_service(
    &self,
    request: &S::Request,
  ) -> Result<S::Response, DdsError<()>> {
    let req_id = self.async_send_request(request).await?;
    let response = self.async_receive_response(req_id).await?;
    Ok(response)
  }

  fn increment_sequence_number(&self) {
    self
      .sequence_number_gen
      .fetch_add(1, atomic::Ordering::Acquire);
  }

  fn sequence_number(&self) -> request_id::SequenceNumber {
    SequenceNumber::from(self.sequence_number_gen.load(atomic::Ordering::Acquire)).into()
  }
}

impl<S> Evented for Client<S>
where
  S: 'static + Service,
{
  fn register(&self, poll: &Poll, token: Token, interest: Ready, opts: PollOpt) -> io::Result<()> {
    self.response_receiver.register(poll, token, interest, opts)
  }

  fn reregister(
    &self,
    poll: &Poll,
    token: Token,
    interest: Ready,
    opts: PollOpt,
  ) -> io::Result<()> {
    self
      .response_receiver
      .reregister(poll, token, interest, opts)
  }

  fn deregister(&self, poll: &Poll) -> io::Result<()> {
    self.response_receiver.deregister(poll)
  }
}
