use std::{
    collections::HashMap,
    sync::{
        Arc, Mutex,
        atomic::{AtomicI64, AtomicUsize, Ordering},
    },
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use futures::{FutureExt, select};
use thiserror::Error;
use zenoh::{
    Wait,
    bytes::ZBytes,
    query::{ConsolidationMode, Querier, Query, QueryTarget, Queryable},
};

use super::{
    DeclaredEndpoint, Node,
    attachment::Attachment,
    graph::{GraphCache, GraphError},
    keyexpr::{EntityKind, TopicToken},
};
use crate::transport::RequestId;

#[derive(Debug, Error)]
pub enum ServiceError {
    #[error("malformed rmw_zenoh service attachment")]
    MalformedAttachment,
    #[error("service pending-request limit {limit} reached")]
    PendingLimit { limit: usize },
    #[error("service request is absent")]
    RequestAbsent,
    #[error("service request expired")]
    RequestExpired,
    #[error("service request timed out")]
    Timeout,
    #[error("ROS transport is closed")]
    TransportClosed,
    #[error("remote service error: {0}")]
    Remote(String),
    #[error("Zenoh service error: {0}")]
    Zenoh(String),
}

pub fn request_id_from_attachment(bytes: Option<&[u8]>) -> Result<RequestId, ServiceError> {
    let attachment = Attachment::decode(bytes.ok_or(ServiceError::MalformedAttachment)?)
        .map_err(|_| ServiceError::MalformedAttachment)?;
    Ok(RequestId {
        sequence_number: attachment.sequence_number,
        client_gid: attachment.gid,
    })
}

pub struct PendingRequests<T> {
    values: HashMap<RequestId, (T, Duration)>,
    limit: usize,
    expiry: Duration,
}

impl<T> PendingRequests<T> {
    pub fn new(limit: usize, expiry: Duration) -> Self {
        Self {
            values: HashMap::new(),
            limit,
            expiry,
        }
    }
    pub fn insert(&mut self, id: RequestId, value: T, now: Duration) -> Result<(), ServiceError> {
        self.expire(now);
        if self.values.len() >= self.limit && !self.values.contains_key(&id) {
            return Err(ServiceError::PendingLimit { limit: self.limit });
        }
        self.values.insert(id, (value, now));
        Ok(())
    }
    pub fn take(&mut self, id: RequestId, now: Duration) -> Result<T, ServiceError> {
        let expired = self
            .values
            .get(&id)
            .is_some_and(|(_, inserted)| now.saturating_sub(*inserted) >= self.expiry);
        if expired {
            self.expire(now);
            return Err(ServiceError::RequestExpired);
        }
        self.values
            .remove(&id)
            .map(|(value, _)| value)
            .ok_or(ServiceError::RequestAbsent)
    }
    pub fn expire(&mut self, now: Duration) {
        self.values
            .retain(|_, (_, inserted)| now.saturating_sub(*inserted) < self.expiry);
    }
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }
}

fn elapsed(origin: Instant) -> Duration {
    origin.elapsed()
}
fn timestamp_ns() -> Result<i64, ServiceError> {
    i64::try_from(
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_err(|_| ServiceError::Zenoh("clock is before Unix epoch".into()))?
            .as_nanos(),
    )
    .map_err(|_| ServiceError::Zenoh("timestamp overflow".into()))
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ServiceRequest {
    pub id: RequestId,
    pub payload: Vec<u8>,
}

pub struct RawServiceServer {
    _queryable: Queryable<()>,
    requests: flume::Receiver<ServiceRequest>,
    pending: Arc<Mutex<PendingRequests<Query>>>,
    key: String,
    origin: Instant,
}

impl RawServiceServer {
    pub async fn declare(
        session: &zenoh::Session,
        key: &str,
        limit: usize,
        expiry: Duration,
    ) -> Result<Self, ServiceError> {
        let (sender, requests) = flume::bounded(limit);
        let pending = Arc::new(Mutex::new(PendingRequests::new(limit, expiry)));
        let callback_pending = pending.clone();
        let origin = Instant::now();
        let queryable = session
            .declare_queryable(key.to_owned())
            .complete(true)
            .callback(move |query| {
                let attachment = query.attachment().map(|value| value.to_bytes());
                let id = request_id_from_attachment(attachment.as_deref());
                let payload = query.payload().map(|value| value.to_bytes().into_owned());
                match (id, payload) {
                    (Ok(id), Some(payload)) => {
                        let request = ServiceRequest { id, payload };
                        let inserted = callback_pending
                            .lock()
                            .unwrap_or_else(|error| error.into_inner())
                            .insert(id, query, elapsed(origin));
                        if inserted.is_ok() && sender.try_send(request).is_err() {
                            let _ = callback_pending
                                .lock()
                                .unwrap_or_else(|error| error.into_inner())
                                .take(id, elapsed(origin));
                        }
                    }
                    _ => {
                        let _ = query.reply_err("malformed service request").wait();
                    }
                }
            })
            .await
            .map_err(|error| ServiceError::Zenoh(error.to_string()))?;
        Ok(Self {
            _queryable: queryable,
            requests,
            pending,
            key: key.into(),
            origin,
        })
    }
    pub async fn recv(&self) -> Result<ServiceRequest, ServiceError> {
        self.requests
            .recv_async()
            .await
            .map_err(|_| ServiceError::TransportClosed)
    }
    pub async fn reply(&self, id: RequestId, payload: &[u8]) -> Result<(), ServiceError> {
        let query = self
            .pending
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .take(id, elapsed(self.origin))?;
        let attachment = Attachment {
            sequence_number: id.sequence_number,
            source_timestamp_ns: timestamp_ns()?,
            gid: id.client_gid,
        }
        .encode()
        .map_err(|_| ServiceError::MalformedAttachment)?;
        query
            .reply(self.key.clone(), ZBytes::from(payload.to_vec()))
            .attachment(ZBytes::from(attachment))
            .await
            .map_err(|error| ServiceError::Zenoh(error.to_string()))
    }
    pub async fn reject(&self, id: RequestId, message: &str) -> Result<(), ServiceError> {
        let query = self
            .pending
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .take(id, elapsed(self.origin))?;
        query
            .reply_err(ZBytes::from(message.to_owned()))
            .await
            .map_err(|error| ServiceError::Zenoh(error.to_string()))
    }
}

pub struct RawServiceClient {
    querier: Querier<'static>,
    gid: [u8; 16],
    sequence: AtomicI64,
    outstanding: AtomicUsize,
    limit: usize,
}

impl RawServiceClient {
    pub async fn declare(
        session: &zenoh::Session,
        key: &str,
        gid: [u8; 16],
        limit: usize,
    ) -> Result<Self, ServiceError> {
        let querier = session
            .declare_querier(key.to_owned())
            .target(QueryTarget::AllComplete)
            .consolidation(ConsolidationMode::None)
            .await
            .map_err(|error| ServiceError::Zenoh(error.to_string()))?;
        Ok(Self {
            querier,
            gid,
            sequence: AtomicI64::new(1),
            outstanding: AtomicUsize::new(0),
            limit,
        })
    }
    pub async fn call(&self, payload: Vec<u8>, timeout: Duration) -> Result<Vec<u8>, ServiceError> {
        let previous = self.outstanding.fetch_add(1, Ordering::AcqRel);
        if previous >= self.limit {
            self.outstanding.fetch_sub(1, Ordering::AcqRel);
            return Err(ServiceError::PendingLimit { limit: self.limit });
        }
        struct Guard<'a>(&'a AtomicUsize);
        impl Drop for Guard<'_> {
            fn drop(&mut self) {
                self.0.fetch_sub(1, Ordering::AcqRel);
            }
        }
        let _guard = Guard(&self.outstanding);
        let sequence_number = self
            .sequence
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
                value.checked_add(1)
            })
            .map_err(|_| ServiceError::Zenoh("service sequence exhausted".into()))?;
        let expected = RequestId {
            sequence_number,
            client_gid: self.gid,
        };
        let attachment = Attachment {
            sequence_number,
            source_timestamp_ns: timestamp_ns()?,
            gid: self.gid,
        }
        .encode()
        .map_err(|_| ServiceError::MalformedAttachment)?;
        let replies = self
            .querier
            .get()
            .payload(ZBytes::from(payload))
            .attachment(ZBytes::from(attachment))
            .await
            .map_err(|error| ServiceError::Zenoh(error.to_string()))?;
        let mut deadline = futures_timer::Delay::new(timeout).fuse();
        loop {
            let mut receive = replies.recv_async().fuse();
            select! {
                reply = receive => match reply {
                    Ok(reply) => match reply.into_result() {
                        Ok(sample) => {
                            let actual = request_id_from_attachment(sample.attachment().map(|value| value.to_bytes()).as_deref())?;
                            if actual == expected { return Ok(sample.payload().to_bytes().into_owned()); }
                        }
                        Err(error) => return Err(ServiceError::Remote(String::from_utf8_lossy(&error.payload().to_bytes()).into_owned())),
                    },
                    Err(_) => return Err(ServiceError::Timeout),
                },
                _ = deadline => return Err(ServiceError::Timeout),
            }
        }
    }
}

pub struct NodeServiceClient(DeclaredEndpoint<RawServiceClient>);
impl NodeServiceClient {
    pub async fn declare(
        node: &Node,
        key: &str,
        topic: TopicToken,
        limit: usize,
    ) -> Result<Self, ServiceError> {
        let id = node.allocate_entity();
        let client = RawServiceClient::declare(node.session(), key, id.gid, limit).await?;
        let endpoint = node
            .declare_endpoint_with_id(client, id, EntityKind::Client, topic)
            .await
            .map_err(|error| ServiceError::Zenoh(error.to_string()))?;
        Ok(Self(endpoint))
    }
    pub async fn call(&self, payload: Vec<u8>, timeout: Duration) -> Result<Vec<u8>, ServiceError> {
        self.0.data_entity.call(payload, timeout).await
    }
}

pub struct NodeServiceServer(DeclaredEndpoint<RawServiceServer>);
impl NodeServiceServer {
    pub async fn declare(
        node: &Node,
        key: &str,
        topic: TopicToken,
        limit: usize,
        expiry: Duration,
    ) -> Result<Self, ServiceError> {
        let server = RawServiceServer::declare(node.session(), key, limit, expiry).await?;
        let endpoint = node
            .declare_endpoint(server, EntityKind::Service, topic)
            .await
            .map_err(|error| ServiceError::Zenoh(error.to_string()))?;
        Ok(Self(endpoint))
    }
    pub async fn recv(&self) -> Result<ServiceRequest, ServiceError> {
        self.0.data_entity.recv().await
    }
    pub async fn reply(&self, id: RequestId, payload: &[u8]) -> Result<(), ServiceError> {
        self.0.data_entity.reply(id, payload).await
    }
    pub async fn reject(&self, id: RequestId, message: &str) -> Result<(), ServiceError> {
        self.0.data_entity.reject(id, message).await
    }
}

#[allow(clippy::too_many_arguments)]
pub async fn wait_for_service(
    graph: &GraphCache,
    name: &str,
    type_name: &str,
    type_hash: &str,
    qos: &str,
    deadline: Instant,
) -> Result<(), ServiceError> {
    loop {
        let snapshot = graph.snapshot();
        if snapshot.closed {
            return Err(ServiceError::TransportClosed);
        }
        if !graph
            .matching_services(name, type_name, type_hash, qos)
            .is_empty()
        {
            return Ok(());
        }
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .ok_or(ServiceError::Timeout)?;
        let changed = graph.wait_for_change(snapshot.generation).fuse();
        let timer = futures_timer::Delay::new(remaining).fuse();
        futures::pin_mut!(changed, timer);
        select! {
            result = changed => match result { Ok(_) => {}, Err(GraphError::Closed) => return Err(ServiceError::TransportClosed), Err(error) => return Err(ServiceError::Zenoh(error.to_string())) },
            _ = timer => return Err(ServiceError::Timeout),
        }
    }
}
