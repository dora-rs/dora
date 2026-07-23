use std::sync::{
    Arc,
    atomic::{AtomicI64, AtomicU64, Ordering},
};

use thiserror::Error;

use crate::transport::{
    Durability, History, MessageMetadata, Reliability, Ros2Qos, TransportError,
};

use super::{
    DeclaredEndpoint, Node,
    attachment::{Attachment, AttachmentError},
    keyexpr::{EntityKind, TopicToken},
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PublisherMode {
    Standard {
        block: bool,
        depth: Option<usize>,
        transient_local: bool,
    },
    Advanced {
        block: bool,
        depth: Option<usize>,
        transient_local: bool,
    },
}

impl PublisherMode {
    pub fn from_qos(qos: &Ros2Qos) -> Result<Self, PubSubError> {
        let block = matches!(qos.reliability, Reliability::Reliable { .. })
            || matches!(qos.history, History::KeepAll);
        let depth = match qos.history {
            History::KeepLast { depth } if depth > 0 => Some(depth as usize),
            History::KeepLast { depth } => return Err(PubSubError::InvalidDepth(depth)),
            History::KeepAll => None,
        };
        let transient_local = qos.durability == Durability::TransientLocal;
        // rmw_zenoh uses ordinary Zenoh publishers for reliable volatile data;
        // advanced pub/sub is needed only for transient-local history replay.
        let advanced = transient_local;
        Ok(if advanced {
            Self::Advanced {
                block,
                depth,
                transient_local,
            }
        } else {
            Self::Standard {
                block,
                depth,
                transient_local,
            }
        })
    }
}

#[derive(Debug)]
pub struct PublisherState {
    sequence: AtomicI64,
    gid: [u8; 16],
}
impl PublisherState {
    pub fn new(gid: [u8; 16]) -> Self {
        Self {
            sequence: AtomicI64::new(1),
            gid,
        }
    }
    pub fn metadata_at(&self, source_timestamp_ns: i64) -> Result<MessageMetadata, PubSubError> {
        let sequence_number = self
            .sequence
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
                value.checked_add(1)
            })
            .map_err(|_| PubSubError::SequenceExhausted)?;
        Ok(MessageMetadata {
            sequence_number,
            source_timestamp_ns,
            publisher_gid: self.gid,
        })
    }
    pub fn metadata_now(&self) -> Result<MessageMetadata, PubSubError> {
        let duration = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|_| PubSubError::TimestampBeforeEpoch)?;
        let nanos =
            i64::try_from(duration.as_nanos()).map_err(|_| PubSubError::TimestampOverflow)?;
        self.metadata_at(nanos)
    }
}

pub struct RawPublisher {
    publisher: NativePublisher,
    state: PublisherState,
    _mode: PublisherMode,
}

enum NativePublisher {
    Standard(zenoh::pubsub::Publisher<'static>),
    Advanced(zenoh_ext::AdvancedPublisher<'static>),
}

impl RawPublisher {
    pub async fn declare(
        session: &zenoh::Session,
        key: &str,
        gid: [u8; 16],
        qos: &Ros2Qos,
    ) -> Result<Self, PubSubError> {
        use zenoh::qos::CongestionControl;
        let mode = PublisherMode::from_qos(qos)?;
        let block = match mode {
            PublisherMode::Standard { block, .. } | PublisherMode::Advanced { block, .. } => block,
        };
        let key = zenoh::key_expr::OwnedKeyExpr::try_from(key.to_owned())
            .map_err(|error| PubSubError::Session(error.to_string()))?;
        let builder = session.declare_publisher(key).congestion_control(if block {
            CongestionControl::Block
        } else {
            CongestionControl::Drop
        });
        let publisher = match mode {
            PublisherMode::Standard { .. } => NativePublisher::Standard(
                builder
                    .await
                    .map_err(|error| PubSubError::Session(error.to_string()))?,
            ),
            PublisherMode::Advanced {
                depth,
                transient_local,
                ..
            } => {
                use zenoh_ext::AdvancedPublisherBuilderExt;
                let builder = builder.advanced();
                let builder = if transient_local {
                    builder
                        .sample_miss_detection(zenoh_ext::MissDetectionConfig::default())
                        .cache(zenoh_ext::CacheConfig::default().max_samples(depth.unwrap_or(1)))
                        .publisher_detection()
                } else {
                    builder
                };
                NativePublisher::Advanced(
                    builder
                        .await
                        .map_err(|error| PubSubError::Session(error.to_string()))?,
                )
            }
        };
        Ok(Self {
            publisher,
            state: PublisherState::new(gid),
            _mode: mode,
        })
    }
    pub fn is_advanced(&self) -> bool {
        matches!(self.publisher, NativePublisher::Advanced(_))
    }
    pub async fn publish_at(
        &self,
        payload: &[u8],
        timestamp_ns: i64,
    ) -> Result<MessageMetadata, PubSubError> {
        let metadata = self.state.metadata_at(timestamp_ns)?;
        self.send(payload, metadata.clone()).await?;
        Ok(metadata)
    }
    async fn send(&self, payload: &[u8], metadata: MessageMetadata) -> Result<(), PubSubError> {
        use zenoh::bytes::ZBytes;
        let attachment = Attachment {
            sequence_number: metadata.sequence_number,
            source_timestamp_ns: metadata.source_timestamp_ns,
            gid: metadata.publisher_gid,
        }
        .encode()?;
        match &self.publisher {
            NativePublisher::Standard(publisher) => {
                publisher
                    .put(ZBytes::from(payload.to_vec()))
                    .attachment(ZBytes::from(attachment))
                    .await
            }
            NativePublisher::Advanced(publisher) => {
                publisher
                    .put(ZBytes::from(payload.to_vec()))
                    .attachment(ZBytes::from(attachment))
                    .await
            }
        }
        .map_err(|error| PubSubError::Session(error.to_string()))?;
        Ok(())
    }
    pub async fn publish(&self, payload: &[u8]) -> Result<MessageMetadata, PubSubError> {
        let metadata = self.state.metadata_now()?;
        self.send(payload, metadata.clone()).await?;
        Ok(metadata)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SubscriptionStats {
    pub malformed_samples: u64,
    pub dropped_samples: u64,
}

pub type Decoder<T> = dyn Fn(&[u8]) -> Result<T, PubSubError> + Send + Sync;
pub struct SubscriptionIngress<T> {
    sender: flume::Sender<(T, MessageMetadata)>,
    receiver: flume::Receiver<(T, MessageMetadata)>,
    max_payload_size: usize,
    decoder: Arc<Decoder<T>>,
    malformed: AtomicU64,
    dropped: AtomicU64,
}

impl<T> SubscriptionIngress<T> {
    pub fn new(
        capacity: usize,
        max_payload_size: usize,
        decoder: Arc<Decoder<T>>,
    ) -> Result<Self, PubSubError> {
        if capacity == 0 {
            return Err(PubSubError::ZeroCapacity);
        }
        let (sender, receiver) = flume::bounded(capacity);
        Ok(Self {
            sender,
            receiver,
            max_payload_size,
            decoder,
            malformed: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
        })
    }
    pub fn ingest(&self, attachment: &[u8], payload: &[u8]) -> Result<bool, PubSubError> {
        // `max_payload_size` is a *pre-decode* guard: it stops the decoder from
        // allocating on an oversize sample. It does NOT bound the raw sample the
        // Zenoh transport already received/buffered before `ingest` runs — that
        // receive-buffer limit is Zenoh's responsibility (session/link config).
        let attachment = Attachment::decode(attachment).map_err(|error| {
            self.malformed.fetch_add(1, Ordering::Relaxed);
            PubSubError::Attachment(error)
        })?;
        if payload.len() > self.max_payload_size {
            self.malformed.fetch_add(1, Ordering::Relaxed);
            return Err(PubSubError::PayloadTooLarge {
                actual: payload.len(),
                maximum: self.max_payload_size,
            });
        }
        let value = (self.decoder)(payload).inspect_err(|_error| {
            self.malformed.fetch_add(1, Ordering::Relaxed);
        })?;
        let metadata = MessageMetadata {
            sequence_number: attachment.sequence_number,
            source_timestamp_ns: attachment.source_timestamp_ns,
            publisher_gid: attachment.gid,
        };
        match self.sender.try_send((value, metadata)) {
            Ok(()) => Ok(true),
            Err(flume::TrySendError::Full(_)) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
                Ok(false)
            }
            Err(flume::TrySendError::Disconnected(_)) => Err(PubSubError::Closed),
        }
    }
    pub fn try_recv(&self) -> Result<(T, MessageMetadata), PubSubError> {
        self.receiver.try_recv().map_err(|_| PubSubError::Empty)
    }
    pub fn stats(&self) -> SubscriptionStats {
        SubscriptionStats {
            malformed_samples: self.malformed.load(Ordering::Relaxed),
            dropped_samples: self.dropped.load(Ordering::Relaxed),
        }
    }
    pub async fn recv_async(&self) -> Result<(T, MessageMetadata), PubSubError> {
        self.receiver
            .recv_async()
            .await
            .map_err(|_| PubSubError::Closed)
    }
}

pub struct RawSubscription<T> {
    _subscriber: NativeSubscriber,
    ingress: Arc<SubscriptionIngress<T>>,
}
enum NativeSubscriber {
    Standard {
        _subscriber: zenoh::pubsub::Subscriber<()>,
    },
    Advanced {
        _subscriber: zenoh_ext::AdvancedSubscriber<()>,
    },
}

impl<T: Send + 'static> RawSubscription<T> {
    pub async fn declare(
        session: &zenoh::Session,
        key: &str,
        capacity: usize,
        max_payload_size: usize,
        decoder: Arc<Decoder<T>>,
    ) -> Result<Self, PubSubError> {
        Self::declare_inner(session, key, capacity, max_payload_size, decoder, None).await
    }
    pub async fn declare_with_qos(
        session: &zenoh::Session,
        key: &str,
        capacity: usize,
        max_payload_size: usize,
        decoder: Arc<Decoder<T>>,
        qos: &Ros2Qos,
    ) -> Result<Self, PubSubError> {
        Self::declare_inner(session, key, capacity, max_payload_size, decoder, Some(qos)).await
    }
    async fn declare_inner(
        session: &zenoh::Session,
        key: &str,
        capacity: usize,
        max_payload_size: usize,
        decoder: Arc<Decoder<T>>,
        qos: Option<&Ros2Qos>,
    ) -> Result<Self, PubSubError> {
        let ingress = Arc::new(SubscriptionIngress::new(
            capacity,
            max_payload_size,
            decoder,
        )?);
        let callback_ingress = ingress.clone();
        let key = zenoh::key_expr::OwnedKeyExpr::try_from(key.to_owned())
            .map_err(|error| PubSubError::Session(error.to_string()))?;
        let builder = session.declare_subscriber(key).callback(move |sample| {
            if std::env::var_os("DORA_ROS2_ZENOH_TRACE").is_some() {
                eprintln!(
                    "rmw_zenoh data sample: key={} payload={} attachment={}",
                    sample.key_expr(),
                    sample.payload().len(),
                    sample.attachment().map_or(0, |value| value.len())
                );
            }
            let Some(attachment) = sample.attachment() else {
                callback_ingress.malformed.fetch_add(1, Ordering::Relaxed);
                return;
            };
            let attachment = attachment.to_bytes();
            let payload = sample.payload().to_bytes();
            if let Err(error) = callback_ingress.ingest(attachment.as_ref(), payload.as_ref())
                && std::env::var_os("DORA_ROS2_ZENOH_TRACE").is_some()
            {
                eprintln!("rmw_zenoh data sample rejected: {error}");
            }
        });
        let advanced = qos.is_some_and(|value| value.durability == Durability::TransientLocal);
        let subscriber = if advanced {
            use zenoh_ext::AdvancedSubscriberBuilderExt;
            let mut history = zenoh_ext::HistoryConfig::default().detect_late_publishers();
            if let Some(History::KeepLast { depth }) = qos.map(|value| value.history) {
                history = history.max_samples(depth as usize);
            }
            NativeSubscriber::Advanced {
                _subscriber: builder
                    .advanced()
                    .history(history)
                    .await
                    .map_err(|error| PubSubError::Session(error.to_string()))?,
            }
        } else {
            NativeSubscriber::Standard {
                _subscriber: builder
                    .await
                    .map_err(|error| PubSubError::Session(error.to_string()))?,
            }
        };
        Ok(Self {
            _subscriber: subscriber,
            ingress,
        })
    }
    pub async fn recv_async(&self) -> Result<(T, MessageMetadata), PubSubError> {
        self.ingress.recv_async().await
    }
    pub fn try_recv(&self) -> Result<Option<(T, MessageMetadata)>, PubSubError> {
        match self.ingress.receiver.try_recv() {
            Ok(value) => Ok(Some(value)),
            Err(flume::TryRecvError::Empty) => Ok(None),
            Err(flume::TryRecvError::Disconnected) => Err(PubSubError::Closed),
        }
    }
    pub fn stats(&self) -> SubscriptionStats {
        self.ingress.stats()
    }
    pub fn is_advanced(&self) -> bool {
        matches!(self._subscriber, NativeSubscriber::Advanced { .. })
    }
}

pub struct NodePublisher(DeclaredEndpoint<RawPublisher>);
impl NodePublisher {
    pub async fn declare(
        node: &Node,
        key: &str,
        topic: TopicToken,
        qos: &Ros2Qos,
    ) -> Result<Self, PubSubError> {
        let id = node.allocate_entity();
        let publisher = RawPublisher::declare(node.session(), key, id.gid, qos).await?;
        let endpoint = node
            .declare_endpoint_with_id(publisher, id, EntityKind::Publisher, topic)
            .await
            .map_err(|error| PubSubError::Token(error.to_string()))?;
        Ok(Self(endpoint))
    }
    pub async fn publish(&self, payload: &[u8]) -> Result<MessageMetadata, PubSubError> {
        self.0.data_entity.publish(payload).await
    }
    pub async fn publish_at(
        &self,
        payload: &[u8],
        timestamp_ns: i64,
    ) -> Result<MessageMetadata, PubSubError> {
        self.0.data_entity.publish_at(payload, timestamp_ns).await
    }
}

pub struct NodeSubscription<T>(DeclaredEndpoint<RawSubscription<T>>);
impl<T: Send + Sync + 'static> NodeSubscription<T> {
    pub async fn declare(
        node: &Node,
        key: &str,
        topic: TopicToken,
        qos: &Ros2Qos,
        capacity: usize,
        max_payload_size: usize,
        decoder: Arc<Decoder<T>>,
    ) -> Result<Self, PubSubError> {
        let id = node.allocate_entity();
        let subscription = RawSubscription::declare_with_qos(
            node.session(),
            key,
            capacity,
            max_payload_size,
            decoder,
            qos,
        )
        .await?;
        let endpoint = node
            .declare_endpoint_with_id(subscription, id, EntityKind::Subscription, topic)
            .await
            .map_err(|error| PubSubError::Token(error.to_string()))?;
        Ok(Self(endpoint))
    }
    pub async fn recv_async(&self) -> Result<(T, MessageMetadata), PubSubError> {
        self.0.data_entity.recv_async().await
    }
    pub fn try_recv(&self) -> Result<Option<(T, MessageMetadata)>, PubSubError> {
        self.0.data_entity.try_recv()
    }
    pub fn stats(&self) -> SubscriptionStats {
        self.0.data_entity.stats()
    }
}

pub fn register_qos_event(event: &'static str) -> Result<(), TransportError> {
    Err(TransportError::UnsupportedQosEvent { event })
}

#[derive(Debug, Error)]
pub enum PubSubError {
    #[error("publisher sequence exhausted")]
    SequenceExhausted,
    #[error("system timestamp is before Unix epoch")]
    TimestampBeforeEpoch,
    #[error("system timestamp does not fit in i64 nanoseconds")]
    TimestampOverflow,
    #[error("ROS2 history depth must be positive, got {0}")]
    InvalidDepth(i32),
    #[error("subscription capacity must be positive")]
    ZeroCapacity,
    #[error(transparent)]
    Attachment(#[from] AttachmentError),
    #[error("payload is {actual} bytes, maximum is {maximum}")]
    PayloadTooLarge { actual: usize, maximum: usize },
    #[error("subscription queue is empty")]
    Empty,
    #[error("subscription is closed")]
    Closed,
    #[error("CDR decode failed: {0}")]
    Decode(String),
    #[error("Zenoh pub/sub operation failed: {0}")]
    Session(String),
    #[error("failed to declare ROS graph endpoint: {0}")]
    Token(String),
}
