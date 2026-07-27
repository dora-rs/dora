pub mod compatibility;

#[cfg(feature = "rmw-zenoh")]
pub mod attachment;
#[cfg(feature = "rmw-zenoh")]
pub mod graph;
#[cfg(feature = "rmw-zenoh")]
pub mod keyexpr;
#[cfg(feature = "rmw-zenoh")]
pub mod pubsub;
#[cfg(feature = "rmw-zenoh")]
pub mod qos;
#[cfg(feature = "rmw-zenoh")]
pub mod service;

#[cfg(feature = "rmw-zenoh")]
pub fn serialize_cdr<T: serde::Serialize>(value: &T) -> Result<Vec<u8>, TransportCdrError> {
    let body = cdr_encoding::to_vec::<_, byteorder::LittleEndian>(value)
        .map_err(|error| TransportCdrError::Encode(error.to_string()))?;
    let mut payload = Vec::with_capacity(body.len() + 4);
    payload.extend_from_slice(&[0, 1, 0, 0]);
    payload.extend_from_slice(&body);
    Ok(payload)
}

#[cfg(feature = "rmw-zenoh")]
pub fn deserialize_cdr<T: serde::de::DeserializeOwned>(
    payload: &[u8],
) -> Result<T, TransportCdrError> {
    if payload.get(..4) != Some(&[0, 1, 0, 0]) {
        return Err(TransportCdrError::Encapsulation);
    }
    let body = &payload[4..];
    let (value, consumed) = cdr_encoding::from_bytes::<T, byteorder::LittleEndian>(body)
        .map_err(|error| TransportCdrError::Decode(error.to_string()))?;
    if consumed != body.len() {
        return Err(TransportCdrError::Trailing(body.len() - consumed));
    }
    Ok(value)
}

#[cfg(feature = "rmw-zenoh")]
#[derive(Debug, thiserror::Error)]
pub enum TransportCdrError {
    #[error("failed to encode ROS2 CDR: {0}")]
    Encode(String),
    #[error("failed to decode ROS2 CDR: {0}")]
    Decode(String),
    #[error("unsupported or missing ROS2 CDR encapsulation")]
    Encapsulation,
    #[error("ROS2 CDR payload has {0} trailing bytes")]
    Trailing(usize),
}

#[cfg(feature = "rmw-zenoh")]
mod lifecycle {
    use std::{
        env,
        path::PathBuf,
        sync::{
            Arc,
            atomic::{AtomicU64, Ordering},
        },
    };

    use thiserror::Error;

    #[derive(Clone, Debug, Eq, PartialEq)]
    pub enum ConfigSource {
        Explicit(PathBuf),
        Environment(PathBuf),
        EmbeddedDefault,
    }

    #[derive(Debug, Error)]
    pub enum ContextError {
        #[error("Zenoh configuration URI must not be empty")]
        EmptyConfigUri,
        #[error("failed to parse Zenoh configuration from {config_source:?}: {message}")]
        InvalidConfig {
            config_source: ConfigSource,
            message: String,
        },
        #[error("failed to open Zenoh session: {0}")]
        Open(String),
        #[error("failed to initialize Zenoh graph: {0}")]
        GraphInitialization(String),
        #[error("failed to declare Zenoh liveliness token: {0}")]
        Token(String),
    }

    pub fn resolve_config_source(
        explicit: Option<&str>,
        environment: Option<&str>,
    ) -> Result<ConfigSource, ContextError> {
        if let Some(uri) = explicit {
            if uri.is_empty() {
                return Err(ContextError::EmptyConfigUri);
            }
            return Ok(ConfigSource::Explicit(uri.into()));
        }
        if let Some(uri) = environment {
            if uri.is_empty() {
                return Err(ContextError::EmptyConfigUri);
            }
            return Ok(ConfigSource::Environment(uri.into()));
        }
        Ok(ConfigSource::EmbeddedDefault)
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct EntityId {
        pub numeric: u64,
        pub gid: [u8; 16],
    }

    pub struct EntityIdAllocator {
        prefix: [u8; 8],
        next: AtomicU64,
    }
    impl EntityIdAllocator {
        pub fn new(prefix: [u8; 8]) -> Self {
            Self {
                prefix,
                next: AtomicU64::new(1),
            }
        }
        pub fn allocate(&self) -> EntityId {
            let numeric = self.next.fetch_add(1, Ordering::Relaxed);
            let mut gid = [0; 16];
            gid[..8].copy_from_slice(&self.prefix);
            gid[8..].copy_from_slice(&numeric.to_le_bytes());
            EntityId { numeric, gid }
        }
    }

    trait ErasedGuard: Send + Sync {}
    impl<T: Send + Sync> ErasedGuard for T {}

    pub struct NodeToken {
        pub key: String,
        _guard: Box<dyn ErasedGuard>,
    }
    impl NodeToken {
        pub fn from_guard(key: impl Into<String>, guard: impl Send + Sync + 'static) -> Self {
            Self {
                key: key.into(),
                _guard: Box::new(guard),
            }
        }
    }
    pub struct EndpointToken {
        pub key: String,
        _guard: Box<dyn ErasedGuard>,
    }
    impl EndpointToken {
        pub fn from_guard(key: impl Into<String>, guard: impl Send + Sync + 'static) -> Self {
            Self {
                key: key.into(),
                _guard: Box::new(guard),
            }
        }
    }

    #[derive(Clone, Debug, Default)]
    pub struct ContextOptions {
        pub domain_id: usize,
        pub config_uri: Option<String>,
    }

    pub struct Context {
        pub graph: super::graph::GraphCache,
        pub config_source: ConfigSource,
        domain: usize,
        ids: Arc<EntityIdAllocator>,
        session: zenoh::Session,
    }

    impl Context {
        pub fn zid(&self) -> String {
            self.session.zid().to_string()
        }

        pub async fn open(options: ContextOptions) -> Result<Self, ContextError> {
            let environment = env::var("ZENOH_SESSION_CONFIG_URI").ok();
            let config_source =
                resolve_config_source(options.config_uri.as_deref(), environment.as_deref())?;
            let config = match &config_source {
                ConfigSource::Explicit(path) | ConfigSource::Environment(path) => {
                    zenoh::Config::from_file(path).map_err(|error| ContextError::InvalidConfig {
                        config_source: config_source.clone(),
                        message: error.to_string(),
                    })?
                }
                ConfigSource::EmbeddedDefault => zenoh::Config::default(),
            };
            let session = zenoh::open(config)
                .await
                .map_err(|error| ContextError::Open(error.to_string()))?;
            let graph = super::graph::GraphCache::new(options.domain_id);
            let selector = format!("@ros2_lv/{}/**", options.domain_id);
            let subscriber = session
                .liveliness()
                .declare_subscriber(selector.clone())
                .await
                .map_err(|error| ContextError::GraphInitialization(error.to_string()))?;
            let replies = session
                .liveliness()
                .get(selector)
                .timeout(std::time::Duration::from_millis(250))
                .await
                .map_err(|error| ContextError::GraphInitialization(error.to_string()))?;
            while let Ok(reply) = replies.recv_async().await {
                if let Ok(sample) = reply.result() {
                    if std::env::var_os("DORA_ROS2_ZENOH_TRACE").is_some() {
                        eprintln!("rmw_zenoh liveliness initial: {}", sample.key_expr());
                    }
                    let _ = graph.apply_put(sample.key_expr().as_str());
                }
            }
            graph.finish_initialization();
            let graph_updates = graph.clone();
            std::thread::Builder::new()
                .name("dora-ros2-zenoh-graph".into())
                .spawn(move || {
                    futures::executor::block_on(async move {
                        use zenoh::sample::SampleKind;
                        while let Ok(sample) = subscriber.recv_async().await {
                            if std::env::var_os("DORA_ROS2_ZENOH_TRACE").is_some() {
                                eprintln!(
                                    "rmw_zenoh liveliness {:?}: {}",
                                    sample.kind(),
                                    sample.key_expr()
                                );
                            }
                            match sample.kind() {
                                SampleKind::Put => {
                                    let _ = graph_updates.apply_put(sample.key_expr().as_str());
                                }
                                SampleKind::Delete => {
                                    let _ = graph_updates.apply_delete(sample.key_expr().as_str());
                                }
                            }
                        }
                    })
                })
                .map_err(|error| ContextError::GraphInitialization(error.to_string()))?;
            let prefix = session.zid().to_le_bytes();
            let mut gid_prefix = [0; 8];
            gid_prefix.copy_from_slice(&prefix[..8]);
            Ok(Self {
                graph,
                config_source,
                domain: options.domain_id,
                ids: Arc::new(EntityIdAllocator::new(gid_prefix)),
                session,
            })
        }

        #[allow(clippy::too_many_arguments)]
        pub async fn create_node(
            &self,
            zid: &str,
            nid: &str,
            enclave: &str,
            namespace: &str,
            name: &str,
        ) -> Result<Node, ContextError> {
            let id = self.ids.allocate();
            let key = super::keyexpr::LivelinessKey::node(
                self.domain,
                zid,
                nid,
                &id.numeric.to_string(),
                enclave,
                namespace,
                name,
            )
            .map_err(|error| ContextError::Token(error.to_string()))?;
            let token = self
                .session
                .liveliness()
                .declare_token(key.as_str())
                .await
                .map_err(|error| ContextError::Token(error.to_string()))?;
            self.graph
                .apply_put_local(key.as_str())
                .map_err(|error| ContextError::Token(error.to_string()))?;
            let token = NodeToken::from_guard(
                key.as_str(),
                DeclaredGraphToken {
                    _token: token,
                    graph: self.graph.clone(),
                    key: key.as_str().into(),
                },
            );
            Ok(Node {
                domain: self.domain,
                zid: zid.into(),
                nid: nid.into(),
                enclave: enclave.into(),
                namespace: namespace.into(),
                name: name.into(),
                session: self.session.clone(),
                graph: self.graph.clone(),
                ids: self.ids.clone(),
                _token: token,
            })
        }
    }

    impl Drop for Context {
        fn drop(&mut self) {
            self.graph.shutdown();
        }
    }

    struct DeclaredGraphToken {
        _token: zenoh::liveliness::LivelinessToken,
        graph: super::graph::GraphCache,
        key: String,
    }
    impl Drop for DeclaredGraphToken {
        fn drop(&mut self) {
            let _ = self.graph.apply_delete(&self.key);
        }
    }

    pub struct Node {
        _token: NodeToken,
        domain: usize,
        zid: String,
        nid: String,
        enclave: String,
        namespace: String,
        name: String,
        session: zenoh::Session,
        graph: super::graph::GraphCache,
        ids: Arc<EntityIdAllocator>,
    }

    pub struct DeclaredEndpoint<D> {
        pub token: EndpointToken,
        pub data_entity: D,
    }

    impl Node {
        pub fn domain(&self) -> usize {
            self.domain
        }

        pub fn session(&self) -> &zenoh::Session {
            &self.session
        }

        pub fn graph(&self) -> &super::graph::GraphCache {
            &self.graph
        }

        pub(super) fn allocate_entity(&self) -> EntityId {
            self.ids.allocate()
        }

        pub fn allocate_gid(&self) -> [u8; 16] {
            self.ids.allocate().gid
        }

        pub async fn declare_endpoint<D: Send + Sync + 'static>(
            &self,
            data_entity: D,
            kind: super::keyexpr::EntityKind,
            topic: super::keyexpr::TopicToken,
        ) -> Result<DeclaredEndpoint<D>, ContextError> {
            let id = self.ids.allocate();
            self.declare_endpoint_with_id(data_entity, id, kind, topic)
                .await
        }

        pub(super) async fn declare_endpoint_with_id<D: Send + Sync + 'static>(
            &self,
            data_entity: D,
            id: EntityId,
            kind: super::keyexpr::EntityKind,
            topic: super::keyexpr::TopicToken,
        ) -> Result<DeclaredEndpoint<D>, ContextError> {
            if kind == super::keyexpr::EntityKind::Node {
                return Err(ContextError::Token("node is not an endpoint kind".into()));
            }
            let key = super::keyexpr::LivelinessKey::endpoint(
                self.domain,
                &self.zid,
                &self.nid,
                &id.numeric.to_string(),
                kind,
                &self.enclave,
                &self.namespace,
                &self.name,
                topic,
            )
            .map_err(|error| ContextError::Token(error.to_string()))?;
            let declared = self
                .session
                .liveliness()
                .declare_token(key.as_str())
                .await
                .map_err(|error| ContextError::Token(error.to_string()))?;
            if std::env::var_os("DORA_ROS2_ZENOH_TRACE").is_some() {
                eprintln!("dora rmw_zenoh liveliness declare: {}", key.as_str());
            }
            self.graph
                .apply_put_local(key.as_str())
                .map_err(|error| ContextError::Token(error.to_string()))?;
            let token = EndpointToken::from_guard(
                key.as_str(),
                DeclaredGraphToken {
                    _token: declared,
                    graph: self.graph.clone(),
                    key: key.as_str().into(),
                },
            );
            Ok(DeclaredEndpoint { token, data_entity })
        }
    }
}

#[cfg(feature = "rmw-zenoh")]
pub use lifecycle::{
    ConfigSource, Context, ContextError, ContextOptions, DeclaredEndpoint, EndpointToken, EntityId,
    EntityIdAllocator, Node, NodeToken, resolve_config_source,
};
