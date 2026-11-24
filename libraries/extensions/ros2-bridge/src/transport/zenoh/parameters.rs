//! ROS 2 parameters for a node on the native Zenoh transport: the six
//! `rcl_interfaces` parameter services and `/parameter_events`, behaving like
//! ros2-client does for a DDS node (#3866).

use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use dora_message::descriptor::RmwZenohCompatibility;
use futures::{FutureExt, select_biased};
use ros2_client::{
    NodeName, ParameterValue,
    builtin_interfaces::Time,
    parameters::{ParameterDescriptor, raw},
    rcl_interfaces as rcl,
};
use serde::{Serialize, de::DeserializeOwned};

use super::{
    Node, TransportCdrError,
    compatibility::{
        RosTypeIdentity, TypeDescriptionResolver, TypeIdentityError, resolve_message,
        resolve_service,
    },
    deserialize_cdr,
    keyexpr::{DataKey, KeyError, TopicToken},
    pubsub::{NodePublisher, NodeSubscription, PubSubError},
    qos::ZenohQosMapping,
    serialize_cdr,
    service::{
        NodeServiceClient, NodeServiceServer, ServiceError, ServiceRequest, wait_for_service,
    },
};
use crate::transport::parameters::{
    PARAMETER_QOS_DEPTH, ParameterService as Service, check_target, parameters_qos,
};

/// The built-in parameter every node has, as ros2-client declares it.
const USE_SIM_TIME: &str = "use_sim_time";

/// Checks a value before it is set; `Err` is the reason sent back.
pub type ParameterValidator =
    Box<dyn Fn(&str, &ParameterValue) -> Result<(), String> + Send + Sync>;

#[derive(Debug, thiserror::Error)]
pub enum ParameterError {
    #[error(transparent)]
    Type(#[from] TypeIdentityError),
    #[error("invalid Zenoh key for `{name}`: {error}")]
    Key { name: String, error: KeyError },
    #[error(transparent)]
    Service(#[from] ServiceError),
    #[error(transparent)]
    Publisher(#[from] PubSubError),
    #[error(transparent)]
    Cdr(#[from] TransportCdrError),
    #[error("{0}")]
    Target(String),
}

/// The `rcl_interfaces` types a parameter server needs. Resolve them before
/// creating the node, so a missing type leaves nothing in the ROS graph.
pub struct ParameterTypes {
    services: Vec<(Service, RosTypeIdentity)>,
    event: RosTypeIdentity,
}

impl ParameterTypes {
    pub fn resolve(
        compatibility: RmwZenohCompatibility,
        resolver: &TypeDescriptionResolver,
    ) -> Result<Self, ParameterError> {
        let services = Service::ALL
            .into_iter()
            .map(|service| {
                Ok((
                    service,
                    resolve_service(
                        compatibility,
                        "rcl_interfaces",
                        service.type_name(),
                        resolver,
                    )?,
                ))
            })
            .collect::<Result<_, ParameterError>>()?;
        Ok(Self {
            services,
            event: resolve_message(compatibility, "rcl_interfaces", "ParameterEvent", resolver)?,
        })
    }
}

/// `rcl_interfaces/msg/ParameterEvent`. ros2-client's `raw::ParameterEvent`
/// stamps it with a `rustdds::Timestamp` (seconds + 2^-32 fractions), which
/// is not the `builtin_interfaces/Time` (seconds + nanoseconds) ROS expects.
#[derive(Serialize)]
struct ParameterEvent<'a> {
    stamp: Time,
    node: &'a str,
    new_parameters: Vec<raw::Parameter>,
    changed_parameters: Vec<raw::Parameter>,
    deleted_parameters: Vec<raw::Parameter>,
}

/// `rcl_interfaces/srv/SetParametersAtomically` replies with one result.
/// (ros2-client's alias reuses `SetParameters`' list of results.)
#[derive(Serialize)]
struct SetParametersAtomicallyResponse {
    result: raw::SetParametersResult,
}

struct Store {
    values: BTreeMap<String, ParameterValue>,
    validator: Option<ParameterValidator>,
}

impl Store {
    /// The same checks as ros2-client's `Spinner::set_parameter`: only
    /// declared parameters, `use_sim_time` must stay a bool, then the
    /// validator.
    fn check(&self, name: &str, value: &ParameterValue) -> Result<(), String> {
        if !self.values.contains_key(name) {
            return Err(format!(
                "Setting undeclared parameter '{name}' is not allowed."
            ));
        }
        match (name, &self.validator) {
            (USE_SIM_TIME, _) if !matches!(value, ParameterValue::Boolean(_)) => {
                Err(format!("Parameter '{USE_SIM_TIME}' must be Boolean."))
            }
            (USE_SIM_TIME, _) | (_, None) => Ok(()),
            // A panicking validator rejects the value instead of stopping the
            // thread that serves the parameter services.
            (_, Some(validator)) => {
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| validator(name, value)))
                    .unwrap_or_else(|_| {
                        Err(format!("the validator for parameter '{name}' panicked"))
                    })
            }
        }
    }

    /// Sets all of `parameters` if every one passes `check`, else none of
    /// them. Returns what changed, for `/parameter_events`.
    fn set_all(
        &mut self,
        parameters: Vec<(String, ParameterValue)>,
    ) -> Result<Vec<raw::Parameter>, String> {
        parameters
            .iter()
            .try_for_each(|(name, value)| self.check(name, value))?;
        Ok(parameters
            .into_iter()
            .map(|(name, value)| {
                self.values.insert(name.clone(), value.clone());
                raw::Parameter {
                    name,
                    value: value.into(),
                }
            })
            .collect())
    }
}

struct Shared {
    node: String,
    store: Mutex<Store>,
    events: NodePublisher,
    /// Held from storing a set to publishing its event, so racing sets
    /// publish in the order they were stored.
    publishing: futures::lock::Mutex<()>,
}

/// Decodes a request, answers it with `respond`, and encodes the response.
fn handle<Q: DeserializeOwned, R: Serialize>(
    payload: &[u8],
    respond: impl FnOnce(Q) -> R,
) -> Result<Vec<u8>, TransportCdrError> {
    serialize_cdr(&respond(deserialize_cdr(payload)?))
}

impl Shared {
    fn store(&self) -> std::sync::MutexGuard<'_, Store> {
        self.store
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Sets `parameters` all or nothing and publishes the change.
    async fn set(&self, parameters: Vec<(String, ParameterValue)>) -> Result<(), String> {
        let _in_order = self.publishing.lock().await;
        let changed = self.store().set_all(parameters)?;
        if changed.is_empty() {
            return Ok(());
        }
        let since_epoch = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        let event = ParameterEvent {
            stamp: Time::from_nanos(i64::try_from(since_epoch.as_nanos()).unwrap_or(i64::MAX)),
            node: &self.node,
            new_parameters: Vec::new(),
            changed_parameters: changed,
            deleted_parameters: Vec::new(),
        };
        match serialize_cdr(&event) {
            Ok(payload) => {
                if let Err(error) = self.events.publish(&payload).await {
                    tracing::warn!("failed to publish a ROS 2 parameter event: {error}");
                }
            }
            Err(error) => tracing::warn!("failed to encode a ROS 2 parameter event: {error}"),
        }
        Ok(())
    }

    /// Answers one request, or rejects it when its payload doesn't decode.
    async fn answer(&self, service: Service, server: &NodeServiceServer, request: ServiceRequest) {
        let sent = match self.respond(service, &request.payload).await {
            Ok(payload) => server.reply(request.id, &payload).await,
            Err(error) => server.reject(request.id, &error.to_string()).await,
        };
        if let Err(error) = sent {
            tracing::warn!("failed to answer a ROS 2 parameter request: {error}");
        }
    }

    async fn respond(
        &self,
        service: Service,
        payload: &[u8],
    ) -> Result<Vec<u8>, TransportCdrError> {
        match service {
            Service::Get => handle(payload, |request: rcl::GetParametersRequest| {
                let store = self.store();
                let values = request
                    .names
                    .iter()
                    .map(|name| {
                        store
                            .values
                            .get(name)
                            .cloned()
                            .unwrap_or(ParameterValue::NotSet)
                            .into()
                    })
                    .collect();
                rcl::GetParametersResponse { values }
            }),
            Service::GetTypes => handle(payload, |request: rcl::GetParameterTypesRequest| {
                let store = self.store();
                let values = request
                    .names
                    .iter()
                    .map(|name| {
                        store.values.get(name).map_or(
                            raw::ParameterType::NOT_SET,
                            ParameterValue::to_parameter_type_raw,
                        )
                    })
                    .collect();
                rcl::GetParameterTypesResponse { values }
            }),
            Service::Set => {
                // Each parameter on its own, like ros2-client and rclcpp.
                let request: rcl::SetParametersRequest = deserialize_cdr(payload)?;
                let mut results = Vec::with_capacity(request.parameter.len());
                for parameter in request.parameter {
                    let parameter = ros2_client::Parameter::from(parameter);
                    let result = self.set(vec![(parameter.name, parameter.value)]).await;
                    results.push(result.into());
                }
                serialize_cdr(&rcl::SetParametersResponse { results })
            }
            Service::SetAtomically => {
                let request: rcl::SetParametersRequest = deserialize_cdr(payload)?;
                let parameters = request
                    .parameter
                    .into_iter()
                    .map(|parameter| {
                        let parameter = ros2_client::Parameter::from(parameter);
                        (parameter.name, parameter.value)
                    })
                    .collect();
                let result = self.set(parameters).await;
                serialize_cdr(&SetParametersAtomicallyResponse {
                    result: result.into(),
                })
            }
            Service::List => handle(payload, |request: rcl::ListParametersRequest| {
                let names = self
                    .store()
                    .values
                    .keys()
                    .filter(|name| {
                        request.prefixes.is_empty()
                            || request
                                .prefixes
                                .iter()
                                .any(|prefix| name.starts_with(prefix))
                    })
                    .cloned()
                    .collect();
                rcl::ListParametersResponse {
                    result: rcl::ListParametersResult {
                        names,
                        prefixes: Vec::new(),
                    },
                }
            }),
            Service::Describe => handle(payload, |request: rcl::DescribeParametersRequest| {
                let store = self.store();
                let values = request
                    .names
                    .iter()
                    .map(|name| match store.values.get(name) {
                        Some(value) => ParameterDescriptor::from_value(name, value),
                        None => ParameterDescriptor::unknown(name),
                    })
                    .map(raw::ParameterDescriptor::from)
                    .collect();
                rcl::DescribeParametersResponse { values }
            }),
        }
    }
}

/// A Zenoh node's ROS 2 parameters: the values, the six parameter services
/// (served by a background thread) and the `/parameter_events` publisher.
/// Dropping it stops the services.
pub struct ParameterServer {
    shared: Arc<Shared>,
    _stop: flume::Sender<()>,
}

impl ParameterServer {
    /// The services live under `node_name`'s fully qualified name. Like
    /// ros2-client, `use_sim_time` is always declared as `false`, replacing
    /// any value in `declared`.
    pub async fn declare(
        node: &Node,
        node_name: &NodeName,
        types: &ParameterTypes,
        declared: Vec<(String, ParameterValue)>,
        validator: Option<ParameterValidator>,
    ) -> Result<Self, ParameterError> {
        let node_name = node_name.fully_qualified_name();
        let mut values: BTreeMap<_, _> = declared.into_iter().collect();
        values.insert(USE_SIM_TIME.to_owned(), ParameterValue::Boolean(false));

        let events = key(node, PARAMETER_EVENTS, &types.event)?;
        let events = NodePublisher::declare(
            node,
            events.as_str(),
            token(PARAMETER_EVENTS, &types.event),
            &parameters_qos(),
        )
        .await?;

        let mut servers = Vec::with_capacity(types.services.len());
        for (service, identity) in &types.services {
            let name = format!("{node_name}/{}", service.suffix());
            let server = NodeServiceServer::declare(
                node,
                key(node, &name, identity)?.as_str(),
                token(&name, identity),
                QUEUE_DEPTH as usize,
                REQUEST_EXPIRY,
            )
            .await?;
            servers.push((*service, server));
        }

        let shared = Arc::new(Shared {
            node: node_name.clone(),
            publishing: futures::lock::Mutex::new(()),
            store: Mutex::new(Store { values, validator }),
            events,
        });
        let (stop, stopped) = flume::bounded::<()>(1);
        let serving = shared.clone();
        std::thread::Builder::new()
            .name(format!("ros2-parameters {node_name}"))
            .spawn(move || futures::executor::block_on(serve(serving, servers, stopped)))
            .map_err(|error| ServiceError::Zenoh(error.to_string()))?;
        Ok(Self {
            shared,
            _stop: stop,
        })
    }

    pub fn get(&self, name: &str) -> Option<ParameterValue> {
        self.shared.store().values.get(name).cloned()
    }

    pub fn has(&self, name: &str) -> bool {
        self.shared.store().values.contains_key(name)
    }

    pub fn list(&self) -> Vec<String> {
        self.shared.store().values.keys().cloned().collect()
    }

    /// Sets a declared parameter, with the same checks as a remote
    /// `set_parameters` call, and publishes the change on `/parameter_events`.
    pub async fn set(&self, name: &str, value: ParameterValue) -> Result<(), String> {
        self.shared.set(vec![(name.to_owned(), value)]).await
    }
}

const PARAMETER_EVENTS: &str = "/parameter_events";

/// The graph token of a parameter service or of `/parameter_events`.
fn token(name: &str, identity: &RosTypeIdentity) -> TopicToken {
    TopicToken {
        name: name.to_owned(),
        type_name: identity.dds_name.clone(),
        type_hash: identity.key_hash_component(),
        qos: ZenohQosMapping::from_ros_qos(&parameters_qos()).to_string(),
    }
}

fn key(node: &Node, name: &str, identity: &RosTypeIdentity) -> Result<DataKey, ParameterError> {
    DataKey::new(node.domain(), name, identity).map_err(|error| ParameterError::Key {
        name: name.to_owned(),
        error,
    })
}

/// The QoS history depth of the parameter services and events, and so also
/// how many requests a parameter service holds before it drops new ones.
const QUEUE_DEPTH: i32 = PARAMETER_QOS_DEPTH;
/// How long a request may wait for its reply before it expires.
const REQUEST_EXPIRY: Duration = Duration::from_secs(30);

/// Answers the parameter services one request at a time until the
/// `ParameterServer` is dropped (or a server's transport closes).
async fn serve(
    shared: Arc<Shared>,
    servers: Vec<(Service, NodeServiceServer)>,
    stopped: flume::Receiver<()>,
) {
    let mut stopped = stopped.recv_async().fuse();
    // `select_all` takes the first ready request in list order, so each
    // round starts after the service served last: the services take turns
    // and a busy one can't starve the others.
    let mut first = 0;
    loop {
        let next = futures::future::select_all(
            servers
                .iter()
                .enumerate()
                .cycle()
                .skip(first)
                .take(servers.len())
                .map(|(index, (service, server))| {
                    async move { (index, *service, server, server.recv().await) }.boxed()
                }),
        )
        .fuse();
        futures::pin_mut!(next);
        select_biased! {
            _ = stopped => break,
            ((index, service, server, request), _, _) = next => {
                first = (index + 1) % servers.len();
                match request {
                    Ok(request) => shared.answer(service, server, request).await,
                    Err(ServiceError::TransportClosed) => break,
                    Err(error) => tracing::warn!("ROS 2 parameter request failed: {error}"),
                }
            }
        }
    }
}

/// A client for the parameter services and `/parameter_events` of the node
/// with the fully qualified name `target`.
pub struct ParameterClient {
    target: String,
    services: Vec<(Service, NodeServiceClient, TopicToken)>,
    events: NodeSubscription<raw::ParameterEvent>,
}

/// Requests one parameter client may have waiting at once.
const CLIENT_PENDING_REQUESTS: usize = 16;
/// `/parameter_events` messages a client holds before the oldest are dropped.
const EVENT_CAPACITY: usize = 256;
/// The largest `/parameter_events` message a client accepts.
const MAX_EVENT_SIZE: usize = 1024 * 1024;

impl ParameterClient {
    pub async fn declare(
        node: &Node,
        target: &str,
        types: &ParameterTypes,
    ) -> Result<Self, ParameterError> {
        check_target(target).map_err(ParameterError::Target)?;
        let mut services = Vec::with_capacity(types.services.len());
        for (service, identity) in &types.services {
            let name = format!("{target}/{}", service.suffix());
            let token = token(&name, identity);
            let client = NodeServiceClient::declare(
                node,
                key(node, &name, identity)?.as_str(),
                token.clone(),
                CLIENT_PENDING_REQUESTS,
            )
            .await?;
            services.push((*service, client, token));
        }
        let events = NodeSubscription::declare(
            node,
            key(node, PARAMETER_EVENTS, &types.event)?.as_str(),
            token(PARAMETER_EVENTS, &types.event),
            &parameters_qos(),
            EVENT_CAPACITY,
            MAX_EVENT_SIZE,
            Arc::new(|bytes: &[u8]| {
                deserialize_cdr::<raw::ParameterEvent>(bytes)
                    .map_err(|error| PubSubError::Decode(error.to_string()))
            }),
        )
        .await?;
        Ok(Self {
            target: target.to_owned(),
            services,
            events,
        })
    }

    /// Waits until all six services of the target are in the graph.
    pub async fn wait_for_services(
        &self,
        node: &Node,
        timeout: Duration,
    ) -> Result<(), ParameterError> {
        let deadline = std::time::Instant::now() + timeout;
        for (_, _, token) in &self.services {
            wait_for_service(
                node.graph(),
                &token.name,
                &token.type_name,
                &token.type_hash,
                &token.qos,
                deadline,
            )
            .await?;
        }
        Ok(())
    }

    pub fn target(&self) -> &str {
        &self.target
    }

    /// Calls the target's `service` with `request`.
    pub async fn call<Q: Serialize, R: DeserializeOwned>(
        &self,
        service: Service,
        request: &Q,
        timeout: Duration,
    ) -> Result<R, ParameterError> {
        let client = self
            .services
            .iter()
            .find_map(|(declared, client, _)| (*declared == service).then_some(client))
            .ok_or(ParameterError::Service(ServiceError::TransportClosed))?;
        let response = client.call(serialize_cdr(request)?, timeout).await?;
        Ok(deserialize_cdr(&response)?)
    }

    /// The next `/parameter_events` message that has arrived, of any node.
    pub fn try_recv_event(&self) -> Result<Option<raw::ParameterEvent>, ParameterError> {
        Ok(self.events.try_recv()?.map(|(event, _)| event))
    }

    /// Waits for the next `/parameter_events` message, of any node.
    pub async fn recv_event(&self) -> Result<raw::ParameterEvent, ParameterError> {
        Ok(self.events.recv_async().await?.0)
    }
}
