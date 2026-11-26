//! The parameter client on the DDS transport, through ros2-client.

use std::time::{Duration, Instant};

use futures::{
    future::{Either, select},
    lock::Mutex,
};
use ros2_client::{
    Client, Message, MessageTypeName, Name, ServiceMapping, ServiceTypeName, Subscription,
    parameters::raw, rcl_interfaces as rcl,
};

use super::{ParameterEvents, ParameterService, check_target, parameters_qos, spawn_event_pump};
use crate::transport::dds::{detect_service_mapping, to_qos_profile};

/// One call at a time: `Client::async_receive_response(id)` drops the
/// responses for other ids, so concurrent calls on one client would steal
/// each other's responses.
type OneAtATime<Q, R> = Mutex<Client<Q, R>>;

/// A client for the parameter services and `/parameter_events` of the node
/// with the fully qualified name `target`.
pub struct DdsParameterClient {
    target: String,
    get: OneAtATime<rcl::GetParametersRequest, rcl::GetParametersResponse>,
    get_types: OneAtATime<rcl::GetParameterTypesRequest, rcl::GetParameterTypesResponse>,
    set: OneAtATime<rcl::SetParametersRequest, rcl::SetParametersResponse>,
    set_atomically: OneAtATime<rcl::SetParametersRequest, rcl::SetParametersAtomicallyResponse>,
    list: OneAtATime<rcl::ListParametersRequest, rcl::ListParametersResponse>,
    describe: OneAtATime<rcl::DescribeParametersRequest, rcl::DescribeParametersResponse>,
    events: ParameterEvents,
}

impl DdsParameterClient {
    /// A client with the service mapping of `RMW_IMPLEMENTATION` (see
    /// [`detect_service_mapping`]).
    pub fn new(node: &mut ros2_client::Node, target: &str) -> Result<Self, String> {
        Self::with_service_mapping(node, target, detect_service_mapping())
    }

    pub fn with_service_mapping(
        node: &mut ros2_client::Node,
        target: &str,
        mapping: ServiceMapping,
    ) -> Result<Self, String> {
        check_target(target)?;
        let qos = to_qos_profile(&parameters_qos());
        macro_rules! client {
            ($service:expr) => {{
                let service: ParameterService = $service;
                let name = Name::new(target, service.suffix())
                    .map_err(|error| format!("invalid service name under `{target}`: {error:?}"))?;
                Mutex::new(
                    node.create_client(
                        mapping,
                        &name,
                        &ServiceTypeName::new("rcl_interfaces", service.type_name()),
                        qos.clone(),
                        qos.clone(),
                    )
                    .map_err(|error| {
                        format!("failed to create `{}`: {error:?}", service.suffix())
                    })?,
                )
            }};
        }
        let topic = node
            .create_topic(
                &Name::new("/", "parameter_events")
                    .map_err(|error| format!("invalid topic name: {error:?}"))?,
                MessageTypeName::new("rcl_interfaces", "ParameterEvent"),
                &qos,
            )
            .map_err(|error| format!("failed to create /parameter_events: {error:?}"))?;
        let subscription: Subscription<raw::ParameterEvent> = node
            .create_subscription(&topic, Some(qos.clone()))
            .map_err(|error| format!("failed to subscribe to /parameter_events: {error:?}"))?;
        let events = spawn_event_pump(
            target.to_owned(),
            futures::stream::unfold(subscription, |subscription| async move {
                let event = subscription
                    .async_take()
                    .await
                    .map(|(event, _)| event)
                    .map_err(|error| error.to_string());
                Some((event, subscription))
            }),
        )?;
        Ok(Self {
            target: target.to_owned(),
            get: client!(ParameterService::Get),
            get_types: client!(ParameterService::GetTypes),
            set: client!(ParameterService::Set),
            set_atomically: client!(ParameterService::SetAtomically),
            list: client!(ParameterService::List),
            describe: client!(ParameterService::Describe),
            events,
        })
    }

    pub fn target(&self) -> &str {
        &self.target
    }

    /// Waits until the target's six services are matched and answer, or
    /// `timeout`.
    ///
    /// An rcl server can still drop the first request of a client it has
    /// only just discovered (#4213): it answers before it has matched the
    /// client's response reader. So each service whose request changes
    /// nothing is called once with an empty request, retried until it
    /// answers. `set_parameters_atomically` is left out: rclcpp and rclpy
    /// publish an event for it and run the node's set callbacks even when it
    /// is empty. By the time the others answer, its endpoints are matched.
    pub async fn wait_for_services(
        &self,
        node: &ros2_client::Node,
        timeout: Duration,
    ) -> Result<(), String> {
        let deadline = Instant::now() + timeout;
        self.wait_for_matches(node, timeout).await?;
        // The services are independent, so they warm up together.
        futures::try_join!(
            answered(
                &self.get,
                rcl::GetParametersRequest { names: vec![] },
                deadline
            ),
            answered(
                &self.get_types,
                rcl::GetParameterTypesRequest { names: vec![] },
                deadline,
            ),
            answered(
                &self.describe,
                rcl::DescribeParametersRequest { names: vec![] },
                deadline,
            ),
            answered(
                &self.list,
                rcl::ListParametersRequest {
                    prefixes: vec![],
                    depth: 1,
                },
                deadline,
            ),
            answered(
                &self.set,
                rcl::SetParametersRequest { parameter: vec![] },
                deadline
            ),
        )
        .map(|_| ())
        .map_err(|error| {
            format!(
                "the parameter services of `{}` do not answer: {error}",
                self.target
            )
        })
    }

    async fn wait_for_matches(
        &self,
        node: &ros2_client::Node,
        timeout: Duration,
    ) -> Result<(), String> {
        let all = async {
            self.get.lock().await.wait_for_service(node).await;
            self.get_types.lock().await.wait_for_service(node).await;
            self.set.lock().await.wait_for_service(node).await;
            self.set_atomically
                .lock()
                .await
                .wait_for_service(node)
                .await;
            self.list.lock().await.wait_for_service(node).await;
            self.describe.lock().await.wait_for_service(node).await;
        };
        futures::pin_mut!(all);
        match select(all, futures_timer::Delay::new(timeout)).await {
            Either::Left(((), _)) => Ok(()),
            Either::Right(_) => Err(format!(
                "the parameter services of `{}` did not appear within {timeout:?}",
                self.target
            )),
        }
    }

    pub(super) async fn get(
        &self,
        request: rcl::GetParametersRequest,
        timeout: Duration,
    ) -> Result<rcl::GetParametersResponse, String> {
        call(&self.get, request, timeout).await
    }

    pub(super) async fn get_types(
        &self,
        request: rcl::GetParameterTypesRequest,
        timeout: Duration,
    ) -> Result<rcl::GetParameterTypesResponse, String> {
        call(&self.get_types, request, timeout).await
    }

    pub(super) async fn set(
        &self,
        request: rcl::SetParametersRequest,
        timeout: Duration,
    ) -> Result<rcl::SetParametersResponse, String> {
        call(&self.set, request, timeout).await
    }

    pub(super) async fn set_atomically(
        &self,
        request: rcl::SetParametersRequest,
        timeout: Duration,
    ) -> Result<rcl::SetParametersAtomicallyResponse, String> {
        call(&self.set_atomically, request, timeout).await
    }

    pub(super) async fn list(
        &self,
        request: rcl::ListParametersRequest,
        timeout: Duration,
    ) -> Result<rcl::ListParametersResponse, String> {
        call(&self.list, request, timeout).await
    }

    pub(super) async fn describe(
        &self,
        request: rcl::DescribeParametersRequest,
        timeout: Duration,
    ) -> Result<rcl::DescribeParametersResponse, String> {
        call(&self.describe, request, timeout).await
    }

    pub(super) fn events(&self) -> &ParameterEvents {
        &self.events
    }
}

/// How long one warm-up request waits for its answer before it is resent.
const WARM_UP_ATTEMPT: Duration = Duration::from_millis(500);

/// How long to wait before resending a warm-up request that failed at once.
const WARM_UP_BACKOFF: Duration = Duration::from_millis(100);

/// Calls `client` with `request` until it answers, or `deadline`.
async fn answered<Q: Message + Clone + 'static, R: Message + 'static>(
    client: &OneAtATime<Q, R>,
    request: Q,
    deadline: Instant,
) -> Result<(), String> {
    loop {
        let started = Instant::now();
        let remaining = deadline.saturating_duration_since(started);
        match call(client, request.clone(), WARM_UP_ATTEMPT.min(remaining)).await {
            Ok(_) => return Ok(()),
            Err(error) if remaining <= WARM_UP_ATTEMPT => return Err(error),
            // A call that failed at once (not a timeout) is not resent in a
            // busy loop.
            Err(_) if started.elapsed() < WARM_UP_BACKOFF => {
                futures_timer::Delay::new(WARM_UP_BACKOFF).await;
            }
            Err(_) => {}
        }
    }
}

async fn call<Q: Message + 'static, R: Message + 'static>(
    client: &OneAtATime<Q, R>,
    request: Q,
    timeout: Duration,
) -> Result<R, String> {
    // The timeout covers waiting for an earlier call on the same service.
    let call = async {
        let client = client.lock().await;
        let id = client
            .async_send_request(request)
            .await
            .map_err(|error| error.to_string())?;
        client
            .async_receive_response(id)
            .await
            .map_err(|error| error.to_string())
    };
    futures::pin_mut!(call);
    match select(call, futures_timer::Delay::new(timeout)).await {
        Either::Left((result, _)) => result,
        Either::Right(_) => Err(format!("no response within {timeout:?}")),
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use futures::{
        executor::block_on,
        future::{Either, select},
        lock::Mutex,
    };
    use ros2_client::{
        Context, ContextOptions, Name, NodeName, NodeOptions, ServiceMapping, ServiceTypeName,
        rcl_interfaces as rcl,
    };

    use super::{OneAtATime, call};
    use crate::transport::{dds::to_qos_profile, parameters::parameters_qos};

    /// A call queued behind another on the same service still ends at its
    /// own timeout, instead of waiting for the other call first.
    #[test]
    fn a_call_waiting_for_the_service_still_times_out() {
        let context = Context::with_options(ContextOptions::new().domain_id(91)).unwrap();
        let mut node = context
            .new_node(NodeName::new("/", "lock_test").unwrap(), NodeOptions::new())
            .unwrap();
        let qos = to_qos_profile(&parameters_qos());
        let client: OneAtATime<rcl::GetParametersRequest, rcl::GetParametersResponse> = Mutex::new(
            node.create_client(
                ServiceMapping::Enhanced,
                &Name::new("/nobody", "get_parameters").unwrap(),
                &ServiceTypeName::new("rcl_interfaces", "GetParameters"),
                qos.clone(),
                qos,
            )
            .unwrap(),
        );
        // Another call holds the service.
        let _held = block_on(client.lock());
        let started = Instant::now();
        let call = call(
            &client,
            rcl::GetParametersRequest { names: vec![] },
            Duration::from_millis(200),
        );
        let outcome = block_on(select(
            Box::pin(call),
            futures_timer::Delay::new(Duration::from_secs(2)),
        ));
        let Either::Left((result, _)) = outcome else {
            panic!("the call waited for the other one instead of timing out");
        };
        assert!(result.is_err());
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "{:?}",
            started.elapsed()
        );
    }
}
