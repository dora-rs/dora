//! The parameter client on the DDS transport, through ros2-client.

use std::time::Duration;

use futures::{
    future::{Either, select},
    lock::Mutex,
};
use ros2_client::{
    Client, Message, MessageTypeName, Name, ServiceMapping, ServiceTypeName, Subscription,
    parameters::raw, rcl_interfaces as rcl,
};

use super::{ParameterService, parameters_qos};
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
    events: Subscription<raw::ParameterEvent>,
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
        if !target.starts_with('/') {
            return Err(format!("`{target}` is not a fully qualified node name"));
        }
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
        let events = node
            .create_subscription(&topic, Some(qos.clone()))
            .map_err(|error| format!("failed to subscribe to /parameter_events: {error:?}"))?;
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

    /// Waits until the target's six services are matched, or `timeout`.
    pub async fn wait_for_services(
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

    pub(super) fn try_recv_event(&self) -> Result<Option<raw::ParameterEvent>, String> {
        self.events
            .take()
            .map(|event| event.map(|(event, _)| event))
            .map_err(|error| error.to_string())
    }

    pub(super) async fn recv_event(&self) -> Result<raw::ParameterEvent, String> {
        self.events
            .async_take()
            .await
            .map(|(event, _)| event)
            .map_err(|error| error.to_string())
    }
}

async fn call<Q: Message + 'static, R: Message + 'static>(
    client: &OneAtATime<Q, R>,
    request: Q,
    timeout: Duration,
) -> Result<R, String> {
    let client = client.lock().await;
    let call = async {
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
