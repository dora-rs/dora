//! A client for another ROS 2 node's parameters: its six `rcl_interfaces`
//! parameter services and its changes on `/parameter_events` (#4297).

use std::time::Duration;

use super::{Durability, History, Liveliness, Reliability, Ros2Qos};
use ros2_client::{
    ParameterValue,
    parameters::{SetParametersResult, raw},
    rcl_interfaces as rcl,
};

mod dds;
pub use dds::DdsParameterClient;

/// rmw's `rmw_qos_profile_parameter_events` and `rmw_qos_profile_parameters`:
/// what parameter services and `/parameter_events` use.
pub fn parameters_qos() -> Ros2Qos {
    Ros2Qos {
        reliability: Reliability::Reliable {
            max_blocking_time: Duration::ZERO,
        },
        durability: Durability::Volatile,
        history: History::KeepLast { depth: 1000 },
        liveliness: Liveliness::Automatic {
            lease_duration: None,
        },
    }
}

/// The six `rcl_interfaces` parameter services every ROS 2 node offers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParameterService {
    Get,
    GetTypes,
    Set,
    SetAtomically,
    List,
    Describe,
}

impl ParameterService {
    pub const ALL: [Self; 6] = [
        Self::Get,
        Self::GetTypes,
        Self::Set,
        Self::SetAtomically,
        Self::List,
        Self::Describe,
    ];

    /// The service's name under its node, e.g. `get_parameters`.
    pub fn suffix(self) -> &'static str {
        match self {
            Self::Get => "get_parameters",
            Self::GetTypes => "get_parameter_types",
            Self::Set => "set_parameters",
            Self::SetAtomically => "set_parameters_atomically",
            Self::List => "list_parameters",
            Self::Describe => "describe_parameters",
        }
    }

    /// Its `rcl_interfaces/srv` type, e.g. `GetParameters`.
    pub fn type_name(self) -> &'static str {
        match self {
            Self::Get => "GetParameters",
            Self::GetTypes => "GetParameterTypes",
            Self::Set => "SetParameters",
            Self::SetAtomically => "SetParametersAtomically",
            Self::List => "ListParameters",
            Self::Describe => "DescribeParameters",
        }
    }
}

/// One change to the target node's parameters, from `/parameter_events`.
#[derive(Debug, Clone, PartialEq)]
pub struct ParameterChange {
    pub new: Vec<(String, ParameterValue)>,
    pub changed: Vec<(String, ParameterValue)>,
    pub deleted: Vec<String>,
}

impl ParameterChange {
    /// Whether nothing was added, changed or deleted.
    pub fn is_empty(&self) -> bool {
        self.new.is_empty() && self.changed.is_empty() && self.deleted.is_empty()
    }

    pub fn from_event(event: raw::ParameterEvent) -> Self {
        let values = |parameters: Vec<raw::Parameter>| {
            parameters
                .into_iter()
                .map(|parameter| (parameter.name, parameter.value.into()))
                .collect()
        };
        Self {
            new: values(event.new_parameters),
            changed: values(event.changed_parameters),
            deleted: event
                .deleted_parameters
                .into_iter()
                .map(|parameter| parameter.name)
                .collect(),
        }
    }
}

/// The parameter names and prefixes `list_parameters` returned.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParameterList {
    pub names: Vec<String>,
    pub prefixes: Vec<String>,
}

#[derive(Debug, thiserror::Error)]
pub enum ParameterClientError {
    #[error("`{service}` of {target} failed: {error}")]
    Call {
        service: &'static str,
        target: String,
        error: String,
    },
    #[error("`{service}` of {target} answered {got} entries for {asked} names")]
    Length {
        service: &'static str,
        target: String,
        asked: usize,
        got: usize,
    },
    #[error("parameter events of {target} failed: {error}")]
    Events { target: String, error: String },
}

/// A client for the parameters of one ROS 2 node, on either transport.
/// Both are boxed: each holds six service clients and a subscription.
pub enum ParameterClient {
    Dds(Box<DdsParameterClient>),
    #[cfg(feature = "rmw-zenoh")]
    Zenoh(Box<super::zenoh::parameters::ParameterClient>),
}

/// Calls `$method` of the DDS client, or `$service` of the Zenoh one.
macro_rules! dispatch {
    ($self:ident, $service:expr, $method:ident, $request:expr, $timeout:expr) => {{
        let service: ParameterService = $service;
        let request = $request;
        match $self {
            Self::Dds(client) => client
                .$method(request, $timeout)
                .await
                .map_err(|error| $self.call_error(service, error)),
            #[cfg(feature = "rmw-zenoh")]
            Self::Zenoh(client) => client
                .call(service, &request, $timeout)
                .await
                .map_err(|error| $self.call_error(service, error)),
        }
    }};
}

impl ParameterClient {
    /// The target node's fully qualified name.
    pub fn target(&self) -> &str {
        match self {
            Self::Dds(client) => client.target(),
            #[cfg(feature = "rmw-zenoh")]
            Self::Zenoh(client) => client.target(),
        }
    }

    /// Turns a transport error of `service` into a [`ParameterClientError`].
    fn call_error(
        &self,
        service: ParameterService,
        error: impl std::fmt::Display,
    ) -> ParameterClientError {
        ParameterClientError::Call {
            service: service.suffix(),
            target: self.target().to_owned(),
            error: error.to_string(),
        }
    }

    /// Fails unless `service` answered one entry for each of `asked` names.
    fn check_length(
        &self,
        service: ParameterService,
        asked: usize,
        got: usize,
    ) -> Result<(), ParameterClientError> {
        if asked == got {
            Ok(())
        } else {
            Err(ParameterClientError::Length {
                service: service.suffix(),
                target: self.target().to_owned(),
                asked,
                got,
            })
        }
    }

    /// The values of `names`; `None` for a name the target does not have.
    pub async fn get(
        &self,
        names: &[String],
        timeout: Duration,
    ) -> Result<Vec<Option<ParameterValue>>, ParameterClientError> {
        let service = ParameterService::Get;
        let values = per_name(self.target(), service, names, |names| async move {
            let response: rcl::GetParametersResponse = dispatch!(
                self,
                service,
                get,
                rcl::GetParametersRequest { names },
                timeout
            )?;
            Ok(response.values)
        })
        .await?;
        Ok(values
            .into_iter()
            .map(|value| match value.map(ParameterValue::from) {
                None | Some(ParameterValue::NotSet) => None,
                Some(value) => Some(value),
            })
            .collect())
    }

    /// The `rcl_interfaces/msg/ParameterType` of each of `names`
    /// (`NOT_SET`, 0, for a name the target does not have).
    pub async fn get_types(
        &self,
        names: &[String],
        timeout: Duration,
    ) -> Result<Vec<u8>, ParameterClientError> {
        let service = ParameterService::GetTypes;
        let types = per_name(self.target(), service, names, |names| async move {
            let response: rcl::GetParameterTypesResponse = dispatch!(
                self,
                service,
                get_types,
                rcl::GetParameterTypesRequest { names },
                timeout
            )?;
            Ok(response.values)
        })
        .await?;
        Ok(types
            .into_iter()
            .map(|ty| ty.unwrap_or(raw::ParameterType::NOT_SET))
            .collect())
    }

    /// Sets each of `values` on its own; one result per value.
    pub async fn set(
        &self,
        values: Vec<(String, ParameterValue)>,
        timeout: Duration,
    ) -> Result<Vec<SetParametersResult>, ParameterClientError> {
        let service = ParameterService::Set;
        let asked = values.len();
        let response: rcl::SetParametersResponse = dispatch!(
            self,
            service,
            set,
            rcl::SetParametersRequest {
                parameter: parameters(values),
            },
            timeout
        )?;
        self.check_length(service, asked, response.results.len())?;
        Ok(response.results.into_iter().map(set_result).collect())
    }

    /// Sets all of `values` or none of them.
    pub async fn set_atomically(
        &self,
        values: Vec<(String, ParameterValue)>,
        timeout: Duration,
    ) -> Result<SetParametersResult, ParameterClientError> {
        let response: rcl::SetParametersAtomicallyResponse = dispatch!(
            self,
            ParameterService::SetAtomically,
            set_atomically,
            rcl::SetParametersRequest {
                parameter: parameters(values),
            },
            timeout
        )?;
        Ok(set_result(response.result))
    }

    /// The target's parameter names under `prefixes` (all if empty), to
    /// `depth` levels (0: no limit).
    pub async fn list(
        &self,
        prefixes: Vec<String>,
        depth: u64,
        timeout: Duration,
    ) -> Result<ParameterList, ParameterClientError> {
        let response: rcl::ListParametersResponse = dispatch!(
            self,
            ParameterService::List,
            list,
            rcl::ListParametersRequest { prefixes, depth },
            timeout
        )?;
        Ok(ParameterList {
            names: response.result.names,
            prefixes: response.result.prefixes,
        })
    }

    /// The descriptors of `names`; `None` for a name the target does not
    /// have.
    pub async fn describe(
        &self,
        names: &[String],
        timeout: Duration,
    ) -> Result<Vec<Option<raw::ParameterDescriptor>>, ParameterClientError> {
        let service = ParameterService::Describe;
        per_name(self.target(), service, names, |names| async move {
            let response: rcl::DescribeParametersResponse = dispatch!(
                self,
                service,
                describe,
                rcl::DescribeParametersRequest { names },
                timeout
            )?;
            Ok(response.values)
        })
        .await
    }

    fn events_error(&self, error: impl std::fmt::Display) -> ParameterClientError {
        ParameterClientError::Events {
            target: self.target().to_owned(),
            error: error.to_string(),
        }
    }

    /// The next change to the target's parameters that has arrived, if any.
    /// Events of other nodes, and events that change nothing (rclcpp and
    /// rclpy publish one for an empty `set_parameters_atomically`), are
    /// skipped.
    pub fn try_next_change(&self) -> Result<Option<ParameterChange>, ParameterClientError> {
        loop {
            let event = match self {
                Self::Dds(client) => client.try_recv_event().map_err(|e| self.events_error(e)),
                #[cfg(feature = "rmw-zenoh")]
                Self::Zenoh(client) => client.try_recv_event().map_err(|e| self.events_error(e)),
            }?;
            match event {
                None => return Ok(None),
                Some(event) if event.node == self.target() => {
                    let change = ParameterChange::from_event(event);
                    if !change.is_empty() {
                        return Ok(Some(change));
                    }
                }
                Some(_) => {}
            }
        }
    }

    /// Waits for the next change to the target's parameters. Events of other
    /// nodes are skipped.
    pub async fn next_change(&self) -> Result<ParameterChange, ParameterClientError> {
        loop {
            let event = match self {
                Self::Dds(client) => client.recv_event().await.map_err(|e| self.events_error(e)),
                #[cfg(feature = "rmw-zenoh")]
                Self::Zenoh(client) => client.recv_event().await.map_err(|e| self.events_error(e)),
            }?;
            if event.node == self.target() {
                let change = ParameterChange::from_event(event);
                if !change.is_empty() {
                    return Ok(change);
                }
            }
        }
    }
}

fn parameters(values: Vec<(String, ParameterValue)>) -> Vec<raw::Parameter> {
    values
        .into_iter()
        .map(|(name, value)| raw::Parameter {
            name,
            value: value.into(),
        })
        .collect()
}

fn set_result(result: raw::SetParametersResult) -> SetParametersResult {
    if result.successful {
        Ok(())
    } else {
        Err(result.reason)
    }
}

/// One answer per name, `None` for a name the target does not have.
///
/// ros2-client answers `NOT_SET` for an undeclared name, but rclcpp and rclpy
/// answer `get_parameters`, `get_parameter_types` and `describe_parameters`
/// with an empty list when any of the names is undeclared. Then each name is
/// asked alone, and an empty answer means that name is undeclared.
async fn per_name<T, F, Fut>(
    target: &str,
    service: ParameterService,
    names: &[String],
    mut call: F,
) -> Result<Vec<Option<T>>, ParameterClientError>
where
    F: FnMut(Vec<String>) -> Fut,
    Fut: std::future::Future<Output = Result<Vec<T>, ParameterClientError>>,
{
    let answer = call(names.to_vec()).await?;
    let length_error = |asked, got| ParameterClientError::Length {
        service: service.suffix(),
        target: target.to_owned(),
        asked,
        got,
    };
    if answer.len() == names.len() {
        return Ok(answer.into_iter().map(Some).collect());
    }
    if !answer.is_empty() {
        return Err(length_error(names.len(), answer.len()));
    }
    let mut each = Vec::with_capacity(names.len());
    for name in names {
        let mut one = call(vec![name.clone()]).await?;
        match one.len() {
            0 => each.push(None),
            1 => each.push(one.pop()),
            got => return Err(length_error(1, got)),
        }
    }
    Ok(each)
}

#[cfg(test)]
mod per_name_tests {
    use futures::executor::block_on;

    use super::{ParameterClientError, ParameterService, per_name};

    /// A target that has `declared` and answers like rclpy, or like
    /// ros2-client (`0` for an undeclared name) when `fills_missing`.
    fn target(
        declared: &'static [&'static str],
        fills_missing: bool,
    ) -> impl FnMut(Vec<String>) -> std::future::Ready<Result<Vec<u8>, ParameterClientError>> {
        move |names: Vec<String>| {
            let known = |name: &String| declared.contains(&name.as_str());
            std::future::ready(Ok(if fills_missing {
                names.iter().map(|name| u8::from(known(name))).collect()
            } else if names.iter().all(known) {
                vec![1; names.len()]
            } else {
                vec![]
            }))
        }
    }

    fn names(names: &[&str]) -> Vec<String> {
        names.iter().map(|name| name.to_string()).collect()
    }

    #[test]
    fn a_full_answer_is_used_as_is() {
        let answer = block_on(per_name(
            "/arm",
            ParameterService::GetTypes,
            &names(&["speed", "gains"]),
            target(&["speed", "gains"], false),
        ))
        .unwrap();
        assert_eq!(answer, [Some(1), Some(1)]);
    }

    #[test]
    fn an_empty_answer_is_asked_again_name_by_name() {
        let answer = block_on(per_name(
            "/arm",
            ParameterService::GetTypes,
            &names(&["speed", "missing", "gains"]),
            target(&["speed", "gains"], false),
        ))
        .unwrap();
        assert_eq!(answer, [Some(1), None, Some(1)]);
    }

    #[test]
    fn no_names_need_no_retry() {
        let answer = block_on(per_name(
            "/arm",
            ParameterService::Get,
            &[],
            target(&[], false),
        ))
        .unwrap();
        assert!(answer.is_empty());
    }

    #[test]
    fn a_partial_answer_is_an_error() {
        let error = block_on(per_name(
            "/arm",
            ParameterService::Describe,
            &names(&["speed", "gains"]),
            |_| std::future::ready(Ok(vec![1_u8])),
        ))
        .unwrap_err();
        assert!(
            matches!(
                error,
                ParameterClientError::Length {
                    asked: 2,
                    got: 1,
                    ..
                }
            ),
            "{error}"
        );
    }
}

#[cfg(test)]
mod tests {
    use ros2_client::{
        ParameterValue,
        builtin_interfaces::Time,
        parameters::raw::{self, ParameterEvent},
    };

    use super::ParameterChange;

    fn parameter(name: &str, value: ParameterValue) -> raw::Parameter {
        raw::Parameter {
            name: name.into(),
            value: value.into(),
        }
    }

    #[test]
    fn an_event_without_parameters_is_no_change() {
        let event = ParameterEvent {
            stamp: Time::ZERO,
            node: "/robot/arm".into(),
            new_parameters: vec![],
            changed_parameters: vec![],
            deleted_parameters: vec![],
        };
        assert!(ParameterChange::from_event(event).is_empty());
    }

    #[test]
    fn a_parameter_event_becomes_a_change() {
        let event = ParameterEvent {
            stamp: Time::ZERO,
            node: "/robot/arm".into(),
            new_parameters: vec![parameter("gains", ParameterValue::DoubleArray(vec![1.0]))],
            changed_parameters: vec![parameter("speed", ParameterValue::Double(2.0))],
            deleted_parameters: vec![parameter("old", ParameterValue::NotSet)],
        };
        assert_eq!(
            ParameterChange::from_event(event),
            ParameterChange {
                new: vec![("gains".into(), ParameterValue::DoubleArray(vec![1.0]))],
                changed: vec![("speed".into(), ParameterValue::Double(2.0))],
                deleted: vec!["old".into()],
            }
        );
    }
}
