use core::fmt;
use std::{
    collections::{BTreeMap, BTreeSet},
    str::FromStr,
    time::Duration,
};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::OnceLock;

use crate::descriptor;
pub use crate::id::{DataId, NodeId, OperatorId};

/// Filter for the `adora/logs` virtual input.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, JsonSchema)]
pub struct LogSubscriptionFilter {
    /// Minimum log level to receive. `None` means all levels (including stdout).
    #[schemars(with = "Option<String>")]
    pub min_level: Option<crate::common::LogLevelOrStdout>,
    /// Only receive logs from this specific node. `None` means all nodes.
    pub node_filter: Option<NodeId>,
}

/// Default queue size when none is configured.
pub const DEFAULT_QUEUE_SIZE: usize = 10;

/// Queue overflow policy for an input.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema, Default)]
#[serde(rename_all = "snake_case")]
pub enum QueuePolicy {
    /// Drop the oldest queued message when the queue is full (default).
    #[default]
    DropOldest,
    /// Buffer up to 10x `queue_size` without dropping. Drops with ERROR log at hard cap.
    Backpressure,
}

impl QueuePolicy {
    /// Returns the effective capacity for a given configured queue size.
    ///
    /// - `DropOldest`: returns `queue_size` as-is.
    /// - `Backpressure`: returns `10 * queue_size` (min 100) as a hard safety cap.
    pub fn effective_cap(&self, queue_size: usize) -> usize {
        match self {
            Self::DropOldest => queue_size,
            Self::Backpressure => queue_size.saturating_mul(10).max(100),
        }
    }
}

/// Contains the input and output configuration of the node.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct NodeRunConfig {
    /// Inputs for the nodes as a map from input ID to `node_id/output_id`.
    ///
    /// e.g.
    ///
    /// inputs:
    ///
    ///   example_input: example_node/example_output1
    ///
    #[serde(default)]
    pub inputs: BTreeMap<DataId, Input>,
    /// List of output IDs.
    ///
    /// e.g.
    ///
    /// outputs:
    ///
    ///  - output_1
    ///
    ///  - output_2
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,
    /// Optional type annotations for outputs
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub output_types: BTreeMap<DataId, String>,
    /// Per-output framing overrides (default: Raw for all).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub output_framing: BTreeMap<DataId, descriptor::OutputFraming>,
    /// Optional type annotations for inputs
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub input_types: BTreeMap<DataId, String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(from = "InputDef", into = "InputDef")]
pub struct Input {
    pub mapping: InputMapping,
    pub queue_size: Option<usize>,
    pub input_timeout: Option<f64>,
    pub queue_policy: Option<QueuePolicy>,
}

impl PartialEq for Input {
    fn eq(&self, other: &Self) -> bool {
        self.mapping == other.mapping
            && self.queue_size == other.queue_size
            && self.input_timeout.map(f64::to_bits) == other.input_timeout.map(f64::to_bits)
            && self.queue_policy == other.queue_policy
    }
}

impl Eq for Input {}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum InputDef {
    MappingOnly(InputMapping),
    WithOptions {
        source: InputMapping,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        queue_size: Option<usize>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        input_timeout: Option<f64>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        queue_policy: Option<QueuePolicy>,
    },
}

impl PartialEq for InputDef {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::MappingOnly(a), Self::MappingOnly(b)) => a == b,
            (
                Self::WithOptions {
                    source: s1,
                    queue_size: q1,
                    input_timeout: t1,
                    queue_policy: p1,
                },
                Self::WithOptions {
                    source: s2,
                    queue_size: q2,
                    input_timeout: t2,
                    queue_policy: p2,
                },
            ) => s1 == s2 && q1 == q2 && t1.map(f64::to_bits) == t2.map(f64::to_bits) && p1 == p2,
            _ => false,
        }
    }
}

impl Eq for InputDef {}

impl From<Input> for InputDef {
    fn from(input: Input) -> Self {
        if input.queue_size.is_none()
            && input.input_timeout.is_none()
            && input.queue_policy.is_none()
        {
            Self::MappingOnly(input.mapping)
        } else {
            Self::WithOptions {
                source: input.mapping,
                queue_size: input.queue_size,
                input_timeout: input.input_timeout,
                queue_policy: input.queue_policy,
            }
        }
    }
}

impl From<InputDef> for Input {
    fn from(value: InputDef) -> Self {
        match value {
            InputDef::MappingOnly(mapping) => Self {
                mapping,
                queue_size: None,
                input_timeout: None,
                queue_policy: None,
            },
            InputDef::WithOptions {
                source,
                queue_size,
                input_timeout,
                queue_policy,
            } => Self {
                mapping: source,
                queue_size,
                input_timeout,
                queue_policy,
            },
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, JsonSchema)]
pub enum InputMapping {
    Timer {
        interval: Duration,
    },
    /// Subscribe to log messages from all (or filtered) nodes in the dataflow.
    ///
    /// Syntax: `adora/logs`, `adora/logs/{level}`, `adora/logs/{level}/{node_id}`
    Logs(LogSubscriptionFilter),
    User(UserInputMapping),
}

impl InputMapping {
    pub fn source(&self) -> &NodeId {
        static ADORA_NODE_ID: OnceLock<NodeId> = OnceLock::new();

        match self {
            InputMapping::User(mapping) => &mapping.source,
            InputMapping::Timer { .. } | InputMapping::Logs(_) => {
                ADORA_NODE_ID.get_or_init(|| NodeId("adora".to_string()))
            }
        }
    }
}

impl fmt::Display for InputMapping {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            InputMapping::Timer { interval } => {
                let duration = format_duration(*interval);
                write!(f, "adora/timer/{duration}")
            }
            InputMapping::Logs(filter) => {
                write!(f, "adora/logs")?;
                if let Some(level) = &filter.min_level {
                    write!(f, "/{}", format_log_level(level))?;
                    if let Some(node) = &filter.node_filter {
                        write!(f, "/{node}")?;
                    }
                }
                Ok(())
            }
            InputMapping::User(mapping) => {
                write!(f, "{}/{}", mapping.source, mapping.output)
            }
        }
    }
}

impl FromStr for InputMapping {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (source, output) = s
            .split_once('/')
            .ok_or("input must start with `<source>/`")?;

        let mapping = match source {
            "adora" => match output.split_once('/') {
                Some(("timer", output)) => {
                    let (unit, value) = output.split_once('/').ok_or(
                        "timer input must specify unit and value (e.g. `secs/5` or `millis/100`)",
                    )?;
                    let interval = match unit {
                        "secs" => {
                            let value = value
                                .parse()
                                .map_err(|_| format!("secs must be an integer (got `{value}`)"))?;
                            Duration::from_secs(value)
                        }
                        "millis" => {
                            let value = value.parse().map_err(|_| {
                                format!("millis must be an integer (got `{value}`)")
                            })?;
                            Duration::from_millis(value)
                        }
                        other => {
                            return Err(format!(
                                "timer unit must be either secs or millis (got `{other}`"
                            ));
                        }
                    };
                    Self::Timer { interval }
                }
                Some(("logs", rest)) => {
                    // adora/logs/{level} or adora/logs/{level}/{node_id}
                    let (level_str, node_filter) = match rest.split_once('/') {
                        Some((level, node)) => (Some(level), Some(NodeId(node.to_owned()))),
                        None => {
                            if rest.is_empty() {
                                (None, None)
                            } else {
                                (Some(rest), None)
                            }
                        }
                    };
                    let min_level = level_str.map(parse_log_level_str).transpose()?;
                    Self::Logs(LogSubscriptionFilter {
                        min_level,
                        node_filter,
                    })
                }
                Some((other, _)) => {
                    return Err(format!("unknown adora input `{other}`"));
                }
                // "adora/logs" with no sub-path
                None if output == "logs" => Self::Logs(LogSubscriptionFilter {
                    min_level: None,
                    node_filter: None,
                }),
                None => return Err("adora input has invalid format".into()),
            },
            _ => Self::User(UserInputMapping {
                source: source.to_owned().into(),
                output: output.to_owned().into(),
            }),
        };

        Ok(mapping)
    }
}

fn parse_log_level_str(s: &str) -> Result<crate::common::LogLevelOrStdout, String> {
    use crate::common::{LogLevel, LogLevelOrStdout};
    match s.to_lowercase().as_str() {
        "stdout" => Ok(LogLevelOrStdout::Stdout),
        "error" => Ok(LogLevelOrStdout::LogLevel(LogLevel::Error)),
        "warn" => Ok(LogLevelOrStdout::LogLevel(LogLevel::Warn)),
        "info" => Ok(LogLevelOrStdout::LogLevel(LogLevel::Info)),
        "debug" => Ok(LogLevelOrStdout::LogLevel(LogLevel::Debug)),
        "trace" => Ok(LogLevelOrStdout::LogLevel(LogLevel::Trace)),
        other => Err(format!(
            "unknown log level `{other}` (expected: stdout, error, warn, info, debug, trace)"
        )),
    }
}

fn format_log_level(level: &crate::common::LogLevelOrStdout) -> &'static str {
    use crate::common::{LogLevel, LogLevelOrStdout};
    match level {
        LogLevelOrStdout::Stdout => "stdout",
        LogLevelOrStdout::LogLevel(l) => match *l {
            LogLevel::Error => "error",
            LogLevel::Warn => "warn",
            LogLevel::Info => "info",
            LogLevel::Debug => "debug",
            LogLevel::Trace => "trace",
        },
    }
}

pub struct FormattedDuration(pub Duration);

impl fmt::Display for FormattedDuration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0.subsec_millis() == 0 {
            write!(f, "secs/{}", self.0.as_secs())
        } else {
            write!(f, "millis/{}", self.0.as_millis())
        }
    }
}

pub fn format_duration(interval: Duration) -> FormattedDuration {
    FormattedDuration(interval)
}

impl Serialize for InputMapping {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for InputMapping {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let string = String::deserialize(deserializer)?;
        string.parse().map_err(serde::de::Error::custom)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, JsonSchema)]
pub struct UserInputMapping {
    pub source: NodeId,
    pub output: DataId,
}

#[derive(Debug, Default, Serialize, Deserialize, JsonSchema, Clone)]
#[serde(deny_unknown_fields, rename_all = "lowercase")]
pub struct CommunicationConfig {
    // see https://github.com/dtolnay/serde-yaml/issues/298
    #[serde(
        default,
        with = "serde_yaml::with::singleton_map",
        rename = "_unstable_local"
    )]
    #[schemars(with = "String")]
    pub local: LocalCommunicationConfig,
    #[serde(
        default,
        with = "serde_yaml::with::singleton_map",
        rename = "_unstable_remote"
    )]
    #[schemars(with = "String")]
    pub remote: RemoteCommunicationConfig,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize, Default)]
pub enum LocalCommunicationConfig {
    #[default]
    Tcp,
    Shmem,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields, rename_all = "lowercase")]
#[derive(Default)]
pub enum RemoteCommunicationConfig {
    #[default]
    Tcp,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_input_without_queue_policy() {
        let yaml = "source: node_a/output_1\nqueue_size: 5\n";
        let input: Input = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(input.queue_size, Some(5));
        assert_eq!(input.queue_policy, None);
    }

    #[test]
    fn parse_input_with_drop_oldest_policy() {
        let yaml = "source: node_a/output_1\nqueue_size: 5\nqueue_policy: drop_oldest\n";
        let input: Input = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(input.queue_policy, Some(QueuePolicy::DropOldest));
    }

    #[test]
    fn parse_input_with_backpressure_policy() {
        let yaml = "source: node_a/output_1\nqueue_size: 10\nqueue_policy: backpressure\n";
        let input: Input = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(input.queue_policy, Some(QueuePolicy::Backpressure));
    }

    #[test]
    fn parse_short_form_input_has_no_policy() {
        let yaml = "node_a/output_1";
        let input: Input = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(input.queue_policy, None);
        assert_eq!(input.queue_size, None);
    }

    #[test]
    fn roundtrip_input_with_policy() {
        let input = Input {
            mapping: "node_a/output_1".parse().unwrap(),
            queue_size: Some(3),
            input_timeout: None,
            queue_policy: Some(QueuePolicy::Backpressure),
        };
        let yaml = serde_yaml::to_string(&input).unwrap();
        let parsed: Input = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(input, parsed);
    }

    #[test]
    fn roundtrip_input_without_policy_uses_short_form() {
        let input = Input {
            mapping: "node_a/output_1".parse().unwrap(),
            queue_size: None,
            input_timeout: None,
            queue_policy: None,
        };
        let yaml = serde_yaml::to_string(&input).unwrap();
        // Short form should not contain "source:" key
        assert!(!yaml.contains("source:"));
        let parsed: Input = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(input, parsed);
    }

    #[test]
    fn queue_policy_default_is_drop_oldest() {
        assert_eq!(QueuePolicy::default(), QueuePolicy::DropOldest);
    }

    #[test]
    fn parse_logs_all() {
        let mapping: InputMapping = "adora/logs".parse().unwrap();
        assert!(matches!(
            mapping,
            InputMapping::Logs(LogSubscriptionFilter {
                min_level: None,
                node_filter: None,
            })
        ));
    }

    #[test]
    fn parse_logs_with_level() {
        use crate::common::{LogLevel, LogLevelOrStdout};
        let mapping: InputMapping = "adora/logs/info".parse().unwrap();
        match mapping {
            InputMapping::Logs(f) => {
                assert_eq!(
                    f.min_level,
                    Some(LogLevelOrStdout::LogLevel(LogLevel::Info))
                );
                assert_eq!(f.node_filter, None);
            }
            _ => panic!("expected Logs variant"),
        }
    }

    #[test]
    fn parse_logs_with_level_and_node() {
        use crate::common::{LogLevel, LogLevelOrStdout};
        let mapping: InputMapping = "adora/logs/error/sensor".parse().unwrap();
        match mapping {
            InputMapping::Logs(f) => {
                assert_eq!(
                    f.min_level,
                    Some(LogLevelOrStdout::LogLevel(LogLevel::Error))
                );
                assert_eq!(f.node_filter, Some(NodeId("sensor".to_string())));
            }
            _ => panic!("expected Logs variant"),
        }
    }

    #[test]
    fn parse_logs_invalid_level() {
        let result: Result<InputMapping, _> = "adora/logs/banana".parse();
        assert!(result.is_err());
    }

    #[test]
    fn display_roundtrip_logs_all() {
        let mapping: InputMapping = "adora/logs".parse().unwrap();
        assert_eq!(mapping.to_string(), "adora/logs");
    }

    #[test]
    fn display_roundtrip_logs_with_level() {
        let mapping: InputMapping = "adora/logs/warn".parse().unwrap();
        assert_eq!(mapping.to_string(), "adora/logs/warn");
    }

    #[test]
    fn display_roundtrip_logs_with_level_and_node() {
        let mapping: InputMapping = "adora/logs/debug/camera".parse().unwrap();
        assert_eq!(mapping.to_string(), "adora/logs/debug/camera");
    }

    #[test]
    fn logs_source_is_adora() {
        let mapping: InputMapping = "adora/logs".parse().unwrap();
        assert_eq!(mapping.source().to_string(), "adora");
    }

    #[test]
    fn parse_logs_trailing_slash() {
        let mapping: InputMapping = "adora/logs/".parse().unwrap();
        assert!(matches!(
            mapping,
            InputMapping::Logs(LogSubscriptionFilter {
                min_level: None,
                node_filter: None,
            })
        ));
    }
}
