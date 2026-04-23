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

/// Filter for the `dora/logs` virtual input.
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

    /// Size of the zenoh shared memory pool for zero-copy output publishing.
    ///
    /// Accepts an integer (raw bytes) or a string with a unit suffix
    /// (`KB`, `MB`, `GB`, case-insensitive).
    ///
    /// e.g.
    ///
    /// shared_memory_pool_size: 128MB
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shared_memory_pool_size: Option<ByteSize>,
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
    /// Syntax: `dora/logs`, `dora/logs/{level}`, `dora/logs/{level}/{node_id}`
    Logs(LogSubscriptionFilter),
    User(UserInputMapping),
}

impl InputMapping {
    pub fn source(&self) -> &NodeId {
        static DORA_NODE_ID: OnceLock<NodeId> = OnceLock::new();

        match self {
            InputMapping::User(mapping) => &mapping.source,
            InputMapping::Timer { .. } | InputMapping::Logs(_) => {
                DORA_NODE_ID.get_or_init(|| NodeId("dora".to_string()))
            }
        }
    }
}

impl fmt::Display for InputMapping {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            InputMapping::Timer { interval } => {
                let duration = format_duration(*interval);
                write!(f, "dora/timer/{duration}")
            }
            InputMapping::Logs(filter) => {
                write!(f, "dora/logs")?;
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
            "dora" => match output.split_once('/') {
                Some(("timer", output)) => {
                    let (unit, value) = output.split_once('/').ok_or(
                        "timer input must specify unit and value (e.g. `secs/5`, `millis/100`, or `hz/30`)",
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
                        "hz" => {
                            let hz: f64 = value.parse().map_err(|_| {
                                format!("hz must be a positive number (got `{value}`)")
                            })?;
                            if !hz.is_finite() || hz <= 0.0 {
                                return Err(format!(
                                    "hz must be a positive finite number (got `{value}`)"
                                ));
                            }
                            Duration::from_secs_f64(1.0 / hz)
                        }
                        other => {
                            return Err(format!(
                                "timer unit must be `secs`, `millis`, or `hz` (got `{other}`)"
                            ));
                        }
                    };
                    Self::Timer { interval }
                }
                Some(("logs", rest)) => {
                    // dora/logs/{level} or dora/logs/{level}/{node_id}
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
                    return Err(format!("unknown dora input `{other}`"));
                }
                // "dora/logs" with no sub-path
                None if output == "logs" => Self::Logs(LogSubscriptionFilter {
                    min_level: None,
                    node_filter: None,
                }),
                None => return Err("dora input has invalid format".into()),
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
#[serde(rename_all = "lowercase")]
pub enum LocalCommunicationConfig {
    #[default]
    Tcp,
}

impl fmt::Display for LocalCommunicationConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Tcp => write!(f, "tcp"),
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields, rename_all = "lowercase")]
#[derive(Default)]
pub enum RemoteCommunicationConfig {
    #[default]
    Tcp,
}

/// A byte size that can be deserialized from either an integer (raw bytes) or a
/// string with a unit suffix (`KB`, `MB`, `GB`, case-insensitive).
///
/// Examples: `67108864`, `"64MB"`, `"1 GB"`, `"512kb"`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ByteSize(pub usize);

impl ByteSize {
    /// Returns the size in raw bytes.
    pub fn as_bytes(&self) -> usize {
        self.0
    }
}

impl FromStr for ByteSize {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim();
        let (num_part, unit_part) = match s.find(|c: char| c.is_alphabetic()) {
            Some(pos) => (s[..pos].trim(), s[pos..].trim()),
            None => {
                let bytes: usize = s.parse().map_err(|_| format!("invalid byte size: `{s}`"))?;
                return Ok(ByteSize(bytes));
            }
        };

        let num: f64 = num_part
            .parse()
            .map_err(|_| format!("invalid number in byte size: `{num_part}`"))?;

        let multiplier: usize = match unit_part.to_uppercase().as_str() {
            "B" => 1,
            "KB" | "K" => 1024,
            "MB" | "M" => 1024 * 1024,
            "GB" | "G" => 1024 * 1024 * 1024,
            other => return Err(format!("unknown byte size unit: `{other}`")),
        };

        Ok(ByteSize((num * multiplier as f64) as usize))
    }
}

impl fmt::Display for ByteSize {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let bytes = self.0;
        if bytes == 0 {
            write!(f, "0B")
        } else if bytes.is_multiple_of(1024 * 1024 * 1024) {
            write!(f, "{}GB", bytes / (1024 * 1024 * 1024))
        } else if bytes.is_multiple_of(1024 * 1024) {
            write!(f, "{}MB", bytes / (1024 * 1024))
        } else if bytes.is_multiple_of(1024) {
            write!(f, "{}KB", bytes / 1024)
        } else {
            write!(f, "{bytes}")
        }
    }
}

impl Serialize for ByteSize {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.0.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for ByteSize {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use serde::de;

        struct ByteSizeVisitor;

        impl de::Visitor<'_> for ByteSizeVisitor {
            type Value = ByteSize;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("a byte size as integer or string (e.g. 67108864, \"64MB\")")
            }

            fn visit_u64<E: de::Error>(self, v: u64) -> Result<ByteSize, E> {
                Ok(ByteSize(v as usize))
            }

            fn visit_i64<E: de::Error>(self, v: i64) -> Result<ByteSize, E> {
                if v < 0 {
                    return Err(E::custom("byte size cannot be negative"));
                }
                Ok(ByteSize(v as usize))
            }

            fn visit_str<E: de::Error>(self, v: &str) -> Result<ByteSize, E> {
                v.parse().map_err(E::custom)
            }
        }

        deserializer.deserialize_any(ByteSizeVisitor)
    }
}

impl JsonSchema for ByteSize {
    fn schema_name() -> std::borrow::Cow<'static, str> {
        "ByteSize".into()
    }

    fn json_schema(_gen: &mut schemars::SchemaGenerator) -> schemars::Schema {
        schemars::json_schema!({
            "anyOf": [
                { "type": "integer" },
                { "type": "string" }
            ],
            "description": "Byte size: integer (raw bytes) or string with unit (e.g. \"128MB\", \"1GB\")"
        })
    }
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

    /// Regression tests for dora-rs/adora#144: `dora/timer/hz/N` is
    /// documented across README, guide, schema, and real examples
    /// (`streaming-example`, `dynamic-agent-tools`) but the parser
    /// previously only accepted `secs` / `millis`.
    #[test]
    fn parse_timer_hz_integer() {
        let mapping: InputMapping = "dora/timer/hz/30".parse().unwrap();
        match mapping {
            InputMapping::Timer { interval } => {
                // 1 / 30 Hz ≈ 33.333 ms
                assert_eq!(interval, Duration::from_secs_f64(1.0 / 30.0));
            }
            other => panic!("expected Timer, got {other:?}"),
        }
    }

    #[test]
    fn parse_timer_hz_fractional() {
        // Used in examples/streaming-example/dataflow.yml
        let mapping: InputMapping = "dora/timer/hz/0.5".parse().unwrap();
        match mapping {
            InputMapping::Timer { interval } => {
                assert_eq!(interval, Duration::from_secs(2));
            }
            other => panic!("expected Timer, got {other:?}"),
        }
    }

    #[test]
    fn parse_timer_hz_rejects_zero() {
        let err = "dora/timer/hz/0".parse::<InputMapping>().unwrap_err();
        assert!(err.contains("hz"), "error should mention hz: {err}");
    }

    #[test]
    fn parse_timer_hz_rejects_negative() {
        let err = "dora/timer/hz/-1".parse::<InputMapping>().unwrap_err();
        assert!(err.contains("hz"), "error should mention hz: {err}");
    }

    #[test]
    fn parse_timer_hz_rejects_non_numeric() {
        let err = "dora/timer/hz/foo".parse::<InputMapping>().unwrap_err();
        assert!(err.contains("hz"), "error should mention hz: {err}");
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
        let mapping: InputMapping = "dora/logs".parse().unwrap();
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
        let mapping: InputMapping = "dora/logs/info".parse().unwrap();
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
        let mapping: InputMapping = "dora/logs/error/sensor".parse().unwrap();
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
        let result: Result<InputMapping, _> = "dora/logs/banana".parse();
        assert!(result.is_err());
    }

    #[test]
    fn display_roundtrip_logs_all() {
        let mapping: InputMapping = "dora/logs".parse().unwrap();
        assert_eq!(mapping.to_string(), "dora/logs");
    }

    #[test]
    fn display_roundtrip_logs_with_level() {
        let mapping: InputMapping = "dora/logs/warn".parse().unwrap();
        assert_eq!(mapping.to_string(), "dora/logs/warn");
    }

    #[test]
    fn display_roundtrip_logs_with_level_and_node() {
        let mapping: InputMapping = "dora/logs/debug/camera".parse().unwrap();
        assert_eq!(mapping.to_string(), "dora/logs/debug/camera");
    }

    #[test]
    fn logs_source_is_dora() {
        let mapping: InputMapping = "dora/logs".parse().unwrap();
        assert_eq!(mapping.source().to_string(), "dora");
    }

    #[test]
    fn parse_logs_trailing_slash() {
        let mapping: InputMapping = "dora/logs/".parse().unwrap();
        assert!(matches!(
            mapping,
            InputMapping::Logs(LogSubscriptionFilter {
                min_level: None,
                node_filter: None,
            })
        ));
    }

    #[test]
    fn communication_config_local_tcp_lowercase() {
        let yaml = "_unstable_local: tcp\n";
        let config: CommunicationConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.local, LocalCommunicationConfig::Tcp);
    }

    #[test]
    fn communication_config_remote_tcp_lowercase() {
        let yaml = "_unstable_remote: tcp\n";
        let config: CommunicationConfig = serde_yaml::from_str(yaml).unwrap();
        assert!(matches!(config.remote, RemoteCommunicationConfig::Tcp));
    }

    #[test]
    fn communication_config_unknown_local_variant_errors() {
        let yaml = "_unstable_local: unixdomain\n";
        assert!(serde_yaml::from_str::<CommunicationConfig>(yaml).is_err());
    }

    #[test]
    fn local_communication_config_display() {
        assert_eq!(LocalCommunicationConfig::Tcp.to_string(), "tcp");
    }

    #[test]
    fn communication_config_roundtrip_local() {
        let config = CommunicationConfig {
            local: LocalCommunicationConfig::Tcp,
            remote: RemoteCommunicationConfig::Tcp,
        };
        let yaml = serde_yaml::to_string(&config).unwrap();
        let parsed: CommunicationConfig = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(parsed.local, LocalCommunicationConfig::Tcp);
    }

    #[test]
    fn byte_size_parses_raw_bytes() {
        assert_eq!("1024".parse::<ByteSize>().unwrap(), ByteSize(1024));
        assert_eq!("0".parse::<ByteSize>().unwrap(), ByteSize(0));
    }

    #[test]
    fn byte_size_parses_units_case_insensitively() {
        assert_eq!("1KB".parse::<ByteSize>().unwrap(), ByteSize(1024));
        assert_eq!("1kb".parse::<ByteSize>().unwrap(), ByteSize(1024));
        assert_eq!("1MB".parse::<ByteSize>().unwrap(), ByteSize(1024 * 1024));
        assert_eq!(
            "1GB".parse::<ByteSize>().unwrap(),
            ByteSize(1024 * 1024 * 1024)
        );
        assert_eq!(
            "128 MB".parse::<ByteSize>().unwrap(),
            ByteSize(128 * 1024 * 1024)
        );
        assert_eq!("512B".parse::<ByteSize>().unwrap(), ByteSize(512));
    }

    #[test]
    fn byte_size_rejects_unknown_unit() {
        assert!("1TB".parse::<ByteSize>().is_err());
        assert!("abc".parse::<ByteSize>().is_err());
    }

    #[test]
    fn byte_size_deserializes_int_or_string() {
        let from_int: ByteSize = serde_yaml::from_str("67108864").unwrap();
        assert_eq!(from_int, ByteSize(64 * 1024 * 1024));

        let from_str: ByteSize = serde_yaml::from_str(r#""64MB""#).unwrap();
        assert_eq!(from_str, ByteSize(64 * 1024 * 1024));
    }

    #[test]
    fn byte_size_serializes_as_integer() {
        let yaml = serde_yaml::to_string(&ByteSize(1024)).unwrap();
        assert_eq!(yaml.trim(), "1024");
    }

    #[test]
    fn byte_size_display_uses_largest_exact_unit() {
        assert_eq!(ByteSize(1024).to_string(), "1KB");
        assert_eq!(ByteSize(1024 * 1024).to_string(), "1MB");
        assert_eq!(ByteSize(2 * 1024 * 1024 * 1024).to_string(), "2GB");
        assert_eq!(ByteSize(1500).to_string(), "1500");
    }

    #[test]
    fn node_run_config_parses_shared_memory_pool_size() {
        let yaml = "shared_memory_pool_size: 128MB\n";
        let config: NodeRunConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(
            config.shared_memory_pool_size,
            Some(ByteSize(128 * 1024 * 1024))
        );
    }

    #[test]
    fn node_run_config_shared_memory_pool_size_optional() {
        let yaml = "outputs:\n  - foo\n";
        let config: NodeRunConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.shared_memory_pool_size, None);
    }
}
