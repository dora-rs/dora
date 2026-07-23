use core::fmt;
use std::{
    collections::{BTreeMap, BTreeSet},
    str::FromStr,
    time::Duration,
};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

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
    /// - `DropOldest`: returns `queue_size`, but never less than 1.
    /// - `Backpressure`: returns `10 * queue_size` (min 100) as a hard safety cap.
    ///
    /// A `DropOldest` cap of 0 would drop 100% of the input's events — the
    /// runtime operator channel sets the just-queued event to `None` on every
    /// `add_event`, so the operator never receives a single message and the
    /// dataflow silently hangs. Clamp `queue_size: 0` to 1 (latest-only)
    /// instead of turning the input into a dead port.
    pub fn effective_cap(&self, queue_size: usize) -> usize {
        match self {
            Self::DropOldest => queue_size.max(1),
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
                        "micros" => {
                            let value = value.parse().map_err(|_| {
                                format!("micros must be an integer (got `{value}`)")
                            })?;
                            Duration::from_micros(value)
                        }
                        "nanos" => {
                            let value = value
                                .parse()
                                .map_err(|_| format!("nanos must be an integer (got `{value}`)"))?;
                            Duration::from_nanos(value)
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
                            // A very large hz makes `1/hz` round below 1ns, so
                            // `try_from_secs_f64` returns `Ok(Duration::ZERO)`;
                            // that zero interval is caught by the shared guard
                            // after this `match`, together with `<unit>/0`.
                            Duration::try_from_secs_f64(1.0 / hz).map_err(|e| {
                                format!("hz `{value}` produces an out-of-range interval: {e}")
                            })?
                        }
                        other => {
                            return Err(format!(
                                "timer unit must be `secs`, `millis`, `micros`, `nanos`, or `hz` (got `{other}`)"
                            ));
                        }
                    };
                    // A zero-length interval is invalid for every unit: the timer
                    // task builds `tokio::time::interval(interval)`, which panics
                    // (`period` must be non-zero). Reject it at parse time for all
                    // units -- `secs/0`, `millis/0`, `micros/0`, `nanos/0`, and a
                    // huge `hz` that rounds `1/hz` below 1ns -- so a bad descriptor
                    // fails at load with a clear message instead of panicking a
                    // daemon task later.
                    if interval.is_zero() {
                        return Err(format!(
                            "timer interval must be non-zero (`{unit}/{value}` \
                             produces a zero-length interval)"
                        ));
                    }
                    Self::Timer { interval }
                }
                Some(("logs", rest)) => {
                    // dora/logs/{level} or dora/logs/{level}/{node_id}
                    let (level_str, node_filter) = match rest.split_once('/') {
                        Some((level, node)) => {
                            // Validate the node segment instead of constructing a
                            // `NodeId` directly: `rest.split_once('/')` keeps every
                            // slash after the first inside `node`, so an input such
                            // as `dora/logs/info/a/b` would otherwise build a NodeId
                            // containing `/` -- a value `validate_node_id` forbids and
                            // that no real node id can ever equal, silently producing
                            // a filter that never matches.
                            let node_id = node.parse::<NodeId>().map_err(|e| e.to_string())?;
                            (Some(level), Some(node_id))
                        }
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
            _ => {
                // Validate the source/output segments instead of using the
                // panicking `From<String>` impls (`.into()`): an input mapping
                // string comes straight from user-authored descriptor YAML, so an
                // invalid identifier (e.g. containing a space) must surface as a
                // clean deserialization error rather than panicking the parser.
                let source = source.parse::<NodeId>().map_err(|e| e.to_string())?;
                let output = output.parse::<DataId>().map_err(|e| e.to_string())?;
                Self::User(UserInputMapping { source, output })
            }
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
        // Emit the coarsest unit that represents the interval exactly, so the
        // value round-trips through `FromStr` without loss. Using `millis` for
        // sub-millisecond intervals (e.g. `dora/timer/hz/3000` ~= 333µs) would
        // truncate to `millis/0`, i.e. a 0ms busy-loop interval (dora-rs#2031).
        let nanos = self.0.as_nanos();
        if nanos.is_multiple_of(1_000_000_000) {
            write!(f, "secs/{}", self.0.as_secs())
        } else if nanos.is_multiple_of(1_000_000) {
            write!(f, "millis/{}", self.0.as_millis())
        } else if nanos.is_multiple_of(1_000) {
            write!(f, "micros/{}", self.0.as_micros())
        } else {
            write!(f, "nanos/{nanos}")
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
///
/// Negative, non-finite, and overflowing values are rejected:
///
/// ```
/// use dora_message::config::ByteSize;
///
/// assert_eq!("64MB".parse::<ByteSize>().unwrap().as_bytes(), 64 * 1024 * 1024);
/// assert_eq!("1.5 KB".parse::<ByteSize>().unwrap().as_bytes(), 1536);
/// assert!("-1KB".parse::<ByteSize>().is_err());
/// assert!("1TB".parse::<ByteSize>().is_err());
/// ```
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

        let multiplier: usize = match unit_part.to_uppercase().as_str() {
            "B" => 1,
            "KB" | "K" => 1024,
            "MB" | "M" => 1024 * 1024,
            "GB" | "G" => 1024 * 1024 * 1024,
            other => return Err(format!("unknown byte size unit: `{other}`")),
        };

        // Use integer parse when possible to avoid f64 rounding above 2^53.
        if let Ok(num) = num_part.parse::<usize>() {
            return num
                .checked_mul(multiplier)
                .map(ByteSize)
                .ok_or_else(|| format!("byte size `{s}` is too large"));
        }

        let num: f64 = num_part
            .parse()
            .map_err(|_| format!("invalid number in byte size: `{num_part}`"))?;

        // Casting a negative or non-finite f64 to usize saturates (negatives
        // and NaN to 0, +inf to usize::MAX) instead of erroring, so reject
        // them up front.
        if !num.is_finite() || num < 0.0 {
            return Err(format!(
                "byte size must be a non-negative, finite number: `{s}`"
            ));
        }
        let bytes = num * multiplier as f64;
        // `usize::MAX as f64` rounds up to 2^64, and no f64 values exist
        // between usize::MAX and 2^64, so `>=` rejects exactly the results
        // that exceed usize::MAX.
        if bytes >= usize::MAX as f64 {
            return Err(format!("byte size `{s}` is too large"));
        }
        Ok(ByteSize(bytes as usize))
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
                usize::try_from(v)
                    .map(ByteSize)
                    .map_err(|_| E::custom(format!("byte size `{v}` is too large")))
            }

            fn visit_i64<E: de::Error>(self, v: i64) -> Result<ByteSize, E> {
                if v < 0 {
                    return Err(E::custom("byte size cannot be negative"));
                }
                usize::try_from(v)
                    .map(ByteSize)
                    .map_err(|_| E::custom(format!("byte size `{v}` is too large")))
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
    fn drop_oldest_cap_is_never_zero() {
        // A `queue_size: 0` DropOldest input must keep at least the latest
        // message; a cap of 0 would starve the operator entirely.
        assert_eq!(QueuePolicy::DropOldest.effective_cap(0), 1);
        // Non-zero sizes are unchanged.
        assert_eq!(QueuePolicy::DropOldest.effective_cap(1), 1);
        assert_eq!(QueuePolicy::DropOldest.effective_cap(5), 5);
    }

    #[test]
    fn backpressure_cap_has_floor() {
        assert_eq!(QueuePolicy::Backpressure.effective_cap(0), 100);
        assert_eq!(QueuePolicy::Backpressure.effective_cap(5), 100);
        assert_eq!(QueuePolicy::Backpressure.effective_cap(20), 200);
    }

    #[test]
    fn parse_user_mapping_rejects_invalid_ids() {
        // A source node id with a space is not a valid `NodeId` and must be
        // rejected with a clean error rather than panicking the parser via the
        // `From<String>` impl.
        let result: Result<InputMapping, _> = "bad node/output".parse();
        assert!(result.is_err(), "invalid source node id must be rejected");

        // An output data id with an invalid character must likewise be rejected.
        let result: Result<InputMapping, _> = "node_a/bad output".parse();
        assert!(result.is_err(), "invalid output data id must be rejected");

        // A valid mapping still parses successfully.
        let mapping: InputMapping = "node_a/output_1".parse().unwrap();
        assert!(matches!(mapping, InputMapping::User(_)));
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
    fn parse_timer_hz_rejects_overflow() {
        // A pathologically small hz makes 1/hz overflow Duration; this must
        // return an error, not panic (was `Duration::from_secs_f64`, which
        // panics on out-of-range input).
        let err = "dora/timer/hz/0.00000000000000000001"
            .parse::<InputMapping>()
            .unwrap_err();
        assert!(err.contains("hz"), "error should mention hz: {err}");
    }

    #[test]
    fn parse_timer_rejects_zero_interval_for_every_unit() {
        // A zero-length interval panics `tokio::time::interval` in the timer
        // task (`period` must be non-zero), so every unit that can express it
        // must be rejected at parse time -- not just `hz`. `<unit>/0` is the
        // obvious case; a huge `hz` reaches the same zero interval because
        // `1/hz` rounds below 1ns.
        let cases = [
            "dora/timer/secs/0",
            "dora/timer/millis/0",
            "dora/timer/micros/0",
            "dora/timer/nanos/0",
            "dora/timer/hz/1000000000000",
        ];
        for case in cases {
            let err = case.parse::<InputMapping>().unwrap_err();
            assert!(
                err.contains("non-zero"),
                "`{case}` should be rejected as a zero-length interval, got: {err}"
            );
        }
        // Sanity check: the huge-hz conversion really does round to zero.
        assert_eq!(
            Duration::try_from_secs_f64(1.0 / 1_000_000_000_000.0),
            Ok(Duration::ZERO)
        );
    }

    /// Regression test for dora-rs#2031: the `Display` impl previously emitted
    /// `millis/0` (or `secs/0`) for any sub-millisecond interval, so a valid
    /// high-rate timer like `dora/timer/hz/3000` (~=333µs) round-tripped into a
    /// 0ms busy-loop interval. Sub-ms intervals must survive Display -> parse.
    #[test]
    fn timer_subms_interval_roundtrips() {
        let cases = [
            "dora/timer/hz/3000",  // ~= 333_333 ns, not a whole µs
            "dora/timer/micros/1", // 1µs
            "dora/timer/nanos/1",  // 1ns
            "dora/timer/nanos/500",
            "dora/timer/micros/250",
        ];
        for case in cases {
            let mapping: InputMapping = case.parse().unwrap();
            let InputMapping::Timer { interval } = mapping else {
                panic!("expected Timer for `{case}`, got {mapping:?}");
            };
            assert!(!interval.is_zero(), "`{case}` parsed to a zero interval");
            let rendered = mapping.to_string();
            let reparsed: InputMapping = rendered.parse().unwrap();
            assert_eq!(
                mapping, reparsed,
                "`{case}` did not round-trip (rendered as `{rendered}`)"
            );
        }
    }

    #[test]
    fn timer_display_uses_coarsest_exact_unit() {
        let render = |d: Duration| format_duration(d).to_string();
        assert_eq!(render(Duration::from_secs(5)), "secs/5");
        assert_eq!(render(Duration::from_millis(100)), "millis/100");
        assert_eq!(render(Duration::from_micros(250)), "micros/250");
        assert_eq!(render(Duration::from_nanos(500)), "nanos/500");
        // 1/3000 Hz = 333_333 ns (not a whole microsecond) -> nanos.
        assert_eq!(render(Duration::from_nanos(333_333)), "nanos/333333");
    }

    #[test]
    fn parse_timer_micros_and_nanos() {
        let micros: InputMapping = "dora/timer/micros/250".parse().unwrap();
        assert_eq!(
            micros,
            InputMapping::Timer {
                interval: Duration::from_micros(250)
            }
        );
        let nanos: InputMapping = "dora/timer/nanos/500".parse().unwrap();
        assert_eq!(
            nanos,
            InputMapping::Timer {
                interval: Duration::from_nanos(500)
            }
        );
    }

    #[test]
    fn timer_whole_second_and_milli_still_roundtrip() {
        for case in ["dora/timer/secs/2", "dora/timer/millis/100"] {
            let mapping: InputMapping = case.parse().unwrap();
            let reparsed: InputMapping = mapping.to_string().parse().unwrap();
            assert_eq!(mapping, reparsed);
        }
        // Whole seconds/millis still render with their original unit.
        assert_eq!(
            "dora/timer/secs/2"
                .parse::<InputMapping>()
                .unwrap()
                .to_string(),
            "dora/timer/secs/2"
        );
        assert_eq!(
            "dora/timer/millis/100"
                .parse::<InputMapping>()
                .unwrap()
                .to_string(),
            "dora/timer/millis/100"
        );
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
    fn parse_logs_rejects_invalid_node_filter() {
        // A node segment with an extra `/` (kept by `split_once`) is not a valid
        // NodeId and must be rejected rather than silently yielding a filter that
        // can never match a real (validated) node id.
        let result: Result<InputMapping, _> = "dora/logs/info/a/b".parse();
        assert!(result.is_err(), "node filter `a/b` must be rejected");

        // A node segment containing a space is likewise invalid.
        let result: Result<InputMapping, _> = "dora/logs/info/bad node".parse();
        assert!(result.is_err(), "node filter `bad node` must be rejected");
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
    fn byte_size_rejects_negative() {
        // Negative f64 → usize casts saturate to 0, so without an explicit
        // check `-1KB` silently parsed as a zero-byte pool size.
        assert!("-1KB".parse::<ByteSize>().is_err());
        assert!("-0.5MB".parse::<ByteSize>().is_err());
        assert!("-1".parse::<ByteSize>().is_err());
        // The integer deserialize path already rejected negatives; the string
        // path must agree.
        assert!(serde_yaml::from_str::<ByteSize>("-1").is_err());
        assert!(serde_yaml::from_str::<ByteSize>(r#""-1KB""#).is_err());
    }

    #[test]
    fn byte_size_rejects_overflow() {
        // f64 path: finite but larger than usize::MAX must error, not
        // saturate silently.
        assert!("99999999999999999999999GB".parse::<ByteSize>().is_err());
        // Integer path: checked_mul must catch the overflow.
        assert!(format!("{}KB", usize::MAX).parse::<ByteSize>().is_err());
        // Exactly 2^64 (= 2^54 * 1024) on the float path: `usize::MAX as f64`
        // rounds up to 2^64, so a `>` comparison would let this saturate to
        // usize::MAX silently.
        assert!("18014398509481984.0KB".parse::<ByteSize>().is_err());
    }

    #[test]
    fn byte_size_integer_values_parse_exactly() {
        // 2^53 + 1 is not representable as f64; the integer fast path must
        // preserve it exactly (mirrors dora-core's parse_byte_size).
        assert_eq!(
            "9007199254740993B".parse::<ByteSize>().unwrap(),
            ByteSize(9007199254740993)
        );
    }

    #[test]
    fn byte_size_float_path_still_works() {
        assert_eq!("1.5KB".parse::<ByteSize>().unwrap(), ByteSize(1536));
        assert_eq!("0.5MB".parse::<ByteSize>().unwrap(), ByteSize(512 * 1024));
        assert!("x.5KB".parse::<ByteSize>().is_err());
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
