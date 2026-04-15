use core::fmt;
use std::{
    collections::{BTreeMap, BTreeSet},
    str::FromStr,
    time::Duration,
};

use once_cell::sync::OnceCell;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

pub use crate::id::{DataId, NodeId, OperatorId};

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

    /// Size of the zenoh shared memory pool for zero-copy output publishing.
    ///
    /// Accepts an integer (raw bytes) or a string with a unit suffix
    /// (`KB`, `MB`, `GB`, case-insensitive). Defaults to 8 MB if not set.
    ///
    /// e.g.
    ///
    /// shared_memory_pool_size: 128MB
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shared_memory_pool_size: Option<ByteSize>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(from = "InputDef", into = "InputDef")]
pub struct Input {
    pub mapping: InputMapping,
    pub queue_size: Option<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum InputDef {
    MappingOnly(InputMapping),
    WithOptions {
        source: InputMapping,
        queue_size: Option<usize>,
    },
}

impl From<Input> for InputDef {
    fn from(input: Input) -> Self {
        match input {
            Input {
                mapping,
                queue_size: None,
            } => Self::MappingOnly(mapping),
            Input {
                mapping,
                queue_size,
            } => Self::WithOptions {
                source: mapping,
                queue_size,
            },
        }
    }
}

impl From<InputDef> for Input {
    fn from(value: InputDef) -> Self {
        match value {
            InputDef::MappingOnly(mapping) => Self {
                mapping,
                queue_size: None,
            },
            InputDef::WithOptions { source, queue_size } => Self {
                mapping: source,
                queue_size,
            },
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, JsonSchema)]
pub enum InputMapping {
    Timer { interval: Duration },
    User(UserInputMapping),
}

impl InputMapping {
    pub fn source(&self) -> &NodeId {
        static DORA_NODE_ID: OnceCell<NodeId> = OnceCell::new();

        match self {
            InputMapping::User(mapping) => &mapping.source,
            InputMapping::Timer { .. } => DORA_NODE_ID.get_or_init(|| NodeId("dora".to_string())),
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
                Some((other, _)) => {
                    return Err(format!("unknown dora input `{other}`"));
                }
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
        rename = "_unstable_remote"
    )]
    #[schemars(with = "String")]
    pub remote: RemoteCommunicationConfig,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields, rename_all = "lowercase")]
pub enum RemoteCommunicationConfig {
    Tcp,
}

impl Default for RemoteCommunicationConfig {
    fn default() -> Self {
        Self::Tcp
    }
}

/// A byte size that can be deserialized from either an integer (raw bytes) or a
/// string with a unit suffix (`KB`, `MB`, `GB`, case-insensitive).
///
/// Examples: `67108864`, `"64MB"`, `"1 GB"`, `"512kb"`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ByteSize(pub usize);

impl ByteSize {
    pub fn as_bytes(&self) -> usize {
        self.0
    }
}

impl FromStr for ByteSize {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim();
        // Find where the numeric part ends and the unit begins
        let (num_part, unit_part) = match s.find(|c: char| c.is_alphabetic()) {
            Some(pos) => (s[..pos].trim(), s[pos..].trim()),
            None => {
                // No unit — parse as raw bytes
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
        } else if bytes % (1024 * 1024 * 1024) == 0 {
            write!(f, "{}GB", bytes / (1024 * 1024 * 1024))
        } else if bytes % (1024 * 1024) == 0 {
            write!(f, "{}MB", bytes / (1024 * 1024))
        } else if bytes % 1024 == 0 {
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
        // Serialize as integer (bytes) for round-trip fidelity
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
