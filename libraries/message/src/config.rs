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
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(from = "InputDef", into = "InputDef")]
pub struct Input {
    pub mapping: InputMapping,
    pub queue_size: Option<usize>,
    pub input_timeout: Option<f64>,
}

impl PartialEq for Input {
    fn eq(&self, other: &Self) -> bool {
        self.mapping == other.mapping
            && self.queue_size == other.queue_size
            && self.input_timeout.map(f64::to_bits) == other.input_timeout.map(f64::to_bits)
    }
}

impl Eq for Input {}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum InputDef {
    MappingOnly(InputMapping),
    WithOptions {
        source: InputMapping,
        queue_size: Option<usize>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        input_timeout: Option<f64>,
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
                },
                Self::WithOptions {
                    source: s2,
                    queue_size: q2,
                    input_timeout: t2,
                },
            ) => s1 == s2 && q1 == q2 && t1.map(f64::to_bits) == t2.map(f64::to_bits),
            _ => false,
        }
    }
}

impl Eq for InputDef {}

impl From<Input> for InputDef {
    fn from(input: Input) -> Self {
        if input.queue_size.is_none() && input.input_timeout.is_none() {
            Self::MappingOnly(input.mapping)
        } else {
            Self::WithOptions {
                source: input.mapping,
                queue_size: input.queue_size,
                input_timeout: input.input_timeout,
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
            },
            InputDef::WithOptions {
                source,
                queue_size,
                input_timeout,
            } => Self {
                mapping: source,
                queue_size,
                input_timeout,
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
        static ADORA_NODE_ID: OnceCell<NodeId> = OnceCell::new();

        match self {
            InputMapping::User(mapping) => &mapping.source,
            InputMapping::Timer { .. } => ADORA_NODE_ID.get_or_init(|| NodeId("adora".to_string())),
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
                Some((other, _)) => {
                    return Err(format!("unknown adora input `{other}`"));
                }
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LocalCommunicationConfig {
    Tcp,
    Shmem,
    UnixDomain,
}

impl Default for LocalCommunicationConfig {
    fn default() -> Self {
        Self::Tcp
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields, rename_all = "lowercase")]
pub enum RemoteCommunicationConfig {
    Tcp,
    // TODO:a
    // Zenoh {
    //     config: Option<serde_yaml::Value>,
    //     prefix: String,
    // },
}

impl Default for RemoteCommunicationConfig {
    fn default() -> Self {
        Self::Tcp
    }
}
