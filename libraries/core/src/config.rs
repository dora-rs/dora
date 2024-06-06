use once_cell::sync::OnceCell;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::{
    borrow::Borrow,
    collections::{BTreeMap, BTreeSet},
    convert::Infallible,
    fmt,
    str::FromStr,
    time::Duration,
};

#[derive(
    Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, JsonSchema,
)]
pub struct NodeId(String);

impl FromStr for NodeId {
    type Err = Infallible;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(Self(s.to_owned()))
    }
}

impl From<String> for NodeId {
    fn from(id: String) -> Self {
        Self(id)
    }
}

impl std::fmt::Display for NodeId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(&self.0, f)
    }
}

impl AsRef<str> for NodeId {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

#[derive(
    Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, JsonSchema,
)]
pub struct OperatorId(String);

impl FromStr for OperatorId {
    type Err = Infallible;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(Self(s.to_owned()))
    }
}

impl From<String> for OperatorId {
    fn from(id: String) -> Self {
        Self(id)
    }
}

impl std::fmt::Display for OperatorId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(&self.0, f)
    }
}

impl AsRef<str> for OperatorId {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

#[derive(
    Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, JsonSchema,
)]
pub struct DataId(String);

impl From<DataId> for String {
    fn from(id: DataId) -> Self {
        id.0
    }
}

impl From<String> for DataId {
    fn from(id: String) -> Self {
        Self(id)
    }
}

impl std::fmt::Display for DataId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(&self.0, f)
    }
}

impl std::ops::Deref for DataId {
    type Target = String;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl AsRef<String> for DataId {
    fn as_ref(&self) -> &String {
        &self.0
    }
}

impl AsRef<str> for DataId {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

impl Borrow<String> for DataId {
    fn borrow(&self) -> &String {
        &self.0
    }
}

impl Borrow<str> for DataId {
    fn borrow(&self) -> &str {
        &self.0
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
        let (source, output) = string
            .split_once('/')
            .ok_or_else(|| serde::de::Error::custom("input must start with `<source>/`"))?;

        let deserialized = match source {
            "dora" => match output.split_once('/') {
                Some(("timer", output)) => {
                    let (unit, value) = output.split_once('/').ok_or_else(|| {
                        serde::de::Error::custom(
                            "timer input must specify unit and value (e.g. `secs/5` or `millis/100`)",
                        )
                    })?;
                    let interval = match unit {
                        "secs" => {
                            let value = value.parse().map_err(|_| {
                                serde::de::Error::custom(format!(
                                    "secs must be an integer (got `{value}`)"
                                ))
                            })?;
                            Duration::from_secs(value)
                        }
                        "millis" => {
                            let value = value.parse().map_err(|_| {
                                serde::de::Error::custom(format!(
                                    "millis must be an integer (got `{value}`)"
                                ))
                            })?;
                            Duration::from_millis(value)
                        }
                        other => {
                            return Err(serde::de::Error::custom(format!(
                                "timer unit must be either secs or millis (got `{other}`"
                            )))
                        }
                    };
                    Self::Timer { interval }
                }
                Some((other, _)) => {
                    return Err(serde::de::Error::custom(format!(
                        "unknown dora input `{other}`"
                    )))
                }
                None => return Err(serde::de::Error::custom("dora input has invalid format")),
            },
            _ => Self::User(UserInputMapping {
                source: source.to_owned().into(),
                output: output.to_owned().into(),
            }),
        };

        Ok(deserialized)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, JsonSchema)]
pub struct UserInputMapping {
    pub source: NodeId,
    pub output: DataId,
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

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields, from = "InputDef", into = "InputDef")]
pub struct Input {
    pub mapping: InputMapping,
    pub queue_size: Option<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
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
