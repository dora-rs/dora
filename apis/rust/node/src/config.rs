use once_cell::sync::OnceCell;
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet},
    convert::Infallible,
    fmt::{self, Write as _},
    str::FromStr,
    time::Duration,
};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeRunConfig {
    #[serde(default)]
    pub inputs: BTreeMap<DataId, InputMapping>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
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

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
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

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
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

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
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

    pub fn operator(&self) -> &Option<OperatorId> {
        match self {
            InputMapping::User(mapping) => &mapping.operator,
            InputMapping::Timer { .. } => &None,
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
                if let Some(operator) = &mapping.operator {
                    write!(f, "{}/{operator}/{}", mapping.source, mapping.output)
                } else {
                    write!(f, "{}/{}", mapping.source, mapping.output)
                }
            }
        }
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
        let (source, rest) = string
            .split_once('/')
            .ok_or_else(|| serde::de::Error::custom("input must start with `<source>/`"))?;

        let (operator, output) = rest
            .split_once('/')
            .map(|(op, out)| (Some(op), out))
            .unwrap_or((None, rest));

        let deserialized = match source {
            "dora" => match operator {
                Some("timer") => {
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
                Some(other) => {
                    return Err(serde::de::Error::custom(format!(
                        "unknown dora input `{other}`"
                    )))
                }
                None => {
                    return Err(serde::de::Error::custom(
                        "dora input has invalid format".to_string(),
                    ))
                }
            },
            _ => Self::User(UserInputMapping {
                source: source.to_owned().into(),
                operator: operator.map(|o| o.to_owned().into()),
                output: output.to_owned().into(),
            }),
        };

        Ok(deserialized)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct UserInputMapping {
    pub source: NodeId,
    pub operator: Option<OperatorId>,
    pub output: DataId,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields, rename_all = "lowercase")]
pub enum CommunicationConfig {
    Zenoh {
        #[serde(default)]
        config: zenoh_config::Config,
        prefix: String,
    },
}

impl CommunicationConfig {
    pub fn add_topic_prefix(&mut self, prefix: &str) {
        match self {
            CommunicationConfig::Zenoh {
                prefix: zenoh_prefix,
                ..
            } => {
                write!(zenoh_prefix, "/{}", prefix).unwrap();
            }
        }
    }
}
