use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet},
    convert::Infallible,
    fmt::Write as _,
    str::FromStr,
};

#[derive(Debug, PartialEq, Eq, Serialize, Deserialize)]
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InputMapping {
    pub source: NodeId,
    pub operator: Option<OperatorId>,
    pub output: DataId,
}

impl Serialize for InputMapping {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        if let Some(operator) = &self.operator {
            serializer.collect_str(&format_args!("{}/{operator}/{}", self.source, self.output))
        } else {
            serializer.collect_str(&format_args!("{}/{}", self.source, self.output))
        }
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

        Ok(Self {
            source: source.to_owned().into(),
            operator: operator.map(|o| o.to_owned().into()),
            output: output.to_owned().into(),
        })
    }
}

#[derive(Debug, Serialize, Deserialize)]
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
