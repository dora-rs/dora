use std::{borrow::Borrow, convert::Infallible, str::FromStr};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// Validate that an identifier contains only safe characters: `[a-zA-Z0-9_.-]`.
fn validate_id(id: &str) -> Result<(), InvalidId> {
    if id.is_empty() {
        return Err(InvalidId("identifier must not be empty".into()));
    }
    if let Some(ch) = id
        .chars()
        .find(|c| !c.is_ascii_alphanumeric() && *c != '_' && *c != '-' && *c != '.')
    {
        return Err(InvalidId(format!(
            "identifier contains invalid character '{ch}' -- only [a-zA-Z0-9_.-] are allowed"
        )));
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InvalidId(pub String);

impl std::fmt::Display for InvalidId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for InvalidId {}

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, JsonSchema)]
pub struct NodeId(pub(crate) String);

impl<'de> Deserialize<'de> for NodeId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        validate_id(&s).map_err(serde::de::Error::custom)?;
        Ok(NodeId(s))
    }
}

impl FromStr for NodeId {
    type Err = InvalidId;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        validate_id(s)?;
        Ok(Self(s.to_owned()))
    }
}

/// # Panics
///
/// Panics if `id` contains invalid characters (not in `[a-zA-Z0-9_.-]`).
///
/// **For untrusted input, use `id.parse::<NodeId>()`** which calls
/// `FromStr::from_str` and returns `Result<Self, InvalidId>`.
///
/// Do NOT use `NodeId::try_from(s)`: `TryFrom<String>` is the
/// auto-derived blanket impl that delegates to this `From` impl, so it
/// panics exactly like `.into()`. Only `parse::<NodeId>()` / `from_str`
/// is fallible.
impl From<String> for NodeId {
    fn from(id: String) -> Self {
        if let Err(e) = validate_id(&id) {
            panic!("invalid NodeId '{id}': {e}");
        }
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

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, JsonSchema)]
pub struct DataId(String);

impl<'de> Deserialize<'de> for DataId {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        validate_id(&s).map_err(serde::de::Error::custom)?;
        Ok(DataId(s))
    }
}

impl FromStr for DataId {
    type Err = InvalidId;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        validate_id(s)?;
        Ok(Self(s.to_owned()))
    }
}

impl From<DataId> for String {
    fn from(id: DataId) -> Self {
        id.0
    }
}

/// # Panics
///
/// Panics if `id` contains invalid characters. Prefer `id.parse::<DataId>()`
/// or `DataId::try_from(s)` when handling untrusted input.
impl From<String> for DataId {
    fn from(id: String) -> Self {
        if let Err(e) = validate_id(&id) {
            panic!("invalid DataId '{id}': {e}");
        }
        Self(id)
    }
}

/// # Panics
///
/// Panics if `id` contains invalid characters. Prefer `id.parse::<DataId>()`
/// when handling untrusted input.
impl From<&str> for DataId {
    fn from(id: &str) -> Self {
        id.to_owned().into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_ids() {
        assert!(validate_id("my-node").is_ok());
        assert!(validate_id("my_node").is_ok());
        assert!(validate_id("MyNode123").is_ok());
        assert!(validate_id("node.v2").is_ok());
        assert!(validate_id("a").is_ok());
    }

    #[test]
    fn invalid_ids() {
        assert!(validate_id("").is_err());
        assert!(validate_id("node/output").is_err());
        assert!(validate_id("node name").is_err());
        assert!(validate_id("node;rm").is_err());
        assert!(validate_id("node\0").is_err());
    }

    #[test]
    fn node_id_from_str_rejects_invalid() {
        assert!(NodeId::from_str("hello").is_ok());
        assert!(NodeId::from_str("hello/world").is_err());
        assert!(NodeId::from_str("hello world").is_err());
        assert!(NodeId::from_str("").is_err());
    }

    #[test]
    #[should_panic(expected = "invalid NodeId")]
    fn node_id_from_string_panics_on_invalid() {
        let _id: NodeId = "bad/id".to_string().into();
    }

    #[test]
    fn node_id_parse_rejects_invalid() {
        assert!("hello".parse::<NodeId>().is_ok());
        assert!("bad/id".parse::<NodeId>().is_err());
        assert!("".parse::<NodeId>().is_err());
    }

    #[test]
    fn data_id_parse_rejects_invalid() {
        assert!("output".parse::<DataId>().is_ok());
        assert!("bad;id".parse::<DataId>().is_err());
    }

    #[test]
    fn node_id_deserialize_rejects_invalid() {
        let result: Result<NodeId, _> = serde_json::from_str("\"bad/id\"");
        assert!(result.is_err());
    }

    #[test]
    fn data_id_deserialize_rejects_invalid() {
        let result: Result<DataId, _> = serde_json::from_str("\"bad;id\"");
        assert!(result.is_err());
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
