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

#[derive(
    Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, JsonSchema,
)]
pub struct NodeId(pub(crate) String);

#[derive(Debug)]
pub struct NodeIdContainsSlash;

impl std::fmt::Display for NodeIdContainsSlash {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "NodeId must not contain `/`")
    }
}

impl std::error::Error for NodeIdContainsSlash {}

impl FromStr for NodeId {
    type Err = NodeIdContainsSlash;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s.contains('/') {
            return Err(NodeIdContainsSlash);
        }
        Ok(Self(s.to_owned()))
    }
}

impl From<String> for NodeId {
    fn from(id: String) -> Self {
        if let Err(e) = validate_id(&id) {
            log::warn!("NodeId '{id}' failed validation: {e}");
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
        if let Err(e) = validate_id(&id) {
            log::warn!("DataId '{id}' failed validation: {e}");
        }
        Self(id)
    }
}

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
    fn node_id_from_str_rejects_slash() {
        assert!(NodeId::from_str("hello").is_ok());
        assert!(NodeId::from_str("hello/world").is_err());
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
