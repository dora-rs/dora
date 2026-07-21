use thiserror::Error;
use zenoh::key_expr::OwnedKeyExpr;

use super::compatibility::RosTypeIdentity;

#[derive(Debug, Error)]
pub enum KeyError {
    #[error("invalid rmw_zenoh key expression")]
    Invalid,
    #[error("malformed rmw_zenoh liveliness token")]
    Malformed,
}

#[derive(Clone, Debug)]
pub struct DataKey(OwnedKeyExpr);
impl DataKey {
    pub fn new(
        domain: usize,
        fully_qualified_name: &str,
        identity: &RosTypeIdentity,
    ) -> Result<Self, KeyError> {
        let name = fully_qualified_name
            .strip_prefix('/')
            .unwrap_or(fully_qualified_name);
        if name.is_empty() || name.bytes().any(|byte| matches!(byte, b'*' | b'$' | b'%')) {
            return Err(KeyError::Invalid);
        }
        let value = format!(
            "{domain}/{name}/{}/{}",
            identity.dds_name,
            identity.key_hash_component()
        );
        OwnedKeyExpr::try_from(value)
            .map(Self)
            .map_err(|_| KeyError::Invalid)
    }
    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EntityKind {
    Node,
    Publisher,
    Subscription,
    Service,
    Client,
}
impl EntityKind {
    fn code(self) -> &'static str {
        match self {
            Self::Node => "NN",
            Self::Publisher => "MP",
            Self::Subscription => "MS",
            Self::Service => "SS",
            Self::Client => "SC",
        }
    }
    fn parse(value: &str) -> Option<Self> {
        Some(match value {
            "NN" => Self::Node,
            "MP" => Self::Publisher,
            "MS" => Self::Subscription,
            "SS" => Self::Service,
            "SC" => Self::Client,
            _ => return None,
        })
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TopicToken {
    pub name: String,
    pub type_name: String,
    pub type_hash: String,
    pub qos: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LivelinessKey {
    key: OwnedKeyExpr,
    pub domain: usize,
    pub zid: String,
    pub nid: String,
    pub id: String,
    pub kind: EntityKind,
    pub enclave: String,
    pub namespace: String,
    pub node_name: String,
    pub topic: Option<TopicToken>,
}

impl LivelinessKey {
    #[allow(clippy::too_many_arguments)]
    pub fn node(
        domain: usize,
        zid: &str,
        nid: &str,
        id: &str,
        enclave: &str,
        namespace: &str,
        node_name: &str,
    ) -> Result<Self, KeyError> {
        Self::new(
            domain,
            zid,
            nid,
            id,
            EntityKind::Node,
            enclave,
            namespace,
            node_name,
            None,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn endpoint(
        domain: usize,
        zid: &str,
        nid: &str,
        id: &str,
        kind: EntityKind,
        enclave: &str,
        namespace: &str,
        node_name: &str,
        topic: TopicToken,
    ) -> Result<Self, KeyError> {
        if kind == EntityKind::Node {
            return Err(KeyError::Invalid);
        }
        Self::new(
            domain,
            zid,
            nid,
            id,
            kind,
            enclave,
            namespace,
            node_name,
            Some(topic),
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new(
        domain: usize,
        zid: &str,
        nid: &str,
        id: &str,
        kind: EntityKind,
        enclave: &str,
        namespace: &str,
        node_name: &str,
        topic: Option<TopicToken>,
    ) -> Result<Self, KeyError> {
        let mut parts = vec![
            "@ros2_lv".into(),
            domain.to_string(),
            zid.into(),
            nid.into(),
            id.into(),
            kind.code().into(),
            mangle(enclave)?,
            mangle(namespace)?,
            mangle(node_name)?,
        ];
        if let Some(topic) = &topic {
            parts.extend([
                mangle(&topic.name)?,
                mangle(&topic.type_name)?,
                mangle(&topic.type_hash)?,
                topic.qos.clone(),
            ]);
        }
        let key = OwnedKeyExpr::try_from(parts.join("/")).map_err(|_| KeyError::Invalid)?;
        Ok(Self {
            key,
            domain,
            zid: zid.into(),
            nid: nid.into(),
            id: id.into(),
            kind,
            enclave: enclave.into(),
            namespace: namespace.into(),
            node_name: node_name.into(),
            topic,
        })
    }
    pub fn as_str(&self) -> &str {
        self.key.as_str()
    }
    pub fn parse(value: &str) -> Result<Self, KeyError> {
        OwnedKeyExpr::try_from(value.to_owned()).map_err(|_| KeyError::Malformed)?;
        let p: Vec<_> = value.split('/').collect();
        if p.len() != 9 && p.len() != 13 || p.first() != Some(&"@ros2_lv") {
            return Err(KeyError::Malformed);
        }
        let topic = (p.len() == 13).then(|| TopicToken {
            name: demangle(p[9]),
            type_name: demangle(p[10]),
            type_hash: demangle(p[11]),
            qos: p[12].into(),
        });
        Self::new(
            p[1].parse().map_err(|_| KeyError::Malformed)?,
            p[2],
            p[3],
            p[4],
            EntityKind::parse(p[5]).ok_or(KeyError::Malformed)?,
            &demangle(p[6]),
            &demangle(p[7]),
            &demangle(p[8]),
            topic,
        )
    }
}

fn mangle(value: &str) -> Result<String, KeyError> {
    if value.contains("%%") {
        return Err(KeyError::Invalid);
    }
    Ok(value.replace('/', "%"))
}
fn demangle(value: &str) -> String {
    value.replace('%', "/")
}
