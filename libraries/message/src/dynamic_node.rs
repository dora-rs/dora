//! The dynamic-node configuration reply uses JSON, unlike ordinary node IPC.
//!
//! Keep its optional bootstrap information outside `DaemonReply` and
//! `NodeConfig`: their positional binary representations must not change.
//! The `NodeConfig` variant and `result` retain the legacy JSON shape. Old
//! clients ignore `zenoh`; new clients default it when an old daemon replies.

use crate::daemon_to_node::NodeConfig;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum DynamicNodeConfigReply {
    NodeConfig {
        result: Result<NodeConfig, String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        zenoh: Option<DynamicNodePeering>,
    },
}

/// A dynamic node's listener and the local graph neighbours it must dial.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct DynamicNodePeering {
    pub listen: String,
    pub connect: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::daemon_to_node::DaemonReply;

    fn config() -> NodeConfig {
        NodeConfig {
            dataflow_id: uuid::Uuid::nil(),
            node_id: "dynamic".to_owned().into(),
            run_config: Default::default(),
            daemon_communication: None,
            dataflow_descriptor: serde_yaml::Value::Null,
            dynamic: true,
            write_events_to: None,
            restart_count: 0,
            output_routing: None,
        }
    }

    #[test]
    fn old_client_accepts_reply_with_peering() {
        let reply = DynamicNodeConfigReply::NodeConfig {
            result: Ok(config()),
            zenoh: Some(DynamicNodePeering {
                listen: "tcp/127.0.0.1:1234".into(),
                connect: vec!["tcp/127.0.0.1:1235".into()],
            }),
        };
        let encoded = serde_json::to_vec(&reply).unwrap();
        let old: DaemonReply = serde_json::from_slice(&encoded).unwrap();
        assert!(matches!(old, DaemonReply::NodeConfig { result: Ok(c) } if c.dynamic));
        let DynamicNodeConfigReply::NodeConfig { zenoh, .. } =
            serde_json::from_slice(&encoded).unwrap();
        assert_eq!(zenoh.unwrap().connect, ["tcp/127.0.0.1:1235"]);
    }

    #[test]
    fn new_client_accepts_legacy_success_and_error() {
        for result in [Ok(config()), Err("no node with ID `dynamic`".into())] {
            let success = result.is_ok();
            let encoded = serde_json::to_vec(&DaemonReply::NodeConfig { result }).unwrap();
            let DynamicNodeConfigReply::NodeConfig { result, zenoh } =
                serde_json::from_slice(&encoded).unwrap();
            assert_eq!(result.is_ok(), success);
            assert!(zenoh.is_none());
        }
    }
}
