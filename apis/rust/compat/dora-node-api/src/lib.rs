//! Backward-compatible shim crate that re-exports [`adora_node_api`] under the
//! `dora-node-api` name so that existing dora-hub nodes compile without changes.
//!
//! Usage in dora-hub nodes:
//! ```rust,ignore
//! use dora_node_api::{DoraNode, Event};
//! ```

pub use adora_node_api::*;

// Re-export the crate itself so `dora_node_api::dora_core` works.
pub use adora_node_api::adora_core as dora_core;

#[cfg(test)]
mod tests {
    //! Verify that every import pattern used by dora-hub nodes compiles.

    // Pattern: `use dora_node_api::{DoraNode, Event};`
    #[allow(unused_imports)]
    use crate::{DoraEvent, DoraNode, Event, EventStream};

    // Pattern: `use dora_node_api::dora_core::config::DataId;`
    #[allow(unused_imports)]
    use crate::dora_core::config::DataId;

    // Pattern: `use dora_node_api::arrow;`
    #[allow(unused_imports)]
    use crate::arrow;

    // Common re-exports used by dora-hub
    #[allow(unused_imports)]
    use crate::{
        DataSample, Metadata, MetadataParameters, NodeError, NodeResult, StopCause, TryRecvError,
        ZERO_COPY_THRESHOLD,
    };

    #[test]
    fn dora_node_is_adora_node() {
        // DoraNode and AdoraNode must be the same type.
        fn _assert_same_type(_: DoraNode) {}
        fn _takes_adora(node: super::AdoraNode) {
            _assert_same_type(node);
        }
    }

    #[test]
    fn dora_core_config_types_accessible() {
        // DataId must be constructible via the dora_core path.
        let _id: DataId = DataId::from("test_input".to_string());
    }

    #[test]
    fn event_variants_accessible() {
        // All Event variants that dora-hub matches on must exist.
        fn _match_event(event: &Event) {
            match event {
                Event::Input {
                    id,
                    metadata: _,
                    data: _,
                } => {
                    let _: &crate::dora_core::config::DataId = id;
                }
                Event::InputClosed { id: _ } => {}
                Event::Stop(_) => {}
                _ => {}
            }
        }
    }

    #[test]
    fn metadata_constants_accessible() {
        // Service/action metadata keys used by dora-hub nodes.
        let _ = crate::metadata::REQUEST_ID;
        let _ = crate::metadata::GOAL_ID;
        let _ = crate::metadata::GOAL_STATUS;
    }

    #[test]
    fn zero_copy_threshold_value() {
        assert_eq!(ZERO_COPY_THRESHOLD, 4096);
    }
}
