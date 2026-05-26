//! Filter node for the `dora node` lifecycle E2E test fixture
//! (issue #1703). Added dynamically at runtime via
//! `dora node add --from-yaml filter-node.yml`. Counts inputs on
//! the `input` edge and re-emits a `u64` running total on `output`,
//! so the lifecycle harness can `dora node connect` it into the
//! pipeline and exercise the mapping path without needing arrow
//! pass-through plumbing.

use dora_node_api::{DoraNode, Event, IntoArrow, StopCause, dora_core::config::DataId};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let output = DataId::from("output".to_owned());
    let mut count: u64 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "input" => {
                count = count.wrapping_add(1);
                node.send_output(output.clone(), metadata.parameters, count.into_arrow())?;
            }
            // Same selective-stop pattern as receiver: only exit on
            // operator-initiated Manual stop. See receiver/src/main.rs.
            Event::Stop(StopCause::Manual) => break,
            _ => {}
        }
    }
    Ok(())
}
