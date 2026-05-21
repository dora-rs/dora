//! Sender node for the `dora node` lifecycle E2E test fixture
//! (issue #1703). Emits a monotonically-increasing `u64` counter on
//! the `value` output every tick. Runs indefinitely until a Stop
//! event lands so the lifecycle harness can outlive the test cycle.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let output = DataId::from("value".to_owned());
    let mut counter: u64 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata,
                data: _,
                ..
            } if id.as_str() == "tick" => {
                let payload = counter.into_arrow();
                node.send_output(output.clone(), metadata.parameters, payload)?;
                counter = counter.wrapping_add(1);
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
