//! Emit one `value` on the first tick, then stay alive and silent.
//!
//! The silence-after-first-message pattern is what drives the
//! `input_timeout` path in the daemon: the upstream process is still
//! running (so channel-close doesn't fire) but no new data is flowing,
//! so the consumer must rely on `input_timeout` to be told the input
//! has gone stale.

use dora_node_api::{DoraNode, Event, IntoArrow, dora_core::config::DataId};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;
    let output = DataId::from("value".to_owned());

    let mut sent = false;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, .. } if id.as_str() == "tick" => {
                if !sent {
                    node.send_output(output.clone(), metadata.parameters, 42i64.into_arrow())
                        .context("failed to send value")?;
                    sent = true;
                }
                // Subsequent ticks are intentionally ignored so the
                // downstream must rely on `input_timeout` to notice
                // that data has gone stale.
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
