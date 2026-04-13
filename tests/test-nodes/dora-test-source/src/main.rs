use dora_node_api::{self, DoraNode, dora_core::config::DataId};
use eyre::Context;
use serde::Deserialize;
use std::time::Duration;

#[derive(Deserialize)]
struct TestMessage {
    id: String,
    data: String,
    delay_ms: u64,
}

fn main() -> eyre::Result<()> {
    let (mut node, _events) = DoraNode::init_from_env()?;

    let config_str =
        std::env::var("TEST_DATA").context("TEST_DATA environment variable not set")?;
    let messages: Vec<TestMessage> =
        serde_json::from_str(&config_str).context("failed to parse TEST_DATA json")?;

    for msg in messages {
        std::thread::sleep(Duration::from_millis(msg.delay_ms));
        let data = msg.data.as_bytes();
        let output_id = DataId::from(msg.id);
        node.send_output_bytes(
            output_id,
            dora_node_api::MetadataParameters::default(),
            data.len(),
            data,
        )
        .unwrap();
    }

    Ok(())
}
