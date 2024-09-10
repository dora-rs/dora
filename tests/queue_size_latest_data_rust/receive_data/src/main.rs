use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use chrono::Utc;
use dora_node_api::{
    self,
    arrow::{
        array::{AsArray, PrimitiveArray},
        datatypes::UInt64Type,
    },
    DoraNode,
};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    // Voluntarily sleep for 5 seconds to ensure that the node is dropping the oldest input
    sleep(Duration::from_secs(5));

    while let Some(event) = events.recv() {
        match event {
            dora_node_api::Event::Input {
                id: _,
                metadata: _,
                data,
            } => {
                let data: &PrimitiveArray<UInt64Type> = data.as_primitive();
                let time: &[u64] = data.values();
                let now = Utc::now();
                let timestamp: u64 = now.timestamp() as u64;
                println!("Time Difference: {:?}", timestamp - time[0]);
                assert!(
                    timestamp - time[0] < 2,
                    "Time difference should be less than 2 seconds as data is sent every seconds"
                );
            }
            _ => {}
        }
    }
    Ok(())
}
