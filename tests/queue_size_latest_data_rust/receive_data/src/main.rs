use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use chrono::Utc;
use dora_node_api::{
    self,
    arrow::{
        array::{AsArray, PrimitiveArray},
        datatypes::{Int64Type, UInt64Type},
        temporal_conversions::EPOCH_DAYS_FROM_CE,
    },
    dora_core::config::DataId,
    DoraNode, Metadata,
};
use uhlc::system_time_clock;

fn main() -> eyre::Result<()> {
    let mut printed_error = String::new();
    let (node, mut events) = DoraNode::init_from_env()?;

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
