use std::{thread::sleep, time::Duration};

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
                metadata,
                data,
            } => {
                let data: &PrimitiveArray<UInt64Type> = data.as_primitive();
                let time: u64 = data.values()[0];
                let time_metadata = metadata.timestamp();
                let duration_metadata = time_metadata.get_time().to_system_time().elapsed()?;
                println!("Latency duration: {:?}", duration_metadata);
                assert!(
                    duration_metadata < Duration::from_millis(500),
                    "Time difference should be less than 500ms"
                );
                let time = time as f64;
                let now = Utc::now();

                let timestamp = now.timestamp_nanos_opt().unwrap() as f64;
                let duration = Duration::from_nanos((timestamp - time) as u64);
                println!("Latency duration: {:?}", duration);
                assert!(
                    duration < Duration::from_millis(500),
                    "Time difference should be less than 500ms"
                );
            }
            _ => {}
        }
    }
    Ok(())
}
