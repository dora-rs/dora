use std::time::{Duration, Instant};

use dora_node_api::{
    self, DoraNode, Event,
    arrow::{
        array::{AsArray, PrimitiveArray},
        datatypes::UInt64Type,
    },
};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;
    let timeout = Duration::from_secs(5);
    let poll_interval = Duration::from_millis(10);
    let deadline = Instant::now() + timeout;

    while Instant::now() < deadline {
        let Some(event) = events.recv_timeout(poll_interval) else {
            break;
        };

        if let Event::Input {
            id: _,
            metadata,
            data,
        } = event
        {
            let data: &PrimitiveArray<UInt64Type> = data.as_primitive();
            let _time: u64 = data.values()[0];
            let time_metadata = metadata.timestamp();
            let duration_metadata = time_metadata.get_time().to_system_time().elapsed()?;
            println!("Latency duration: {duration_metadata:?}");

            if duration_metadata < Duration::from_millis(500) {
                return Ok(());
            }
        }
    }

    eyre::bail!(
        "timed out waiting for input with latency below 500ms within {:?}",
        timeout
    )
}
