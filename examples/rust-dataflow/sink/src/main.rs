use dora_node_api::{self, daemon::Event, DoraNode};
use eyre::ContextCompat;

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata: _,
                data,
            } => match id.as_str() {
                "message" => {
                    let data = data.wrap_err("no data")?;
                    let raw = (&data[..]).try_into().unwrap();

                    println!("received data: {:#x}", u64::from_le_bytes(raw));
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            Event::Stop => {
                println!("Received manual stop");
            }
            Event::InputClosed { id } => {
                println!("Input `{id}` was closed");
            }
            other => eprintln!("Received unexpected input: {other:?}"),
        }
    }

    Ok(())
}
