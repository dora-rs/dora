use dora_node_api::{self, dora_core::daemon_messages::NodeEvent, DoraNode};
use eyre::{bail, Context, ContextCompat};

fn main() -> eyre::Result<()> {
    let (_node, events) = DoraNode::init_from_env()?;

    while let Ok(event) = events.recv() {
        match event {
            NodeEvent::Stop => break,
            NodeEvent::Input {
                id,
                metadata: _,
                data,
            } => match id.as_str() {
                "message" => {
                    let data = data.wrap_err("no data")?.map()?;
                    let received_string = std::str::from_utf8(&data)
                        .wrap_err("received message was not utf8-encoded")?;
                    println!("received message: {}", received_string);
                    if !received_string.starts_with("operator received random value ") {
                        bail!("unexpected message format (should start with 'operator received random value')")
                    }
                    if !received_string.ends_with(" ticks") {
                        bail!("unexpected message format (should end with 'ticks')")
                    }
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
        }
    }

    Ok(())
}
