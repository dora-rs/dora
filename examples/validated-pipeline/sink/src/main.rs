use adora_node_api::{AdoraNode, Event, arrow};
use eyre::{ContextCompat, bail};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = AdoraNode::init_from_env()?;
    let mut received: i64 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } if id.as_str() == "doubled" => {
                let arr = data
                    .as_any()
                    .downcast_ref::<arrow::array::Int64Array>()
                    .context("expected Int64Array")?;
                if arr.is_empty() {
                    bail!("received empty array");
                }
                let value = arr.value(0);
                let expected = received * 2;
                if value != expected {
                    bail!("sink: expected {expected}, got {value} (message #{received})");
                }
                eprintln!("sink: validated {value} (expected {expected})");
                received += 1;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    if received < 5 {
        bail!("sink got only {received} messages (expected >= 5)");
    }
    eprintln!("sink: SUCCESS - validated {received} doubled values");
    Ok(())
}
