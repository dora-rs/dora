use adora_node_api::{AdoraNode, Event, arrow};
use eyre::{ContextCompat, bail};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = AdoraNode::init_from_env()?;
    let mut received_count: i64 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } if id.as_str() == "values" => {
                // Python sends pa.array([i * 10], type=pa.int64()) so we receive a single i64
                let arr = data
                    .as_any()
                    .downcast_ref::<arrow::array::Int64Array>()
                    .context("expected Int64Array from Python sender")?;
                if arr.is_empty() {
                    bail!("received empty array from Python sender");
                }
                if arr.len() != 1 {
                    bail!("expected 1 element from Python, got {}", arr.len());
                }
                let value = arr.value(0);
                if value % 10 != 0 || !(0..=90).contains(&value) {
                    bail!(
                        "unexpected value from Python sender: {value} (expected multiple of 10 in [0, 90])"
                    );
                }
                eprintln!("rust-receiver: validated value {value}");
                received_count += 1;
            }
            Event::Stop(_) => {
                eprintln!("rust-receiver: stopping after {received_count} messages");
                break;
            }
            _ => {}
        }
    }

    if received_count < 5 {
        bail!(
            "rust-receiver got only {received_count} messages from Python sender (expected >= 5)"
        );
    }
    eprintln!("rust-receiver: SUCCESS - validated {received_count} messages");
    Ok(())
}
