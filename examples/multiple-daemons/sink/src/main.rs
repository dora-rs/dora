use dora_node_api::{self, DoraNode, Event, EventStream};
use eyre::{Context, bail};

#[cfg(test)]
mod tests;

fn main() -> eyre::Result<()> {
    let (node, events) = DoraNode::init_from_env()?;
    run(node, events)
}

fn run(node: DoraNode, mut events: EventStream) -> eyre::Result<()> {
    // `random` arrives cross-machine (rust-node runs on machine A) from an
    // output that also has a same-machine consumer (runtime-node on A), so it
    // exercises the mixed local+remote delivery of the daemon-pinned path:
    // the payload must reach A's operator locally AND this sink on machine B
    // via inter-daemon forwarding. Count it and fail the example if nothing
    // arrived — a liveness-only run would pass even if forwarding starved.
    // Only enforced when the input is actually declared (the integration
    // tests drive `run` with a config that has no `random` input).
    let expects_random =
        node.node_config()
            .inputs
            .contains_key(&dora_node_api::dora_core::config::DataId::from(
                "random".to_string(),
            ));
    let mut random_count: u64 = 0;
    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata: _,
                data,
            } => match id.as_str() {
                "message" => {
                    let received_string: &str =
                        TryFrom::try_from(&data).context("expected string message")?;
                    println!("sink received message: {received_string}");
                    if !received_string.starts_with("operator received random value ") {
                        bail!(
                            "unexpected message format (should start with 'operator received random value')"
                        )
                    }
                    if !received_string.ends_with(" ticks") {
                        bail!("unexpected message format (should end with 'ticks')")
                    }
                }
                "random" => {
                    random_count += 1;
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            Event::Stop(_) => {
                println!("Received stop");
            }
            Event::InputClosed { id } => {
                println!("Input `{id}` was closed");
            }
            other => eprintln!("Received unexpected input: {other:?}"),
        }
    }

    if expects_random {
        if random_count == 0 {
            bail!(
                "sink never received a cross-machine `random` value: \
                 inter-daemon forwarding of the daemon-pinned output starved"
            );
        }
        println!("sink received {random_count} cross-machine random values");
    }

    Ok(())
}
