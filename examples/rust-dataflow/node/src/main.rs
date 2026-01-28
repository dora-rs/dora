use dora_node_api::{
    self, DoraNode, Event, EventStream, IntoArrow, dora_core::config::DataId, init_tracing,
};
use eyre::Context;
use tracing::{Level, span};

#[cfg(test)]
mod tests;

fn main() -> eyre::Result<()> {
    println!("hello");

    let (node, events) = DoraNode::init_from_env()?;
    let rt = tokio::runtime::Builder::new_multi_thread()
        .build()
        .context("failed to build tokio runtime")?;
    let rt_guard = rt.enter();
    let tracing_guard =
        init_tracing(&node.id().clone(), node.dataflow_id()).context("failed to init tracing")?;

    let result = run(node, events);

    drop(tracing_guard);
    drop(rt_guard);
    result
}

fn run(mut node: DoraNode, mut events: EventStream) -> eyre::Result<()> {
    let span = span!(Level::INFO, "Node Span");
    let _enter = span.enter();
    // use a fixed seed for reproducibility (we use this node's output in integration tests)
    fastrand::seed(42);

    let output = DataId::from("random".to_owned());
    for i in 0..100 {
        let event = match events.recv() {
            Some(input) => input,
            None => break,
        };

        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "tick" => {
                    let random: u64 = fastrand::u64(..);
                    tracing::info!("tick {i} with data {data:?}, sending {random:#x}");
                    node.send_output(output.clone(), metadata.parameters, random.into_arrow())
                        .unwrap();
                }
                other => {
                    tracing::info!(input = other, "Ignoring unexpected input");
                }
            },
            Event::Stop(_) => {
                tracing::info!("Received stop event");
            }
            other => {
                tracing::warn!(?other, "Received unexpected input");
            }
        }
    }
    Ok(())
}
