use std::sync::LazyLock;

use dora_node_api::{
    self, DoraNode, Event, EventStream, IntoArrow, dora_core::config::DataId, init_tracing,
};

#[cfg(test)]
mod tests;
use tokio::runtime::{Builder, Runtime};
use tracing::{Level, span, warn};
static RUNTIME: LazyLock<Runtime> = LazyLock::new(|| {
    Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .unwrap()
});
fn main() -> eyre::Result<()> {
    println!("hello");

    let (node, events) = DoraNode::init_from_env()?;
    let id = node.id().clone();
    let dataflow_id = node.dataflow_id().clone();
    RUNTIME.spawn(async move {
        let guard = init_tracing(&id, &dataflow_id).unwrap();
        loop {
            tokio::time::sleep(std::time::Duration::from_secs(1)).await;
        }
    });

    run(node, events)
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
                    warn!("tick {i} with data {data:?}, sending {random:#x}");
                    node.send_output(output.clone(), metadata.parameters, random.into_arrow())
                        .unwrap();
                }
                other => {
                    tracing::warn!(input = other, "Ignoring unexpected input");
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
