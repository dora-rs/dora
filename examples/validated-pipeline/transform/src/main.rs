use dora_node_api::{DoraNode, Event, EventStream, IntoArrow, arrow, dora_core::config::DataId};
use eyre::{ContextCompat, bail};

#[cfg(test)]
mod tests;

fn main() -> eyre::Result<()> {
    let (node, events) = DoraNode::init_from_env()?;
    run(node, events)
}

fn run(mut node: DoraNode, mut events: EventStream) -> eyre::Result<()> {
    let output = DataId::from("doubled".to_owned());

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id, data, metadata, ..
            } if id.as_str() == "value" => {
                let arr = data
                    .as_any()
                    .downcast_ref::<arrow::array::Int64Array>()
                    .context("expected Int64Array")?;
                if arr.is_empty() {
                    bail!("received empty array");
                }
                let doubled = arr.value(0) * 2;
                eprintln!("transform: {} -> {doubled}", arr.value(0));
                node.send_output(output.clone(), metadata.parameters, doubled.into_arrow())?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
