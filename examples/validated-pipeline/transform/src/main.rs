use adora_node_api::{AdoraNode, Event, IntoArrow, adora_core::config::DataId, arrow};
use eyre::{ContextCompat, bail};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;
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
