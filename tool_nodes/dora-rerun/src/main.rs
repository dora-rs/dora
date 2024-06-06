//! Demonstrates the most barebone usage of the Rerun SDK.

use std::env::VarError;

use dora_node_api::{
    arrow::array::{Float32Array, StringArray, UInt8Array},
    DoraNode, Event,
};
use eyre::{eyre, Context, Result};
use rerun::{
    external::re_types::ArrowBuffer, SpawnOptions, TensorBuffer, TensorData, TensorDimension,
};

fn main() -> Result<()> {
    // rerun `serve()` requires to have a running Tokio runtime in the current context.
    let rt = tokio::runtime::Runtime::new().expect("Failed to create tokio runtime");
    let _guard = rt.enter();

    let (_node, mut events) =
        DoraNode::init_from_env().context("Could not initialize dora node")?;

    // Limit memory usage
    let mut options = SpawnOptions::default();

    let memory_limit = match std::env::var("RERUN_MEMORY_LIMIT") {
        Ok(memory_limit) => memory_limit
            .parse::<String>()
            .context("Could not parse RERUN_MEMORY_LIMIT value")?,
        Err(VarError::NotUnicode(_)) => {
            return Err(eyre!("RERUN_MEMORY_LIMIT env variable is not unicode"));
        }
        Err(VarError::NotPresent) => "25%".to_string(),
    };

    options.memory_limit = memory_limit;

    let rec = rerun::RecordingStreamBuilder::new("dora-rerun")
        .spawn_opts(&options, None)
        .context("Could not spawn rerun visualization")?;

    while let Some(event) = events.recv() {
        if let Event::Input {
            id,
            data,
            metadata: _,
            ..
        } = event
        {
            if id.as_str().contains("image") {
                let shape = vec![
                    TensorDimension {
                        name: Some("height".into()),
                        size: std::env::var(format!("{}_HEIGHT", id.as_str().to_uppercase()))
                            .context(format!(
                                "Could not read {}_HEIGHT env variable for parsing the image",
                                id.as_str().to_uppercase()
                            ))?
                            .parse()
                            .context(format!(
                                "Could not parse env {}_HEIGHT",
                                id.as_str().to_uppercase()
                            ))?,
                    },
                    TensorDimension {
                        name: Some("width".into()),
                        size: std::env::var(format!("{}_WIDTH", id.as_str().to_uppercase()))
                            .context(format!(
                                "Could not read {}_WIDTH env variable for parsing the image",
                                id.as_str().to_uppercase()
                            ))?
                            .parse()
                            .context(format!(
                                "Could not parse env {}_WIDTH",
                                id.as_str().to_uppercase()
                            ))?,
                    },
                    TensorDimension {
                        name: Some("depth".into()),
                        size: std::env::var(format!("{}_DEPTH", id.as_str().to_uppercase()))
                            .context(format!(
                                "Could not read {}_DEPTH env variable for parsing the image",
                                id.as_str().to_uppercase()
                            ))?
                            .parse()
                            .context(format!(
                                "Could not parse env {}_DEPTH",
                                id.as_str().to_uppercase()
                            ))?,
                    },
                ];

                let buffer: UInt8Array = data.to_data().into();
                let buffer: &[u8] = buffer.values();
                let buffer = TensorBuffer::U8(ArrowBuffer::from(buffer));
                let tensordata = TensorData::new(shape.clone(), buffer);
                let image = rerun::Image::new(tensordata);

                rec.log(id.as_str(), &image)
                    .context("could not log image")?;
            } else if id.as_str().contains("textlog") {
                let buffer: StringArray = data.to_data().into();
                buffer.iter().try_for_each(|string| -> Result<()> {
                    if let Some(str) = string {
                        rec.log(id.as_str(), &rerun::TextLog::new(str))
                            .wrap_err("Could not log text")
                    } else {
                        Ok(())
                    }
                })?;
            } else if id.as_str().contains("boxes2d") {
                let buffer: Float32Array = data.to_data().into();
                let buffer: &[f32] = buffer.values();
                let mut centers = vec![];
                let mut sizes = vec![];
                let mut classes = vec![];
                buffer.chunks(6).for_each(|block| {
                    if let [x, y, w, h, _conf, cls] = block {
                        centers.push((*x, *y));
                        sizes.push((*w, *h));
                        classes.push(*cls as u16);
                    }
                });
                rec.log(
                    id.as_str(),
                    &rerun::Boxes2D::from_centers_and_sizes(centers, sizes).with_class_ids(classes),
                )
                .wrap_err("Could not log Boxes2D")?;
            }
        }
    }

    Ok(())
}
