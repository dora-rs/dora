//! Demonstrates the most barebone usage of the Rerun SDK.

use dora_node_api::{
    arrow::{
        array::{PrimitiveArray, StringArray, UInt8Array},
        datatypes::{UInt8Type, Utf8Type},
        ipc::Utf8,
    },
    DoraNode, Event,
};
use eyre::{Context, Result};
use rerun::{
    external::{arrow2::array::Utf8Array, re_types::ArrowBuffer},
    TensorBuffer, TensorData, TensorDimension,
};

fn main() -> Result<()> {
    // `serve()` requires to have a running Tokio runtime in the current context.
    let rt = tokio::runtime::Runtime::new().expect("Failed to create tokio runtime");
    let _guard = rt.enter();

    let (_node, mut events) =
        DoraNode::init_from_env().context("Could not initialize dora node")?;

    let rec = rerun::RecordingStreamBuilder::new("dora-rerun")
        .spawn()
        .context("Could not spawn rerun visualization")?;

    let shape = vec![
        TensorDimension {
            name: Some("width".into()),
            size: std::env::var("IMAGE_WIDTH")
                .context("Could not read image width")?
                .parse()
                .context("Could not parse value of image width env variable")?,
        },
        TensorDimension {
            name: Some("height".into()),
            size: std::env::var("IMAGE_HEIGHT")
                .context("Could not read image height")?
                .parse()
                .context("Could not parse value of image height env variable")?,
        },
        TensorDimension {
            name: Some("depth".into()),
            size: std::env::var("IMAGE_DEPTH")
                .context("Could not read image depth")?
                .parse()
                .context("Could not parse value of image depth env variable")?,
        },
    ];

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                data,
                metadata: _,
            } => {
                if id.as_str().contains("image") {
                    let buffer: UInt8Array = data.to_data().into();
                    let buffer: &[u8] = buffer.values();
                    let buffer = TensorBuffer::U8(ArrowBuffer::from(buffer));
                    let tensordata = TensorData::new(shape.clone(), buffer.into());
                    let image = rerun::Image::new(tensordata);

                    rec.log(id.as_str(), &image)
                        .context("could not log image")?;
                } else if id.as_str().contains("text") {
                    let buffer: StringArray = data.to_data().into();
                    buffer.iter().try_for_each(|string| -> Result<()> {
                        if let Some(str) = string {
                            rec.log(id.as_str(), &rerun::TextLog::new(str))
                                .wrap_err("Could not log text")
                        } else {
                            Ok(())
                        }
                    })?;
                }
            }
            _ => {}
        }
    }

    Ok(())
}
