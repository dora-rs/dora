//! Demonstrates the most barebone usage of the Rerun SDK.

use std::{collections::HashMap, env::VarError, path::Path};

use dora_node_api::{
    arrow::{
        array::{Array, AsArray, Float32Array, Float64Array, StringArray, UInt8Array},
        datatypes::Float32Type,
    },
    dora_core::config::DataId,
    DoraNode, Event, Parameter,
};
use eyre::{eyre, Context, ContextCompat, Result};

use rerun::{
    components::ImageBuffer,
    external::{log::warn, re_types::ArrowBuffer},
    ImageFormat, Points3D, SpawnOptions,
};
pub mod boxes2d;
pub mod series;
pub mod urdf;
use series::update_series;
use urdf::{init_urdf, update_visualization};

pub fn lib_main() -> Result<()> {
    // rerun `serve()` requires to have a running Tokio runtime in the current context.
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .expect("Failed to create tokio runtime");
    let _guard = rt.enter();
    let (node, mut events) = DoraNode::init_from_env().context("Could not initialize dora node")?;

    // Setup an image cache to paint depth images.
    let mut image_cache = HashMap::new();
    let mut mask_cache: HashMap<DataId, Vec<bool>> = HashMap::new();
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

    let rec = match std::env::var("OPERATING_MODE").as_deref() {
        Ok("SPAWN") => rerun::RecordingStreamBuilder::new("dora-rerun")
            .spawn_opts(&options, None)
            .context("Could not spawn rerun visualization")?,
        Ok("CONNECT") => {
            let opt = std::env::var("RERUN_SERVER_ADDR").unwrap_or("127.0.0.1:9876".to_string());

            rerun::RecordingStreamBuilder::new("dora-rerun")
                .connect_tcp_opts(std::net::SocketAddr::V4(opt.parse()?), None)
                .context("Could not connect to rerun visualization")?
        }
        Ok("SAVE") => {
            let id = node.dataflow_id();
            let path = Path::new("out")
                .join(id.to_string())
                .join(format!("archive-{}.rerun", id));

            rerun::RecordingStreamBuilder::new("dora-rerun")
                .save(path)
                .context("Could not save rerun visualization")?
        }
        Ok(_) => {
            warn!("Invalid operating mode, defaulting to SPAWN mode.");

            rerun::RecordingStreamBuilder::new("dora-rerun")
                .spawn_opts(&options, None)
                .context("Could not spawn rerun visualization")?
        }
        Err(_) => rerun::RecordingStreamBuilder::new("dora-rerun")
            .spawn_opts(&options, None)
            .context("Could not spawn rerun visualization")?,
    };

    let chains = init_urdf(&rec).context("Could not load urdf")?;

    match std::env::var("README") {
        Ok(readme) => {
            readme
                .parse::<String>()
                .context("Could not parse readme value")?;
            rec.log("README", &rerun::TextDocument::new(readme))
                .wrap_err("Could not log text")?;
        }
        Err(VarError::NotUnicode(_)) => {
            return Err(eyre!("readme env variable is not unicode"));
        }
        Err(VarError::NotPresent) => (),
    };

    while let Some(event) = events.recv() {
        if let Event::Input { id, data, metadata } = event {
            if id.as_str().contains("image") {
                let height =
                    if let Some(Parameter::Integer(height)) = metadata.parameters.get("height") {
                        height
                    } else {
                        &480
                    };
                let width =
                    if let Some(Parameter::Integer(width)) = metadata.parameters.get("width") {
                        width
                    } else {
                        &640
                    };
                let encoding = if let Some(Parameter::String(encoding)) =
                    metadata.parameters.get("encoding")
                {
                    encoding
                } else {
                    "bgr8"
                };

                if encoding == "bgr8" {
                    let buffer: &UInt8Array = data.as_any().downcast_ref().unwrap();
                    let buffer: &[u8] = buffer.values();

                    // Transpose values from BGR to RGB
                    let buffer: Vec<u8> =
                        buffer.chunks(3).flat_map(|x| [x[2], x[1], x[0]]).collect();
                    image_cache.insert(id.clone(), buffer.clone());
                    let buffer = ArrowBuffer::from(buffer);
                    let image_buffer = ImageBuffer::try_from(buffer)
                        .context("Could not convert buffer to image buffer")?;
                    // let tensordata = ImageBuffer(buffer);

                    let image = rerun::Image::new(
                        image_buffer,
                        ImageFormat::rgb8([*width as u32, *height as u32]),
                    );
                    rec.log(id.as_str(), &image)
                        .context("could not log image")?;
                } else if encoding == "rgb8" {
                    let buffer: &UInt8Array = data.as_any().downcast_ref().unwrap();
                    image_cache.insert(id.clone(), buffer.values().to_vec());
                    let buffer: &[u8] = buffer.values();
                    let buffer = ArrowBuffer::from(buffer);
                    let image_buffer = ImageBuffer::try_from(buffer)
                        .context("Could not convert buffer to image buffer")?;

                    let image = rerun::Image::new(
                        image_buffer,
                        ImageFormat::rgb8([*width as u32, *height as u32]),
                    );
                    rec.log(id.as_str(), &image)
                        .context("could not log image")?;
                } else if ["jpeg", "png"].contains(&encoding) {
                    let buffer: &UInt8Array = data.as_any().downcast_ref().unwrap();
                    let buffer: &[u8] = buffer.values();

                    let image = rerun::EncodedImage::from_file_contents(buffer.to_vec());
                    rec.log(id.as_str(), &image)
                        .context("could not log image")?;
                };
            } else if id.as_str().contains("depth") {
                let width =
                    if let Some(Parameter::Integer(width)) = metadata.parameters.get("width") {
                        width
                    } else {
                        &640
                    };
                let focal_length =
                    if let Some(Parameter::ListInt(focals)) = metadata.parameters.get("focal") {
                        focals.to_vec()
                    } else {
                        vec![605, 605]
                    };
                let resolution = if let Some(Parameter::ListInt(resolution)) =
                    metadata.parameters.get("resolution")
                {
                    resolution.to_vec()
                } else {
                    vec![640, 480]
                };
                let buffer: &Float64Array = data.as_any().downcast_ref().unwrap();
                let points_3d = buffer.iter().enumerate().map(|(i, z)| {
                    let u = i as f32 % *width as f32; // Calculate x-coordinate (u)
                    let v = i as f32 / *width as f32; // Calculate y-coordinate (v)
                    let z = z.unwrap_or_default() as f32;

                    (
                        (u - resolution[0] as f32) * z / focal_length[0] as f32,
                        (v - resolution[1] as f32) * z / focal_length[1] as f32,
                        z,
                    )
                });
                let points_3d = Points3D::new(points_3d);
                if let Some(color_buffer) = image_cache.get(&id.replace("depth", "image")) {
                    let colors = if let Some(mask) = mask_cache.get(&id.replace("depth", "mask")) {
                        let mask_length = color_buffer.len() / 3;
                        let number_masks = mask.len() / mask_length;
                        color_buffer
                            .chunks(3)
                            .enumerate()
                            .map(|(e, x)| {
                                for i in 0..number_masks {
                                    if mask[i * mask_length + e] && (e % 3 == 0) {
                                        if i == 0 {
                                            return rerun::Color::from_rgb(255, x[1], x[2]);
                                        } else if i == 1 {
                                            return rerun::Color::from_rgb(x[0], 255, x[2]);
                                        } else if i == 2 {
                                            return rerun::Color::from_rgb(x[0], x[1], 255);
                                        } else {
                                            return rerun::Color::from_rgb(x[0], 255, x[2]);
                                        }
                                    }
                                }
                                rerun::Color::from_rgb(x[0], x[1], x[2])
                            })
                            .collect::<Vec<_>>()
                    } else {
                        color_buffer
                            .chunks(3)
                            .map(|x| rerun::Color::from_rgb(x[0], x[1], x[2]))
                            .collect::<Vec<_>>()
                    };
                    rec.log(id.as_str(), &points_3d.with_colors(colors))
                        .context("could not log points")?;
                } else {
                    rec.log(id.as_str(), &points_3d)
                        .context("could not log points")?;
                }
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
            } else if id.as_str().contains("boxes2d") {
                boxes2d::update_boxes2d(&rec, id, data, metadata).context("update boxes 2d")?;
            } else if id.as_str().contains("mask") {
                let masks = if let Some(data) = data.as_primitive_opt::<Float32Type>() {
                    let data = data
                        .iter()
                        .map(|x| if let Some(x) = x { x > 0. } else { false })
                        .collect::<Vec<_>>();
                    data
                } else if let Some(data) = data.as_boolean_opt() {
                    let data = data
                        .iter()
                        .map(|x| x.unwrap_or_default())
                        .collect::<Vec<_>>();
                    data
                } else {
                    println!("Got unexpected data type: {}", data.data_type());
                    continue;
                };
                mask_cache.insert(id.clone(), masks.clone());
            } else if id.as_str().contains("jointstate") {
                let buffer: &Float32Array = data
                    .as_any()
                    .downcast_ref()
                    .context("jointstate is not float32")?;
                let mut positions: Vec<f32> = buffer.values().to_vec();

                // Match file name
                let mut id = id.as_str().replace("jointstate_", "");
                id.push_str(".urdf");

                if let Some(chain) = chains.get(&id) {
                    let dof = chain.dof();

                    // Truncate or pad positions to match the chain's dof
                    if dof < positions.len() {
                        positions.truncate(dof);
                    } else {
                        for _ in 0..(dof - positions.len()) {
                            positions.push(0.);
                        }
                    }

                    update_visualization(&rec, chain, &id, &positions)?;
                } else {
                    println!("Could not find chain for {}", id);
                }
            } else if id.as_str().contains("series") {
                update_series(&rec, id, data).context("could not plot series")?;
            } else {
                println!("Could not find handler for {}", id);
            }
        }
    }

    Ok(())
}

#[cfg(feature = "python")]
use pyo3::{
    pyfunction, pymodule,
    types::{PyModule, PyModuleMethods},
    wrap_pyfunction, Bound, PyResult, Python,
};

#[cfg(feature = "python")]
#[pyfunction]
fn py_main(_py: Python) -> eyre::Result<()> {
    lib_main()
}

/// A Python module implemented in Rust.
#[cfg(feature = "python")]
#[pymodule]
fn dora_rerun(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
