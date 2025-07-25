//! Demonstrates the most barebone usage of the Rerun SDK.

use std::{collections::HashMap, env::VarError, path::Path};

use dora_node_api::{
    arrow::{
        array::{Array, AsArray, Float64Array, StringArray, UInt16Array, UInt8Array},
        datatypes::Float32Type,
    },
    dora_core::config::DataId,
    into_vec, DoraNode, Event, Parameter,
};
use eyre::{bail, eyre, Context, Result};

use pinyin::ToPinyin;
use rerun::{
    components::ImageBuffer, external::log::warn, ImageFormat, Points2D, Points3D, SpawnOptions,
};
pub mod boxes2d;
pub mod series;
pub mod urdf;
use series::update_series;
use urdf::{init_urdf, update_visualization};

static KEYS: &[&str] = &[
    "image",
    "depth",
    "text",
    "boxes2d",
    "masks",
    "jointstate",
    "pose",
    "series",
    "points3d",
    "points2d",
];

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
    let mut color_cache: HashMap<DataId, rerun::Color> = HashMap::new();
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
                .connect_grpc_opts(opt, None)
                .context("Could not connect to rerun visualization")?
        }
        Ok("SAVE") => {
            let id = node.dataflow_id();
            let path = Path::new("out")
                .join(id.to_string())
                .join(format!("archive-{id}.rerun"));

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
    let camera_pitch = std::env::var("CAMERA_PITCH")
        .unwrap_or("0.0".to_string())
        .parse::<f32>()
        .unwrap();

    while let Some(event) = events.recv() {
        if let Event::Input { id, data, metadata } = event {
            // Check if the id contains more than one key
            if KEYS
                .iter()
                .filter(|&&key| id.as_str().contains(key))
                .count()
                > 1
            {
                bail!(
                    "Event id `{}` contains more than one visualization keyword: {:?}, please only use one of them.",
                    id,
                    KEYS.iter()
                        .filter(|&&key| id.as_str().contains(key))
                        .collect::<Vec<_>>()
                );
            }

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
                    let image_buffer = ImageBuffer::from(buffer);
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
                    let image_buffer = ImageBuffer::from(buffer);

                    let image = rerun::Image::new(
                        image_buffer,
                        ImageFormat::rgb8([*width as u32, *height as u32]),
                    );
                    rec.log(id.as_str(), &image)
                        .context("could not log image")?;
                } else if ["jpeg", "png", "avif"].contains(&encoding) {
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
                let pitch = if let Some(Parameter::Float(pitch)) = metadata.parameters.get("pitch")
                {
                    *pitch as f32
                } else {
                    camera_pitch
                };
                let cos_theta = pitch.cos();
                let sin_theta = pitch.sin();

                let points = match data.data_type() {
                    dora_node_api::arrow::datatypes::DataType::Float64 => {
                        let buffer: &Float64Array = data.as_any().downcast_ref().unwrap();

                        let mut points = vec![];
                        buffer.iter().enumerate().for_each(|(i, z)| {
                            let u = i as f32 % *width as f32; // Calculate x-coordinate (u)
                            let v = i as f32 / *width as f32; // Calculate y-coordinate (v)

                            if let Some(z) = z {
                                let z = z as f32;
                                // Skip points that have empty depth or is too far away
                                if z == 0. || z > 8.0 {
                                    points.push((0., 0., 0.));
                                    return;
                                }
                                let y = (u - resolution[0] as f32) * z / focal_length[0] as f32;
                                let x = (v - resolution[1] as f32) * z / focal_length[1] as f32;
                                let new_x = sin_theta * z + cos_theta * x;
                                let new_y = -y;
                                let new_z = cos_theta * z - sin_theta * x;

                                points.push((new_x, new_y, new_z));
                            } else {
                                points.push((0., 0., 0.));
                            }
                        });
                        Points3D::new(points)
                    }
                    dora_node_api::arrow::datatypes::DataType::UInt16 => {
                        let buffer: &UInt16Array = data.as_any().downcast_ref().unwrap();
                        let mut points = vec![];
                        buffer.iter().enumerate().for_each(|(i, z)| {
                            let u = i as f32 % *width as f32; // Calculate x-coordinate (u)
                            let v = i as f32 / *width as f32; // Calculate y-coordinate (v)

                            if let Some(z) = z {
                                let z = z as f32 / 1000.0; // Convert to meters
                                                           // Skip points that have empty depth or is too far away
                                if z == 0. || z > 8.0 {
                                    points.push((0., 0., 0.));
                                    return;
                                }
                                let y = (u - resolution[0] as f32) * z / focal_length[0] as f32;
                                let x = (v - resolution[1] as f32) * z / focal_length[1] as f32;
                                let new_x = sin_theta * z + cos_theta * x;
                                let new_y = -y;
                                let new_z = cos_theta * z - sin_theta * x;

                                points.push((new_x, new_y, new_z));
                            } else {
                                points.push((0., 0., 0.));
                            }
                        });
                        Points3D::new(points)
                    }
                    _ => {
                        return Err(eyre!("Unsupported depth data type {}", data.data_type()));
                    }
                };
                if let Some(color_buffer) = image_cache.get(&id.replace("depth", "image")) {
                    let colors = if let Some(mask) = mask_cache.get(&id.replace("depth", "masks")) {
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
                    rec.log(id.as_str(), &points.with_colors(colors))
                        .context("could not log points")?;
                }
            } else if id.as_str().contains("text") {
                let buffer: StringArray = data.to_data().into();
                buffer.iter().try_for_each(|string| -> Result<()> {
                    if let Some(str) = string {
                        let chars = str.chars().collect::<Vec<_>>();
                        let mut new_string = vec![];
                        for char in chars {
                            // Check if the character is a Chinese character
                            if char.is_ascii() || char.is_control() {
                                new_string.push(char);
                                continue;
                            }
                            // If it is a Chinese character, replace it with its pinyin
                            if let Some(pinyin) = char.to_pinyin() {
                                for char in pinyin.with_tone().chars() {
                                    new_string.push(char);
                                }
                                new_string.push(' ');
                            }
                        }
                        let pinyined_str = new_string.iter().collect::<String>();
                        rec.log(id.as_str(), &rerun::TextLog::new(pinyined_str))
                            .wrap_err("Could not log text")
                    } else {
                        Ok(())
                    }
                })?;
            } else if id.as_str().contains("boxes2d") {
                boxes2d::update_boxes2d(&rec, id, data, metadata).context("update boxes 2d")?;
            } else if id.as_str().contains("masks") {
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
            } else if id.as_str().contains("jointstate") || id.as_str().contains("pose") {
                let encoding = if let Some(Parameter::String(encoding)) =
                    metadata.parameters.get("encoding")
                {
                    encoding
                } else {
                    "jointstate"
                };
                if encoding != "jointstate" {
                    warn!("Got unexpected encoding: {encoding} on position pose");
                    continue;
                }
                // Convert to Vec<f32>
                let mut positions: Vec<f32> =
                    into_vec(&data).context("Could not parse jointstate as vec32")?;

                // Match file name
                let mut id = id.as_str().replace("jointstate_", "");
                id.push_str(".urdf");

                if let Some(chain) = chains.get(&id) {
                    let dof = chain.dof();

                    // Truncate or pad positions to match the chain's dof
                    if dof < positions.len() {
                        positions.truncate(dof);
                    } else {
                        #[allow(clippy::same_item_push)]
                        for _ in 0..(dof - positions.len()) {
                            positions.push(0.);
                        }
                    }

                    update_visualization(&rec, chain, &id, &positions)?;
                } else {
                    println!("Could not find chain for {id}. You may not have set its");
                }
            } else if id.as_str().contains("series") {
                update_series(&rec, id, data).context("could not plot series")?;
            } else if id.as_str().contains("points3d") {
                // Get color or assign random color in cache
                let color = color_cache.get(&id);
                let color = if let Some(color) = color {
                    *color
                } else {
                    let color =
                        rerun::Color::from_rgb(rand::random::<u8>(), 180, rand::random::<u8>());

                    color_cache.insert(id.clone(), color);
                    color
                };
                let dataid = id;

                // get a random color
                if let Ok(buffer) = into_vec::<f32>(&data) {
                    let mut points = vec![];
                    let mut colors = vec![];
                    buffer.chunks(3).for_each(|chunk| {
                        points.push((chunk[0], chunk[1], chunk[2]));
                        colors.push(color);
                    });
                    let points = Points3D::new(points).with_radii(vec![0.013; colors.len()]);

                    rec.log(dataid.as_str(), &points.with_colors(colors))
                        .context("could not log points")?;
                }
            } else if id.as_str().contains("points2d") {
                // Get color or assign random color in cache
                let color = color_cache.get(&id);
                let color = if let Some(color) = color {
                    *color
                } else {
                    let color =
                        rerun::Color::from_rgb(rand::random::<u8>(), 180, rand::random::<u8>());

                    color_cache.insert(id.clone(), color);
                    color
                };
                let dataid = id;

                // get a random color
                if let Ok(buffer) = into_vec::<f32>(&data) {
                    let mut points = vec![];
                    let mut colors = vec![];
                    buffer.chunks(2).for_each(|chunk| {
                        points.push((chunk[0], chunk[1]));
                        colors.push(color);
                    });
                    let points = Points2D::new(points);

                    rec.log(dataid.as_str(), &points.with_colors(colors))
                        .context("could not log points")?;
                }
            } else {
                println!("Could not find handler for {id}");
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
