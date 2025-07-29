//! Demonstrates the most barebone usage of the Rerun SDK.

use std::{collections::HashMap, env::VarError, path::Path};

use dora_node_api::{
    arrow::{
        array::{Array, AsArray, Float32Array, StringArray, UInt8Array},
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
pub mod boxes3d;
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

    while let Some(event) = events.recv() {
        if let Event::Input { id, data, metadata } = event {
            let primitive = if let Some(Parameter::String(primitive)) = metadata.parameters.get("primitive") {
                primitive.clone()
            } else {
                bail!("No visualization primitive specified in metadata for input {}", id);
            };

            match primitive.as_str() {
                "image" => {
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
                }
                "depth" => {
                    let width =
                        if let Some(Parameter::Integer(width)) = metadata.parameters.get("width") {
                            *width as usize
                        } else {
                            640
                        };
                    let height =
                        if let Some(Parameter::Integer(height)) = metadata.parameters.get("height") {
                            *height as usize
                        } else {
                            480
                        };

                    // Check if we have camera metadata for pinhole camera setup
                    let has_camera_metadata = metadata.parameters.contains_key("camera_position")
                        && metadata.parameters.contains_key("camera_orientation")
                        && metadata.parameters.contains_key("focal");

                    if has_camera_metadata {
                        // Extract camera parameters
                        let focal_length = if let Some(Parameter::ListFloat(focals)) =
                            metadata.parameters.get("focal")
                        {
                            (focals[0] as f32, focals[1] as f32)
                        } else {
                            (605.0, 605.0)
                        };

                        let principal_point = if let Some(Parameter::ListFloat(pp)) =
                            metadata.parameters.get("principal_point")
                        {
                            (pp[0] as f32, pp[1] as f32)
                        } else {
                            (width as f32 / 2.0, height as f32 / 2.0)
                        };

                        let camera_position = if let Some(Parameter::ListFloat(pos)) =
                            metadata.parameters.get("camera_position")
                        {
                            rerun::Vec3D::new(pos[0] as f32, pos[1] as f32, pos[2] as f32)
                        } else {
                            rerun::Vec3D::new(0.0, 0.0, 0.0)
                        };

                        let camera_orientation = if let Some(Parameter::ListFloat(quat)) =
                            metadata.parameters.get("camera_orientation")
                        {
                            rerun::Quaternion::from_xyzw([
                                quat[0] as f32,
                                quat[1] as f32,
                                quat[2] as f32,
                                quat[3] as f32,
                            ])
                        } else {
                            rerun::Quaternion::from_xyzw([0.0, 0.0, 0.0, 1.0])
                        };

                        // Use the depth ID as parent entity for camera components
                        let camera_entity = id.as_str();

                        // Log camera transform
                        let camera_transform = rerun::Transform3D::from_translation_rotation(
                            camera_position,
                            camera_orientation,
                        );
                        rec.log(camera_entity, &camera_transform)
                            .context("could not log camera transform")?;

                        // Log pinhole camera
                        let pinhole = rerun::Pinhole::from_focal_length_and_resolution(
                            focal_length,
                            (width as f32, height as f32),
                        )
                        .with_camera_xyz(rerun::components::ViewCoordinates::RDF)
                        .with_resolution((width as f32, height as f32))
                        .with_principal_point(principal_point);

                        rec.log(camera_entity, &pinhole)
                            .context("could not log pinhole camera")?;

                        // Convert depth data to DepthImage
                        match data.data_type() {
                            dora_node_api::arrow::datatypes::DataType::Float32 => {
                                let buffer: &Float32Array = data.as_any().downcast_ref().unwrap();
                                let depth_values: Vec<f32> = buffer.values().to_vec();

                                let depth_image = rerun::external::ndarray::Array::from_shape_vec(
                                    (height, width),
                                    depth_values,
                                )
                                .context("Failed to create depth array")?;

                                // Log depth image as a child entity
                                let depth_entity = format!("{}/raw", camera_entity);
                                rec.log(
                                    depth_entity.as_str(),
                                    &rerun::DepthImage::try_from(depth_image)
                                        .context("Failed to create depth image")?
                                        .with_meter(1.0),
                                )
                                .context("could not log depth image")?;
                            }
                            _ => {
                                return Err(eyre!(
                                    "Depth data must be Float32Array, got {}. Please convert depth values to Float32 before sending.",
                                    data.data_type()
                                ));
                            }
                        }
                    } else {
                        // No camera metadata - just log a warning and skip 3D reconstruction
                        warn!("Depth data received without camera metadata (position, orientation, focal). Skipping 3D reconstruction.");
                        warn!("To enable proper 3D reconstruction, ensure the depth data includes camera_position, camera_orientation, and focal metadata.");
                    }
                }
                "text" => {
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
                }
                "boxes2d" => {
                    boxes2d::update_boxes2d(&rec, id, data, metadata).context("update boxes 2d")?;
                }
                "boxes3d" => {
                    boxes3d::update_boxes3d(&rec, id, data, metadata).context("update boxes 3d")?;
                }
                "masks" => {
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
                }
                "jointstate" => {
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
                    let mut urdf_id = id.as_str().replace("jointstate_", "");
                    urdf_id.push_str(".urdf");

                    if let Some(chain) = chains.get(&urdf_id) {
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

                        update_visualization(&rec, chain, &urdf_id, &positions)?;
                    } else {
                        println!("Could not find chain for {urdf_id}. You may not have set its");
                    }
                }
                "pose" => {
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
                    let mut urdf_id = id.as_str().replace("pose_", "");
                    urdf_id.push_str(".urdf");

                    if let Some(chain) = chains.get(&urdf_id) {
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

                        update_visualization(&rec, chain, &urdf_id, &positions)?;
                    } else {
                        println!("Could not find chain for {urdf_id}. You may not have set its");
                    }
                }
                "series" => {
                    update_series(&rec, id, data).context("could not plot series")?;
                }
                "points3d" => {
                    // Get color from metadata
                    let color = if let Some(Parameter::ListInt(rgb)) = metadata.parameters.get("color")
                    {
                        if rgb.len() >= 3 {
                            rerun::Color::from_rgb(rgb[0] as u8, rgb[1] as u8, rgb[2] as u8)
                        } else {
                            rerun::Color::from_rgb(128, 128, 128) // Default gray
                        }
                    } else {
                        rerun::Color::from_rgb(128, 128, 128) // Default gray
                    };

                    let dataid = id;

                    // Get radii from metadata as array
                    let radii = if let Some(Parameter::ListFloat(radii_list)) =
                        metadata.parameters.get("radii")
                    {
                        radii_list.iter().map(|&r| r as f32).collect::<Vec<f32>>()
                    } else {
                        vec![0.01] // Default 1cm radius
                    };

                    if let Ok(buffer) = into_vec::<f32>(&data) {
                        let mut points = vec![];
                        let mut colors = vec![];
                        let num_points = buffer.len() / 3;
                        buffer.chunks(3).for_each(|chunk| {
                            points.push((chunk[0], chunk[1], chunk[2]));
                            colors.push(color);
                        });

                        // Expand single radius to all points if needed
                        let radii_vec = if radii.len() == num_points {
                            radii
                        } else if radii.len() == 1 {
                            vec![radii[0]; num_points]
                        } else {
                            vec![0.01; num_points] // Default 1cm radius
                        };

                        let points = Points3D::new(points).with_radii(radii_vec);

                        rec.log(dataid.as_str(), &points.with_colors(colors))
                            .context("could not log points")?;
                    }
                }
                "points2d" => {
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
                }
                "lines3d" => {
                    // Get color from metadata
                    let color = if let Some(Parameter::ListInt(rgb)) = metadata.parameters.get("color")
                    {
                        if rgb.len() >= 3 {
                            rerun::Color::from_rgb(rgb[0] as u8, rgb[1] as u8, rgb[2] as u8)
                        } else {
                            rerun::Color::from_rgb(0, 255, 0) // Default green
                        }
                    } else {
                        rerun::Color::from_rgb(0, 255, 0) // Default green
                    };

                    // Get radius for line thickness
                    let radius = if let Some(Parameter::Float(r)) = metadata.parameters.get("radius") {
                        *r as f32
                    } else {
                        0.01 // Default radius
                    };

                    if let Ok(buffer) = into_vec::<f32>(&data) {
                        let mut line_points = vec![];
                        buffer.chunks(3).for_each(|chunk| {
                            line_points.push((chunk[0], chunk[1], chunk[2]));
                        });

                        rec.log(
                            id.as_str(),
                            &rerun::LineStrips3D::new([line_points])
                                .with_colors([color])
                                .with_radii([radius]),
                        )
                        .context("could not log line strips")?;
                    }
                }
                _ => bail!("Unknown visualization primitive: {}", primitive),
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
