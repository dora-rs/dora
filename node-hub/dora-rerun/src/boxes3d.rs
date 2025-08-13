use dora_node_api::{
    ArrowData, Metadata, Parameter,
    arrow::{
        array::AsArray,
        datatypes::{
            DataType, Float16Type, Float32Type, Float64Type, Int16Type, Int32Type, Int64Type,
        },
    },
    dora_core::config::DataId,
};
use eyre::{Context, ContextCompat, Result};
use rerun::{RecordingStream, Text};

pub fn update_boxes3d(
    rec: &RecordingStream,
    id: DataId,
    data: ArrowData,
    metadata: Metadata,
) -> Result<()> {
    let format = if let Some(Parameter::String(format)) = metadata.parameters.get("format") {
        format
    } else {
        "center_half_size" // Default format
    };

    // Check if user wants solid fill mode (default is wireframe)
    let solid = matches!(
        metadata.parameters.get("solid"),
        Some(Parameter::Bool(true))
    );

    if let Some(bbox_struct) = data.as_struct_opt() {
        // Cast Bbox
        let bbox_buffer = bbox_struct
            .column_by_name("bbox")
            .context("Did not find bbox field within bbox struct")?;
        let bbox = bbox_buffer
            .as_list_opt::<i32>()
            .context("Could not deserialize bbox as list")?
            .values();
        let bbox: Vec<f32> = match bbox.data_type() {
            DataType::Float16 => bbox
                .as_primitive_opt::<Float16Type>()
                .context("Failed to deserialize bbox")?
                .values()
                .iter()
                .map(|x| f32::from(*x))
                .collect(),
            DataType::Float32 => bbox
                .as_primitive_opt::<Float32Type>()
                .context("Failed to deserialize bbox")?
                .values()
                .to_vec(),
            DataType::Float64 => bbox
                .as_primitive_opt::<Float64Type>()
                .context("Failed to deserialize bbox")?
                .values()
                .iter()
                .map(|x| *x as f32)
                .collect(),
            DataType::Int16 => bbox
                .as_primitive_opt::<Int16Type>()
                .context("Failed to deserialize bbox")?
                .values()
                .iter()
                .map(|x| *x as f32)
                .collect(),
            DataType::Int32 => bbox
                .as_primitive_opt::<Int32Type>()
                .context("Failed to deserialize bbox")?
                .values()
                .iter()
                .map(|x| *x as f32)
                .collect(),
            DataType::Int64 => bbox
                .as_primitive_opt::<Int64Type>()
                .context("Failed to deserialize bbox")?
                .values()
                .iter()
                .map(|x| *x as f32)
                .collect(),
            _ => {
                return Err(eyre::eyre!(
                    "Could not deserialize bbox as float32, float64, int32 or int64"
                ));
            }
        };

        if bbox.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes3D")?;
            return Ok(());
        }

        // Cast Labels (optional)
        let labels = bbox_struct
            .column_by_name("labels")
            .and_then(|labels_buffer| {
                labels_buffer
                    .as_list_opt::<i32>()
                    .and_then(|list| list.values().as_string_opt::<i32>())
                    .map(|labels| {
                        labels
                            .iter()
                            .map(|x| Text::from(x.unwrap()))
                            .collect::<Vec<_>>()
                    })
            });

        // Cast Colors (optional)
        let colors = bbox_struct
            .column_by_name("colors")
            .and_then(|colors_buffer| {
                colors_buffer
                    .as_list_opt::<i32>()
                    .and_then(|list| list.values().as_primitive_opt::<Int32Type>())
                    .map(|colors| {
                        colors
                            .values()
                            .chunks(3)
                            .map(|rgb| {
                                rerun::Color::from_rgb(rgb[0] as u8, rgb[1] as u8, rgb[2] as u8)
                            })
                            .collect::<Vec<_>>()
                    })
            });

        let (centers, half_sizes) = parse_boxes_3d(&bbox, format)?;

        let mut boxes = rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes);

        if solid {
            boxes = boxes.with_fill_mode(rerun::FillMode::Solid);
        }

        if let Some(labels) = labels {
            boxes = boxes.with_labels(labels);
        }

        if let Some(colors) = colors {
            boxes = boxes.with_colors(colors);
        }

        rec.log(id.as_str(), &boxes)
            .wrap_err("Could not log Boxes3D")?;
    } else if let Some(buffer_array) = data.as_primitive_opt::<Float32Type>() {
        let values = buffer_array.values();
        let (centers, half_sizes) = parse_boxes_3d(values, format)?;

        if values.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes3D")?;
            return Ok(());
        } else {
            let mut boxes = rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes);

            if solid {
                boxes = boxes.with_fill_mode(rerun::FillMode::Solid);
            }

            // Support single color from metadata
            if let Some(Parameter::ListInt(rgb)) = metadata.parameters.get("color") {
                if rgb.len() >= 3 {
                    let color = rerun::Color::from_rgb(rgb[0] as u8, rgb[1] as u8, rgb[2] as u8);
                    boxes = boxes.with_colors([color]);
                }
            }

            rec.log(id.as_str(), &boxes)
                .wrap_err("Could not log Boxes3D")?;
        }
    } else if let Some(buffer_array) = data.as_primitive_opt::<Float64Type>() {
        let values: Vec<f32> = buffer_array.values().iter().map(|x| *x as f32).collect();
        let (centers, half_sizes) = parse_boxes_3d(&values, format)?;

        if values.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes3D")?;
            return Ok(());
        } else {
            let mut boxes = rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes);

            if solid {
                boxes = boxes.with_fill_mode(rerun::FillMode::Solid);
            }

            // Support single color from metadata
            if let Some(Parameter::ListInt(rgb)) = metadata.parameters.get("color") {
                if rgb.len() >= 3 {
                    let color = rerun::Color::from_rgb(rgb[0] as u8, rgb[1] as u8, rgb[2] as u8);
                    boxes = boxes.with_colors([color]);
                }
            }

            rec.log(id.as_str(), &boxes)
                .wrap_err("Could not log Boxes3D")?;
        }
    } else if let Some(buffer_array) = data.as_primitive_opt::<Int64Type>() {
        let values: Vec<f32> = buffer_array.values().iter().map(|x| *x as f32).collect();
        let (centers, half_sizes) = parse_boxes_3d(&values, format)?;

        if values.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes3D")?;
            return Ok(());
        } else {
            let boxes = rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes);
            let boxes = if solid {
                boxes.with_fill_mode(rerun::FillMode::Solid)
            } else {
                boxes
            };
            rec.log(id.as_str(), &boxes)
                .wrap_err("Could not log Boxes3D")?;
        }
    } else if let Some(buffer_array) = data.as_primitive_opt::<Int32Type>() {
        let values: Vec<f32> = buffer_array.values().iter().map(|x| *x as f32).collect();
        let (centers, half_sizes) = parse_boxes_3d(&values, format)?;

        if values.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes3D")?;
            return Ok(());
        } else {
            let boxes = rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes);
            let boxes = if solid {
                boxes.with_fill_mode(rerun::FillMode::Solid)
            } else {
                boxes
            };
            rec.log(id.as_str(), &boxes)
                .wrap_err("Could not log Boxes3D")?;
        }
    }
    Ok(())
}

fn parse_boxes_3d(
    values: &[f32],
    format: &str,
) -> Result<(
    Vec<rerun::external::glam::Vec3>,
    Vec<rerun::external::glam::Vec3>,
)> {
    let mut centers = vec![];
    let mut half_sizes = vec![];

    match format {
        "min_max" => {
            // Format: [min_x, min_y, min_z, max_x, max_y, max_z]
            values.chunks(6).for_each(|block| {
                if let [min_x, min_y, min_z, max_x, max_y, max_z] = block {
                    let center = rerun::external::glam::Vec3::new(
                        (min_x + max_x) / 2.0,
                        (min_y + max_y) / 2.0,
                        (min_z + max_z) / 2.0,
                    );
                    let half_size = rerun::external::glam::Vec3::new(
                        (max_x - min_x) / 2.0,
                        (max_y - min_y) / 2.0,
                        (max_z - min_z) / 2.0,
                    );
                    centers.push(center);
                    half_sizes.push(half_size);
                }
            });
        }
        "center_size" => {
            // Format: [center_x, center_y, center_z, size_x, size_y, size_z]
            values.chunks(6).for_each(|block| {
                if let [cx, cy, cz, sx, sy, sz] = block {
                    let center = rerun::external::glam::Vec3::new(*cx, *cy, *cz);
                    let half_size = rerun::external::glam::Vec3::new(sx / 2.0, sy / 2.0, sz / 2.0);
                    centers.push(center);
                    half_sizes.push(half_size);
                }
            });
        }
        "center_half_size" => {
            // Format: [center_x, center_y, center_z, half_size_x, half_size_y, half_size_z]
            values.chunks(6).for_each(|block| {
                if let [cx, cy, cz, hx, hy, hz] = block {
                    let center = rerun::external::glam::Vec3::new(*cx, *cy, *cz);
                    let half_size = rerun::external::glam::Vec3::new(*hx, *hy, *hz);
                    centers.push(center);
                    half_sizes.push(half_size);
                }
            });
        }
        _ => {
            return Err(eyre::eyre!(
                "Unknown format '{}'. Supported formats: min_max, center_size, center_half_size",
                format
            ));
        }
    }

    Ok((centers, half_sizes))
}
