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

pub fn update_boxes2d(
    rec: &RecordingStream,
    id: DataId,
    data: ArrowData,
    metadata: Metadata,
) -> Result<()> {
    let format = if let Some(Parameter::String(format)) = metadata.parameters.get("format") {
        format
    } else {
        "xyxy"
    };
    if let Some(bbox_struct) = data.as_struct_opt() {
        // Cast Bbox
        let bbox_buffer = bbox_struct
            .column_by_name("bbox")
            .context("Did not find labels field within bbox struct")?;
        let bbox = bbox_buffer
            .as_list_opt::<i32>()
            .context("Could not deserialize bbox as list")?
            .values();
        let bbox = match bbox.data_type() {
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
                .wrap_err("Could not log Boxes2D")?;
            return Ok(());
        }

        // Cast Labels
        let labels_buffer = bbox_struct
            .column_by_name("labels")
            .context("Did not find labels field within bbox struct")?;
        let labels = labels_buffer
            .as_list_opt::<i32>()
            .context("Could not deserialize labels as list")?
            .values();
        let labels = labels
            .as_string_opt::<i32>()
            .context("Could not deserialize labels as string")?;
        let labels: Vec<Text> = labels.iter().map(|x| Text::from(x.unwrap())).collect();

        let mut centers = vec![];
        let mut sizes = vec![];

        if format == "xywh" {
            bbox.chunks(4).for_each(|block| {
                if let [x, y, w, h] = block {
                    centers.push((*x, *y));
                    sizes.push((*w, *h));
                }
            });
        } else if format == "xyxy" {
            bbox.chunks(4).for_each(|block| {
                if let [min_x, min_y, max_x, max_y] = block {
                    centers.push(((max_x + min_x) / 2., (max_y + min_y) / 2.));
                    sizes.push(((max_x - min_x), (max_y - min_y)));
                }
            });
        }
        if bbox.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes2D")?;
            return Ok(());
        } else {
            rec.log(
                id.as_str(),
                &rerun::Boxes2D::from_centers_and_sizes(centers, sizes).with_labels(labels),
            )
            .wrap_err("Could not log Boxes2D")?;
        }
    } else if let Some(buffer_array) = data.as_primitive_opt::<Float32Type>() {
        let values = buffer_array.values();
        let mut centers = vec![];
        let mut sizes = vec![];
        if format == "xywh" {
            values.chunks(4).for_each(|block| {
                if let [x, y, w, h] = block {
                    centers.push((*x, *y));
                    sizes.push((*w, *h));
                }
            });
        } else if format == "xyxy" {
            values.chunks(4).for_each(|block| {
                if let [min_x, min_y, max_x, max_y] = block {
                    centers.push(((max_x + min_x) / 2., (max_y + min_y) / 2.));
                    sizes.push(((max_x - min_x), (max_y - min_y)));
                }
            });
        }

        if values.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes2D")?;
            return Ok(());
        } else {
            rec.log(
                id.as_str(),
                &rerun::Boxes2D::from_centers_and_sizes(centers, sizes),
            )
            .wrap_err("Could not log Boxes2D")?;
        }
    } else if let Some(buffer_array) = data.as_primitive_opt::<Float64Type>() {
        let values = buffer_array.values();
        let mut centers = vec![];
        let mut sizes = vec![];
        if format == "xywh" {
            values.chunks(4).for_each(|block| {
                if let [x, y, w, h] = block {
                    centers.push((*x as f32, *y as f32));
                    sizes.push((*w as f32, *h as f32));
                }
            });
        } else if format == "xyxy" {
            values.chunks(4).for_each(|block| {
                if let [min_x, min_y, max_x, max_y] = block {
                    centers.push(((max_x + min_x) as f32 / 2., (max_y + min_y) as f32 / 2.));
                    sizes.push(((max_x - min_x) as f32, (max_y - min_y) as f32));
                }
            });
        }

        if values.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes2D")?;
            return Ok(());
        } else {
            rec.log(
                id.as_str(),
                &rerun::Boxes2D::from_centers_and_sizes(centers, sizes),
            )
            .wrap_err("Could not log Boxes2D")?;
        }
    } else if let Some(buffer_array) = data.as_primitive_opt::<Int64Type>() {
        let values = buffer_array.values();
        let mut centers = vec![];
        let mut sizes = vec![];
        if format == "xywh" {
            values.chunks(4).for_each(|block| {
                if let [x, y, w, h] = block {
                    centers.push((*x as f32, *y as f32));
                    sizes.push((*w as f32, *h as f32));
                }
            });
        } else if format == "xyxy" {
            values.chunks(4).for_each(|block| {
                if let [min_x, min_y, max_x, max_y] = block {
                    centers.push(((max_x + min_x) as f32 / 2., (max_y + min_y) as f32 / 2.));
                    sizes.push(((max_x - min_x) as f32, (max_y - min_y) as f32));
                }
            });
        }

        if values.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes2D")?;
            return Ok(());
        } else {
            rec.log(
                id.as_str(),
                &rerun::Boxes2D::from_centers_and_sizes(centers, sizes),
            )
            .wrap_err("Could not log Boxes2D")?;
        }
    } else if let Some(buffer_array) = data.as_primitive_opt::<Int32Type>() {
        let values = buffer_array.values();
        let mut centers = vec![];
        let mut sizes = vec![];
        if format == "xywh" {
            values.chunks(4).for_each(|block| {
                if let [x, y, w, h] = block {
                    centers.push((*x as f32, *y as f32));
                    sizes.push((*w as f32, *h as f32));
                }
            });
        } else if format == "xyxy" {
            values.chunks(4).for_each(|block| {
                if let [min_x, min_y, max_x, max_y] = block {
                    centers.push(((max_x + min_x) as f32 / 2., (max_y + min_y) as f32 / 2.));
                    sizes.push(((max_x - min_x) as f32, (max_y - min_y) as f32));
                }
            });
        }
        if values.is_empty() {
            rec.log(id.as_str(), &rerun::Clear::flat())
                .wrap_err("Could not log Boxes2D")?;
            return Ok(());
        } else {
            rec.log(
                id.as_str(),
                &rerun::Boxes2D::from_centers_and_sizes(centers, sizes),
            )
            .wrap_err("Could not log Boxes2D")?;
        }
    }
    Ok(())
}
