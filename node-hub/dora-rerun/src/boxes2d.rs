use dora_node_api::{
    arrow::{
        array::AsArray,
        datatypes::{Float32Type, Float64Type, Int32Type, Int64Type},
    },
    dora_core::config::DataId,
    ArrowData, Metadata, Parameter,
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
        let bbox = bbox
            .as_primitive_opt::<Float32Type>()
            .context("Could not get bbox value as list")?
            .values();

        if bbox.len() == 0 {
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

        // Cast confidence
        let conf_buffer = bbox_struct
            .column_by_name("conf")
            .context("Did not find conf field within bbox struct")?;
        let conf = conf_buffer
            .as_list_opt::<i32>()
            .context("Could not deserialize conf as list")?
            .values();
        let _conf = conf
            .as_primitive_opt::<Float32Type>()
            .context("Could not deserialize conf as string")?;

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
        if bbox.len() == 0 {
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

        if values.len() == 0 {
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

        if values.len() == 0 {
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

        if values.len() == 0 {
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
        if values.len() == 0 {
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
