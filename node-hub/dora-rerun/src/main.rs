//! Demonstrates the most barebone usage of the Rerun SDK.

use std::{collections::HashMap, env::VarError};

use dora_node_api::{
    arrow::{
        array::{AsArray, Float32Array, StringArray, StructArray, UInt8Array},
        datatypes::Float32Type,
    },
    DoraNode, Event, Parameter,
};
use eyre::{eyre, Context, ContextCompat, Result};
use k::Chain;
use rerun::{
    components::{ImageBuffer, RotationAxisAngle},
    external::re_types::ArrowBuffer,
    Angle, ImageFormat, RecordingStream, Rotation3D, SpawnOptions, Text, Vec3D,
};
pub struct MyIntersperse<T, I> {
    iterator: I,
    sep: T,
    nxt: Option<T>,
}

pub trait MyIntersperseExt<T: Clone, I: Iterator>: Iterator<Item = T> {
    fn my_intersperse(self, sep: T) -> MyIntersperse<T, I>;
}

impl<T: Clone, I: Iterator<Item = T>> MyIntersperseExt<T, I> for I {
    fn my_intersperse(mut self, sep: T) -> MyIntersperse<T, I> {
        let next = self.next();
        MyIntersperse {
            iterator: self,
            sep: sep.clone(),
            nxt: next,
        }
    }
}

impl<T: Clone, I: Iterator<Item = T>> Iterator for MyIntersperse<T, I> {
    type Item = T;
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(item) = self.nxt.take() {
            Some(item)
        } else {
            self.nxt = self.iterator.next();
            if self.nxt.is_some() {
                Some(self.sep.clone())
            } else {
                None
            }
        }
    }
}

fn get_entity_path(link: &k::Node<f32>, urdf_path: &str) -> String {
    let mut ancestors: Vec<_> = link
        .iter_ancestors()
        .map(|node| node.link().as_ref().unwrap().name.clone())
        .collect();
    ancestors.push(String::from(urdf_path));
    ancestors
        .into_iter()
        .rev()
        .my_intersperse(String::from("/"))
        .collect()
}

fn update_visualization(
    chain: &Chain<f32>,
    rec: &RecordingStream,
    id: &str,
    positions: &[f32],
) -> Result<()> {
    chain.set_joint_positions_clamped(&positions);

    chain.update_transforms();
    chain.update_link_transforms();

    for link_name in chain.iter_links().map(|link| link.name.clone()) {
        let link = chain.find_link(&link_name).unwrap();
        let entity_path = get_entity_path(&link, &id);
        let link_to_world = link
            .world_transform()
            .context("Could not get world transform")?;
        let link_to_parent = if link_name != "base_link" {
            let parent = link.parent().unwrap();
            parent.world_transform().unwrap().inv_mul(&link_to_world)
        } else {
            link_to_world
        };
        let link_to_parent_mat = link_to_parent.to_matrix();

        let trans = link_to_parent_mat.column(3);
        let trans = trans.as_slice();
        let quat = link_to_parent.rotation.quaternion();
        rec.log(
            entity_path,
            &rerun::Transform3D::from_translation_rotation(
                Vec3D::new(trans[0], trans[1], trans[2]),
                rerun::Quaternion::from([quat.i, quat.j, quat.k, quat.w]),
            ),
        )
        .context("Could not log transform")?;
    }
    Ok(())
}

fn main() -> Result<()> {
    // rerun `serve()` requires to have a running Tokio runtime in the current context.
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .expect("Failed to create tokio runtime");
    let _guard = rt.enter();

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

    // Get all env variable that start with URDF_
    let urdfs = std::env::vars()
        .filter(|(key, _)| key.ends_with("_urdf"))
        .collect::<Vec<_>>();
    let mut chains = HashMap::new();
    for (key, urdf_path) in urdfs {
        let path = key.replace("_urdf", ".urdf");
        let chain = k::Chain::<f32>::from_urdf_file(&urdf_path).unwrap();

        let transform = key.replace("_urdf", "_transform");
        rec.log_file_from_path(&urdf_path, None, false).unwrap();
        // Get transform by replacing URDF_ with TRANSFORM_
        if let Ok(transform) = std::env::var(transform) {
            let transform = transform
                .split(' ')
                .map(|x| x.parse::<f32>().unwrap())
                .collect::<Vec<f32>>();
            rec.log(
                path.clone(),
                &rerun::Transform3D::from_translation_rotation(
                    [transform[0], transform[1], transform[2]],
                    Rotation3D::AxisAngle(RotationAxisAngle::new(
                        [0., 0., 0.],
                        Angle::from_degrees(0.0),
                    )),
                ),
            )
            .unwrap();
            chains.insert(path, chain);
        }
    }
    let (_node, mut events) =
        DoraNode::init_from_env().context("Could not initialize dora node")?;

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
                let bbox_struct: StructArray = data.to_data().into();
                let format =
                    if let Some(Parameter::String(format)) = metadata.parameters.get("format") {
                        format
                    } else {
                        "xyxy"
                    };

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
                rec.log(
                    id.as_str(),
                    &rerun::Boxes2D::from_centers_and_sizes(centers, sizes).with_labels(labels),
                )
                .wrap_err("Could not log Boxes2D")?;
            } else if id.as_str().contains("jointstate") {
                let buffer: &Float32Array = data
                    .as_any()
                    .downcast_ref()
                    .context("data is not float32")?;
                let mut positions: Vec<f32> = buffer.values().to_vec();

                // Match file name
                let mut id = id.as_str().replace("jointstate_", "");
                id.push_str(".urdf");

                let chain = chains.get(&id).unwrap();
                let dof = chain.dof();

                // Truncate or pad positions to match the chain's dof
                if dof < positions.len() {
                    positions.truncate(dof);
                } else {
                    for _ in 0..(dof - positions.len()) {
                        positions.push(0.);
                    }
                }

                update_visualization(&chain, &rec, &id, &positions)?;
            }
        }
    }

    Ok(())
}
