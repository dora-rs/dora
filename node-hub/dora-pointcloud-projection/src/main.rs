use dora_node_api::{
    arrow::{
        array::{AsArray, Float64Array, UInt8Array},
        datatypes::Int64Type,
    },
    dora_core::config::DataId,
    DoraNode, Event, IntoArrow, Parameter,
};
use std::{collections::HashMap, error::Error};

fn main() -> Result<(), Box<dyn Error>> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut image_cache = HashMap::new();

    let mut depth_frame = None;
    let mut width = 640;
    let mut height = 640;
    let mut focal_length = vec![605, 605];
    let mut resolution = vec![605, 605];

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "image" => {
                    let buffer: &UInt8Array = data.as_any().downcast_ref().unwrap();
                    image_cache.insert(id.clone(), buffer.values().to_vec());
                }
                "depth" => {
                    height = if let Some(Parameter::Integer(height)) =
                        metadata.parameters.get("height")
                    {
                        *height
                    } else {
                        height
                    };
                    width =
                        if let Some(Parameter::Integer(width)) = metadata.parameters.get("width") {
                            *width
                        } else {
                            width
                        };
                    focal_length = if let Some(Parameter::ListInt(focals)) =
                        metadata.parameters.get("focal")
                    {
                        focals.to_vec()
                    } else {
                        vec![605, 605]
                    };
                    resolution = if let Some(Parameter::ListInt(resolution)) =
                        metadata.parameters.get("resolution")
                    {
                        resolution.to_vec()
                    } else {
                        vec![640, 480]
                    };
                    let buffer: &Float64Array = data.as_any().downcast_ref().unwrap();
                    // if let Some(color_buffer) = image_cache.get(&id.replace("depth", "image")) {
                    // let buffer_length = color_buffer.len();
                    // let mut new_data = vec![0; buffer_length];
                    // buffer.iter().enumerate().for_each(|(i, z)| {
                    // // let i = buffer.len() - i - 1; // Reverse the order of the buffer
                    // let u = i as f32 % width as f32; // Calculate x-coordinate (u)
                    // let v = i as f32 / width as f32; // Calculate y-coordinate (v)
                    // let z = z.unwrap_or_default() as f32;

                    // let y = (u - resolution[0] as f32) * z / focal_length[0] as f32;
                    // let x = (v - resolution[1] as f32) * z / focal_length[1] as f32;

                    // let tmp_x = z * theta_y.cos() - x * theta_y.sin();
                    // let new_u =
                    // height as f32 - ((tmp_x - min_x) / (max_x - min_x) * height as f32);
                    // let new_v = (y - min_y) / (max_y - min_y) * width as f32;

                    // let new_u = new_u.round();
                    // let new_v = new_v.round();

                    // // verify if the point is within the bounds
                    // if new_u >= 0.0
                    // && new_u < height as f32
                    // && new_v >= 0.0
                    // && new_v < width as f32
                    // {
                    // let new_i = (new_v + new_u * width as f32) as usize;
                    // if new_data[new_i * 3] == 0 {
                    // new_data[new_i * 3] = color_buffer[i * 3];
                    // new_data[new_i * 3 + 1] = color_buffer[i * 3 + 1];
                    // new_data[new_i * 3 + 2] = color_buffer[i * 3 + 2];
                    // }
                    // }
                    // });

                    // let mut metadata = metadata.parameters.clone();
                    // metadata.insert(
                    // "encoding".to_string(),
                    // parameter::string("bgr8".to_string()),
                    // );
                    // node.send_output(
                    // dataid::from("image".to_string()),
                    // metadata,
                    // new_data.into_arrow(),
                    // )?;
                    depth_frame = Some(buffer.clone());
                }
                "boxes2d" => {
                    if let Some(data) = data.as_primitive_opt::<Int64Type>() {
                        let data = data.values();
                        let x_min = data[0] as f32;
                        let y_min = data[1] as f32;
                        let x_max = data[2] as f32;
                        let y_max = data[3] as f32;
                        let mut points = vec![];
                        let mut z_min = 10.;
                        let mut z_total = 0.;

                        if let Some(depth_frame) = &depth_frame {
                            depth_frame.iter().enumerate().for_each(|(i, z)| {
                                let u = i as f32 % width as f32; // Calculate x-coordinate (u)
                                let v = i as f32 / width as f32; // Calculate y-coordinate (v)

                                if u > x_min && u < x_max && v > y_min && v < y_max {
                                    if let Some(z) = z {
                                        let z = z as f32;
                                        let y =
                                            (u - resolution[0] as f32) * z / focal_length[0] as f32;
                                        let x =
                                            (v - resolution[1] as f32) * z / focal_length[1] as f32;
                                        let cos_theta = -0.78; // np.cos(np.deg2rad(180-38))
                                        let sin_theta = 0.61; // np.sin(np.deg2rad(180-38))

                                        let x = sin_theta * z + cos_theta * x;
                                        let y = -y;
                                        let z = -cos_theta * z + sin_theta * x;
                                        if z < z_min {
                                            z_min = z;
                                        }
                                        points.push((x, y, z));
                                        z_total += z;
                                    }
                                }
                            });
                        } else {
                            println!("No depth frame found");
                            continue;
                        }
                        if points.is_empty() {
                            continue;
                        }
                        let threshold = (z_total / points.len() as f32 + 1. * z_min) / 2.;

                        let (x, y, z) = points
                            .iter()
                            .filter(|(_x, _y, z)| z > &&threshold)
                            .fold((0., 0., 0.), |(acc_x, acc_y, acc_z), (x, y, z)| {
                                (acc_x + x, acc_y + y, acc_z + z)
                            });
                        let (x, y, z) = (
                            x / points.len() as f32,
                            y / points.len() as f32,
                            z / points.len() as f32,
                        );

                        let metadata = metadata.parameters.clone();

                        node.send_output(
                            DataId::from("point".to_string()),
                            metadata,
                            vec![x, y, z].into_arrow(),
                        )?;
                    }
                }
                other => eprintln!("Received input `{other}`"),
            },
            _ => {}
        }
    }

    Ok(())
}
