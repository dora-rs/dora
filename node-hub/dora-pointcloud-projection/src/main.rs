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
                        let mut z_min = 100.;
                        let mut z_total = 0.;
                        let mut n = 0.;

                        if let Some(depth_frame) = &depth_frame {
                            depth_frame.iter().enumerate().for_each(|(i, z)| {
                                let u = i as f32 % width as f32; // Calculate x-coordinate (u)
                                let v = i as f32 / width as f32; // Calculate y-coordinate (v)

                                if u > x_min && u < x_max && v > y_min && v < y_max {
                                    if let Some(z) = z {
                                        let z = z as f32;

                                        // Skip points that have empty depth.
                                        if z == 0. {
                                            return;
                                        // Skip points that are out of reach of reachy
                                        } else if z > 5.0 {
                                            return;
                                        }
                                        let y =
                                            (u - resolution[0] as f32) * z / focal_length[0] as f32;
                                        let x =
                                            (v - resolution[1] as f32) * z / focal_length[1] as f32;
                                        let cos_theta = -0.78; // np.cos(np.deg2rad(180-38))
                                        let sin_theta = 0.61; // np.sin(np.deg2rad(180-38))
                                                              // (0.32489833, -0.25068134, 0.4761387)
                                        let new_x = sin_theta * z + cos_theta * x;
                                        let new_y = -y;
                                        let new_z = cos_theta * z - sin_theta * x;
                                        if new_z < z_min {
                                            z_min = new_z;
                                        }
                                        points.push((new_x, new_y, new_z));
                                        z_total += new_z;
                                        n += 1.;
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
                        let raw_mean_z = z_total / n as f32;
                        let threshold = (raw_mean_z * 4. + z_min) / 5.;

                        let (x, y, z, sum_xy, sum_x2, sum_y2, n) =
                            points.iter().filter(|(_x, _y, z)| z > &&threshold).fold(
                                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                                |(acc_x, acc_y, acc_z, acc_xy, acc_x2, acc_y2, acc_n),
                                 (x, y, z)| {
                                    (
                                        acc_x + x,
                                        acc_y + y,
                                        acc_z + z,
                                        acc_xy + x * y,
                                        acc_x2 + x * x,
                                        acc_y2 + y * y,
                                        acc_n + 1.,
                                    )
                                },
                            );

                        let (mean_x, mean_y, mean_z) = (x / n as f32, y / n as f32, z / n as f32);

                        // Compute covariance and standard deviations
                        let cov = sum_xy / n - mean_x * mean_y;
                        let std_x = (sum_x2 / n - mean_x * mean_x).sqrt();
                        let std_y = (sum_y2 / n - mean_y * mean_y).sqrt();
                        let corr = cov / (std_x * std_y);
                        let metadata = metadata.parameters.clone();

                        node.send_output(
                            DataId::from("point".to_string()),
                            metadata,
                            vec![mean_x, mean_y, mean_z, 0., 0., corr * 3.1415 / 2.].into_arrow(),
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
