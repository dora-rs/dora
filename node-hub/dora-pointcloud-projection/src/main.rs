use dora_node_api::{
    arrow::array::{Float64Array, UInt8Array},
    dora_core::config::DataId,
    DoraNode, Event, IntoArrow, Parameter,
};
use std::{collections::HashMap, error::Error};

fn main() -> Result<(), Box<dyn Error>> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut image_cache = HashMap::new();

    let min_x = std::env::var("MIN_X").unwrap_or_default().parse::<f32>()?;
    let max_x = std::env::var("MAX_X")
        .unwrap_or("5.0".to_string())
        .parse::<f32>()?;
    let min_y = std::env::var("MIN_Y")
        .unwrap_or("-2.0".to_string())
        .parse::<f32>()?;
    let max_y = std::env::var("MAX_Y")
        .unwrap_or("2.0".to_string())
        .parse::<f32>()?;
    let theta_y = std::env::var("THETA_Y")
        .unwrap_or("0.9".to_string())
        .parse::<f32>()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "image" => {
                    let buffer: &UInt8Array = data.as_any().downcast_ref().unwrap();
                    image_cache.insert(id.clone(), buffer.values().to_vec());
                }
                "depth" => {
                    let height = if let Some(Parameter::Integer(height)) =
                        metadata.parameters.get("height")
                    {
                        height
                    } else {
                        &640
                    };
                    let width =
                        if let Some(Parameter::Integer(width)) = metadata.parameters.get("width") {
                            width
                        } else {
                            &640
                        };
                    let focal_length = if let Some(Parameter::ListInt(focals)) =
                        metadata.parameters.get("focal")
                    {
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

                    if let Some(color_buffer) = image_cache.get(&id.replace("depth", "image")) {
                        let buffer_length = color_buffer.len();
                        let mut new_data = vec![0; buffer_length];
                        buffer.iter().enumerate().for_each(|(i, z)| {
                            // let i = buffer.len() - i - 1; // Reverse the order of the buffer
                            let u = i as f32 % *width as f32; // Calculate x-coordinate (u)
                            let v = i as f32 / *width as f32; // Calculate y-coordinate (v)
                            let z = z.unwrap_or_default() as f32;

                            let y = (u - resolution[0] as f32) * z / focal_length[0] as f32;
                            let x = (v - resolution[1] as f32) * z / focal_length[1] as f32;

                            let tmp_x = z * theta_y.cos() - x * theta_y.sin();
                            let new_u = *height as f32
                                - ((tmp_x - min_x) / (max_x - min_x) * *height as f32);
                            let new_v = (y - min_y) / (max_y - min_y) * *width as f32;

                            let new_u = new_u.round();
                            let new_v = new_v.round();

                            // verify if the point is within the bounds
                            if new_u >= 0.0
                                && new_u < *height as f32
                                && new_v >= 0.0
                                && new_v < *width as f32
                            {
                                let new_i = (new_v + new_u * *width as f32) as usize;
                                if new_data[new_i * 3] == 0 {
                                    new_data[new_i * 3] = color_buffer[i * 3];
                                    new_data[new_i * 3 + 1] = color_buffer[i * 3 + 1];
                                    new_data[new_i * 3 + 2] = color_buffer[i * 3 + 2];
                                }
                            }
                        });

                        let mut metadata = metadata.parameters.clone();
                        metadata.insert(
                            "encoding".to_string(),
                            Parameter::String("bgr8".to_string()),
                        );
                        node.send_output(
                            DataId::from("image".to_string()),
                            metadata,
                            new_data.into_arrow(),
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
