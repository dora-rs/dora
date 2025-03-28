use dav1d::Settings;
use dora_node_api::{arrow::array::UInt8Array, DoraNode, Event, IntoArrow};
use eyre::{Context, Result};
use log::warn;

fn yuv420_to_bgr(
    y_plane: &[u8],
    u_plane: &[u8],
    v_plane: &[u8],
    width: usize,
    height: usize,
) -> Vec<u8> {
    let mut rgb_data = vec![0u8; width * height * 3]; // Output RGB data buffer

    for j in 0..height {
        for i in 0..width {
            let y_idx = j * width + i; // Index in Y plane
            let uv_idx = (j / 2) * (width / 2) + (i / 2); // Index in U/V planes

            let y = y_plane[y_idx] as f32;
            let u = u_plane[uv_idx] as f32 - 128.0;
            let v = v_plane[uv_idx] as f32 - 128.0;

            // Convert YUV to RGB using BT.601 standard formula
            let r = (y + 1.402 * v).clamp(0.0, 255.0) as u8;
            let g = (y - 0.344136 * u - 0.714136 * v).clamp(0.0, 255.0) as u8;
            let b = (y + 1.772 * u).clamp(0.0, 255.0) as u8;

            // Set the RGB values in the output buffer
            let rgb_idx = y_idx * 3;
            rgb_data[rgb_idx] = b;
            rgb_data[rgb_idx + 1] = g;
            rgb_data[rgb_idx + 2] = r;
        }
    }

    rgb_data
}

fn main() -> Result<()> {
    let mut settings = Settings::new();
    // settings.set_n_threads(16);
    settings.set_max_frame_delay(1);

    let mut dec =
        dav1d::Decoder::with_settings(&settings).expect("failed to create decoder instance");

    let (mut node, mut events) =
        DoraNode::init_from_env().context("Could not initialize dora node")?;

    loop {
        match events.recv() {
            Some(Event::Input {
                id,
                data,
                mut metadata,
            }) => {
                let data = data.as_any().downcast_ref::<UInt8Array>().unwrap();
                let data = data.values().clone();

                match dec.send_data(data, None, None, None) {
                    Err(e) => {
                        warn!("Error sending data to the decoder: {}", e);
                    }
                    Ok(()) => {
                        if let Ok(p) = dec.get_picture() {
                            // println!("Time to decode: {:?}", time.elapsed());
                            let y = p.plane(dav1d::PlanarImageComponent::Y);
                            let u = p.plane(dav1d::PlanarImageComponent::U);
                            let v = p.plane(dav1d::PlanarImageComponent::V);
                            let y = yuv420_to_bgr(&y, &u, &v, width, height);

                            let arrow = y.into_arrow();
                            metadata.parameters.insert(
                                "encoding".to_string(),
                                dora_node_api::Parameter::String("bgr8".to_string()),
                            );
                            node.send_output(id, metadata.parameters, arrow).unwrap();
                        }
                    }
                }
            }
            None => break,
            Some(_) => break,
        }
    }

    Ok(())
}
