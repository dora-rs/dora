use std::time::Duration;

use dav1d::Settings;
use dora_node_api::{
    arrow::array::UInt8Array, dora_core::config::DataId, DoraNode, Event, IntoArrow,
    MetadataParameters,
};
use eyre::{Context, Result};

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
    settings.set_n_threads(16);

    let height: usize = std::env::var("IMAGE_HEIGHT")
        .unwrap_or_else(|_| "480".to_string())
        .parse()
        .unwrap();
    let width: usize = std::env::var("IMAGE_WIDTH")
        .unwrap_or_else(|_| "640".to_string())
        .parse()
        .unwrap();

    let mut dec =
        dav1d::Decoder::with_settings(&settings).expect("failed to create decoder instance");

    let (tx, rx) = std::sync::mpsc::sync_channel(1);
    let (mut node, mut events) =
        DoraNode::init_from_env().context("Could not initialize dora node")?;

    let _ = std::thread::spawn(move || {
        let mut now = std::time::Instant::now();
        loop {
            while let Ok(data) = rx.recv_timeout(Duration::from_millis(100)) {
                match dec.send_data(data, None, None, None) {
                    Err(e) if e.is_again() => {
                        panic!("Error sending data to the decoder: {}", e);
                        if let Ok(p) = dec.get_picture() {
                            let mut y = p.plane(dav1d::PlanarImageComponent::Y).to_vec();
                            let mut u = p.plane(dav1d::PlanarImageComponent::U).to_vec();
                            let mut v = p.plane(dav1d::PlanarImageComponent::V).to_vec();
                            y.append(&mut u);
                            y.append(&mut v);
                            // let rgb = yuv420_to_rgb(&y, &u, &v, 100, 100);
                            let arrow = y.into_arrow();
                            let mut metadata = MetadataParameters::default();
                            metadata.insert(
                                "width".to_string(),
                                dora_node_api::Parameter::Integer(640),
                            );
                            metadata.insert(
                                "height".to_string(),
                                dora_node_api::Parameter::Integer(480),
                            );
                            metadata.insert(
                                "encoding".to_string(),
                                dora_node_api::Parameter::String("yuv420".to_string()),
                            );
                            node.send_output(DataId::from("frame".to_string()), metadata, arrow)
                                .unwrap();
                            println!("Time to decode: {:?}", now.elapsed());
                            now = std::time::Instant::now();
                        }
                        // If the decoder did not consume all data, output all
                        // pending pictures and send pending data to the decoder
                        // until it is all used up.
                        //loop {
                        // handle_pending_pictures(&mut dec, false, &mut node);

                        // match dec.send_pending_data() {
                        // Err(e) if e.is_again() => continue,
                        // Err(e) => {
                        // panic!("Error sending pending data to the decoder: {}", e);
                        // }
                        // _ => break,
                        // }
                        //}
                    }
                    Err(e) => {
                        panic!("Error sending data to the decoder: {}", e);
                    }
                    Ok(()) => {
                        if let Ok(p) = dec.get_picture() {
                            let mut y = p.plane(dav1d::PlanarImageComponent::Y).to_vec();
                            let mut u = p.plane(dav1d::PlanarImageComponent::U).to_vec();
                            let mut v = p.plane(dav1d::PlanarImageComponent::V).to_vec();
                            // u.iter_mut().for_each(|e| {
                            // if *e < 128 {
                            // *e = *e + 128
                            // }
                            // });
                            // v.iter_mut().for_each(|e: &mut u8| {
                            // if *e < 128 {
                            // *e = *e + 128
                            // }
                            // });

                            // y.append(&mut u);
                            // y.append(&mut v);
                            let y = yuv420_to_bgr(&y, &u, &v, width, height);

                            let arrow = y.into_arrow();
                            let mut metadata = MetadataParameters::default();
                            metadata.insert(
                                "width".to_string(),
                                dora_node_api::Parameter::Integer(
                                    width.try_into().unwrap_or_default(),
                                ),
                            );
                            metadata.insert(
                                "height".to_string(),
                                dora_node_api::Parameter::Integer(
                                    height.try_into().unwrap_or_default(),
                                ),
                            );
                            metadata.insert(
                                "encoding".to_string(),
                                dora_node_api::Parameter::String("bgr8".to_string()),
                            );
                            node.send_output(DataId::from("frame".to_string()), metadata, arrow)
                                .unwrap();
                            println!("Time to decode: {:?}", now.elapsed());
                            println!("Delay: {:#?}", dec.get_frame_delay());

                            now = std::time::Instant::now();
                        }
                    }
                }
            }
        }
    });
    loop {
        match events.recv() {
            Some(Event::Input {
                id: _,
                data,
                metadata: _,
            }) => {
                let data = data.as_any().downcast_ref::<UInt8Array>().unwrap();
                let data = data.values().clone();

                // Send packet to the decoder

                tx.try_send(data).unwrap_or_default();
                // Handle all pending pictures before sending the next data.
                // handle_pending_pictures(&mut dec, false, &mut node);
            }
            None => break,
            Some(_) => break,
        }
    }
    // Handle all pending pictures that were not output yet.
    // handle_pending_pictures(&mut dec, true, &mut node);

    Ok(())
}
