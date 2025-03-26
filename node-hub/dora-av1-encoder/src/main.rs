// Copyright (c) 2019-2022, The rav1e contributors. All rights reserved
//
// This source code is subject to the terms of the BSD 2 Clause License and
// the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
// was not distributed with this source code in the LICENSE file, you can
// obtain it at www.aomedia.org/license/software. If the Alliance for Open
// Media Patent License 1.0 was not distributed with this source code in the
// PATENTS file, you can obtain it at www.aomedia.org/license/patent.

use std::time::Duration;

use dora_node_api::arrow::array::UInt8Array;
use dora_node_api::dora_core::config::DataId;
use dora_node_api::{DoraNode, Event, IntoArrow, MetadataParameters, Parameter};
use eyre::{Context as EyreContext, Result};
// Encode the same tiny blank frame 30 times
use rav1e::config::RateControlConfig;
use rav1e::config::SpeedSettings;

use rav1e::*;

fn bgr_to_yuv(bgr_data: Vec<u8>, width: usize, height: usize) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
    let mut y_plane = vec![0; width * height];
    let mut u_plane = vec![0; (width / 2) * (height / 2)];
    let mut v_plane = vec![0; (width / 2) * (height / 2)];

    for y in 0..height {
        for x in 0..width {
            let bgr_index = (y * width + x) * 3;
            let b = bgr_data[bgr_index] as f32;
            let g = bgr_data[bgr_index + 1] as f32;
            let r = bgr_data[bgr_index + 2] as f32;

            // Corrected YUV conversion formulas
            let y_value = (0.299 * r + 0.587 * g + 0.114 * b).clamp(0.0, 255.0);
            let u_value = (-0.14713 * r - 0.28886 * g + 0.436 * b + 128.0).clamp(0.0, 255.0);
            let v_value = (0.615 * r - 0.51499 * g - 0.10001 * b + 128.0).clamp(0.0, 255.0);

            let y_index = y * width + x;
            y_plane[y_index] = y_value.round() as u8;

            if x % 2 == 0 && y % 2 == 0 {
                let uv_index = (y / 2) * (width / 2) + (x / 2);
                u_plane[uv_index] = u_value.round() as u8;
                v_plane[uv_index] = v_value.round() as u8;
            }
        }
    }

    (y_plane, u_plane, v_plane)
}

fn get_yuv_planes(buffer: Vec<u8>, width: usize, height: usize) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
    // Calculate sizes of Y, U, and V planes for YUV420 format
    let y_size = width * height; // Y has full resolution
    let uv_width = width / 2; // U and V are subsampled by 2 in both dimensions
    let uv_height = height / 2; // U and V are subsampled by 2 in both dimensions
    let uv_size = uv_width * uv_height; // Size for U and V planes

    // Ensure the buffer has the correct size
    // if buffer.len() != y_size + 2 * uv_size {
    // panic!("Invalid buffer size for the given width and height!");
    // }

    // Extract Y, U, and V planes
    let y_plane = buffer[0..y_size].to_vec();
    let u_plane = buffer[y_size..y_size + uv_size].to_vec();
    let v_plane = buffer[y_size + uv_size..].to_vec();

    (y_plane, u_plane, v_plane)
}

fn main() -> Result<()> {
    let mut height = std::env::var("IMAGE_HEIGHT")
        .unwrap_or_else(|_| "480".to_string())
        .parse()
        .unwrap();
    let mut width = std::env::var("IMAGE_WIDTH")
        .unwrap_or_else(|_| "640".to_string())
        .parse()
        .unwrap();
    let enc = EncoderConfig {
        width,
        height,
        speed_settings: SpeedSettings::from_preset(8),
        low_latency: true,
        ..Default::default()
    };

    let cfg = Config::new()
        //    .with_rate_control(RateControlConfig::new().with_emit_data(true))
        .with_encoder_config(enc.clone())
        .with_threads(16);

    let mut ctx: Context<u16> = cfg.new_context().unwrap();

    let (mut node, mut events) =
        DoraNode::init_from_env().context("Could not initialize dora node")?;

    let mut time = std::time::Instant::now();
    loop {
        let buffer = match events.recv() {
            Some(Event::Input { id, data, metadata }) => {
                if let Some(Parameter::Integer(h)) = metadata.parameters.get("height") {
                    height = *h as usize;
                };
                if let Some(Parameter::Integer(w)) = metadata.parameters.get("width") {
                    width = *w as usize;
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
                    let buffer: Vec<u8> = buffer.values().to_vec();
                    buffer
                    // Transpose values from BGR to RGB
                    // let buffer: Vec<u8> = buffer.chunks(3).flat_map(|x| [x[2], x[1], x[0]]).collect();

                    //un
                } else if encoding == "yuv420" {
                    let buffer: &UInt8Array = data.as_any().downcast_ref().unwrap();
                    let buffer: Vec<u8> = buffer.values().to_vec();
                    buffer
                } else if encoding == "rgb8" {
                    unimplemented!("We haven't worked on additional encodings.");
                    let buffer: &UInt8Array = data.as_any().downcast_ref().unwrap();
                    let buffer: &[u8] = buffer.values();
                    let mut f = ctx.new_frame();

                    for p in &mut f.planes {
                        let stride = (enc.width + p.cfg.xdec) >> p.cfg.xdec;
                        p.copy_from_raw_u8(&buffer, stride, 1);
                    }
                    buffer.to_vec()
                } else {
                    unimplemented!("We haven't worked on additional encodings.");
                    continue;
                }
            }
            Some(Event::Error(e)) => {
                continue;
            }
            _ => break,
        };
        //let (y, u, v) = bgr_to_yuv(buffer, 640 as usize, 480 as usize);

        let (y, u, v) = get_yuv_planes(buffer, width, height);
        let mut f = ctx.new_frame();

        let xdec = f.planes[0].cfg.xdec;
        let stride = (enc.width + xdec) >> xdec;
        f.planes[0].copy_from_raw_u8(&y, stride, 1);
        let xdec = f.planes[1].cfg.xdec;
        let stride = (enc.width + xdec) >> xdec;
        f.planes[1].copy_from_raw_u8(&u, stride, 1);
        let xdec = f.planes[2].cfg.xdec;
        let stride = (enc.width + xdec) >> xdec;
        f.planes[2].copy_from_raw_u8(&v, stride, 1);

        match ctx.send_frame(f) {
            Ok(_) => {}
            Err(e) => match e {
                EncoderStatus::EnoughData => {
                    println!("Unable to append frame to the internal queue");
                    panic!("Unable to send frame ");
                }
                _ => {
                    panic!("Unable to send frame ");
                }
            },
        }
        println!("Frame sent to encoder");
        for _ in 0..1 {
            match ctx.receive_packet() {
                Ok(pkt) => {
                    println!("Time to encode: {:?}", time.elapsed());
                    time = std::time::Instant::now();
                    let data = pkt.data;
                    println!("frame compression: {:#?}", width * height * 3 / data.len());
                    println!("frame size: {:#?}", data.len());
                    let arrow = data.into_arrow();
                    node.send_output(
                        DataId::from("frame".to_owned()),
                        MetadataParameters::default(),
                        arrow,
                    )
                    .context("could not send output")
                    .unwrap();

                    break;
                }
                Err(e) => match e {
                    EncoderStatus::LimitReached => {
                        break;
                    }
                    EncoderStatus::Encoded => {
                        break;
                    }
                    EncoderStatus::NeedMoreData => {
                        break;
                    }
                    _ => {
                        panic!("Unable to receive packet",);
                    }
                },
            }
        }
    }

    Ok(())
}
