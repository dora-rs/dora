// Copyright (c) 2019-2022, The rav1e contributors. All rights reserved
//
// This source code is subject to the terms of the BSD 2 Clause License and
// the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
// was not distributed with this source code in the LICENSE file, you can
// obtain it at www.aomedia.org/license/software. If the Alliance for Open
// Media Patent License 1.0 was not distributed with this source code in the
// PATENTS file, you can obtain it at www.aomedia.org/license/patent.

use std::env::var;

use dora_node_api::arrow::array::AsArray;
use dora_node_api::arrow::datatypes::{UInt16Type, UInt8Type};
use dora_node_api::dora_core::config::DataId;
use dora_node_api::{DoraNode, Event, IntoArrow, Metadata, Parameter};
use eyre::{Context as EyreContext, Result};
use log::warn;
use rav1e::color::{ColorDescription, MatrixCoefficients};
// Encode the same tiny blank frame 30 times
use rav1e::config::SpeedSettings;

use rav1e::*;

pub fn fill_zeros_toward_center_y_plane_in_place(y: &mut [u16], width: usize, height: usize) {
    assert_eq!(y.len(), width * height);

    for row in 0..height {
        let row_start = row * width;
        let center = width / 2;

        // --- Fill left half (left to center)
        let mut last = 0u16;
        for col in (0..center).rev() {
            let idx = row_start + col;
            if y[idx] == 0 {
                y[idx] = last;
            } else if y[idx] > 4096 {
                y[idx] = 4096;
            } else {
                last = y[idx];
            }
        }

        // --- Fill right half (right to center)
        last = 0u16;
        for col in center..width {
            let idx = row_start + col;
            if y[idx] == 0 {
                y[idx] = last;
            } else if y[idx] > 4096 {
                y[idx] = 4096;
            } else {
                last = y[idx];
            }
        }
    }
}

fn bgr8_to_yuv420(bgr_data: Vec<u8>, width: usize, height: usize) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
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

fn get_yuv_planes(buffer: &[u8], width: usize, height: usize) -> (&[u8], &[u8], &[u8]) {
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
    let y_plane = &buffer[0..y_size]; //.to_vec();
    let u_plane = &buffer[y_size..y_size + uv_size]; //.to_vec();
    let v_plane = &buffer[y_size + uv_size..]; //.to_vec();

    (y_plane, u_plane, v_plane)
}

fn send_yuv(
    y: &[u8],
    u: &[u8],
    v: &[u8],
    enc: EncoderConfig,
    width: usize,
    height: usize,
    node: &mut DoraNode,
    id: DataId,
    metadata: &mut Metadata,
    output_encoding: &str,
) -> () {
    // Create a new Arrow array for the YUV420 data
    let cfg = Config::new().with_encoder_config(enc.clone());
    let mut ctx: Context<u8> = cfg.new_context().unwrap();
    let mut f = ctx.new_frame();

    let xdec = f.planes[0].cfg.xdec;
    let stride = (width + xdec) >> xdec;
    f.planes[0].copy_from_raw_u8(&y, stride, 1);
    let xdec = f.planes[1].cfg.xdec;
    let stride = (width + xdec) >> xdec;
    f.planes[1].copy_from_raw_u8(&u, stride, 1);
    let xdec = f.planes[2].cfg.xdec;
    let stride = (width + xdec) >> xdec;
    f.planes[2].copy_from_raw_u8(&v, stride, 1);

    match ctx.send_frame(f) {
        Ok(_) => {}
        Err(e) => match e {
            EncoderStatus::EnoughData => {
                warn!("Unable to send frame ");
            }
            _ => {
                warn!("Unable to send frame ");
            }
        },
    }
    ctx.flush();
    match ctx.receive_packet() {
        Ok(pkt) => match output_encoding {
            "avif" => {
                let data = pkt.data.clone();
                metadata.parameters.insert(
                    "encoding".to_string(),
                    Parameter::String("avif".to_string()),
                );
                let matrix_coefficients = if let Some(desc) = enc.color_description {
                    desc.matrix_coefficients
                } else {
                    MatrixCoefficients::BT709
                };
                let data = avif_serialize::Aviffy::new()
                    .set_chroma_subsampling((true, true))
                    .set_seq_profile(0)
                    .matrix_coefficients(match matrix_coefficients {
                        MatrixCoefficients::Identity => {
                            avif_serialize::constants::MatrixCoefficients::Rgb
                        }
                        MatrixCoefficients::BT709 => {
                            avif_serialize::constants::MatrixCoefficients::Bt709
                        }
                        MatrixCoefficients::Unspecified => {
                            avif_serialize::constants::MatrixCoefficients::Unspecified
                        }
                        MatrixCoefficients::BT601 => {
                            avif_serialize::constants::MatrixCoefficients::Bt601
                        }
                        MatrixCoefficients::YCgCo => {
                            avif_serialize::constants::MatrixCoefficients::Ycgco
                        }
                        MatrixCoefficients::BT2020NCL => {
                            avif_serialize::constants::MatrixCoefficients::Bt2020Ncl
                        }
                        MatrixCoefficients::BT2020CL => {
                            avif_serialize::constants::MatrixCoefficients::Bt2020Cl
                        }
                        _ => {
                            warn!("color matrix coefficients");
                            avif_serialize::constants::MatrixCoefficients::Rgb
                        }
                    })
                    .to_vec(
                        &data,
                        None,
                        width as u32,
                        height as u32,
                        enc.bit_depth as u8,
                    );

                let arrow = data.into_arrow();
                node.send_output(id, metadata.parameters.clone(), arrow)
                    .context("could not send output")
                    .unwrap();
            }
            _ => {
                metadata
                    .parameters
                    .insert("encoding".to_string(), Parameter::String("av1".to_string()));
                metadata
                    .parameters
                    .insert("height".to_string(), Parameter::Integer(enc.height as i64));
                metadata
                    .parameters
                    .insert("width".to_string(), Parameter::Integer(enc.width as i64));

                let data = pkt.data;
                let arrow = data.into_arrow();
                node.send_output(id, metadata.parameters.clone(), arrow)
                    .context("could not send output")
                    .unwrap();
            }
        },
        Err(e) => match e {
            EncoderStatus::LimitReached => {}
            EncoderStatus::Encoded => {}
            EncoderStatus::NeedMoreData => {}
            _ => {
                panic!("Unable to receive packet",);
            }
        },
    }
}

pub fn lib_main() -> Result<()> {
    let mut height = std::env::var("IMAGE_HEIGHT")
        .unwrap_or_else(|_| "480".to_string())
        .parse()
        .unwrap();
    let mut width = std::env::var("IMAGE_WIDTH")
        .unwrap_or_else(|_| "640".to_string())
        .parse()
        .unwrap();
    let speed = var("RAV1E_SPEED").map(|s| s.parse().unwrap()).unwrap_or(10);
    let output_encoding = var("ENCODING").unwrap_or("av1".to_string());
    let mut enc = EncoderConfig {
        width,
        height,
        speed_settings: SpeedSettings::from_preset(speed),
        chroma_sampling: color::ChromaSampling::Cs420,
        color_description: Some(ColorDescription {
            matrix_coefficients: MatrixCoefficients::BT709,
            transfer_characteristics: color::TransferCharacteristics::BT709,
            color_primaries: color::ColorPrimaries::BT709,
        }),
        low_latency: true,
        ..Default::default()
    };
    let cfg = Config::new().with_encoder_config(enc.clone());
    cfg.validate()?;

    let (mut node, mut events) =
        DoraNode::init_from_env().context("Could not initialize dora node")?;

    loop {
        match events.recv() {
            Some(Event::Input {
                id,
                data,
                mut metadata,
            }) => {
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
                enc = EncoderConfig {
                    width,
                    height,
                    speed_settings: SpeedSettings::from_preset(speed),
                    low_latency: true,
                    chroma_sampling: color::ChromaSampling::Cs420,
                    ..Default::default()
                };
                match encoding {
                    "mono16" => {
                        enc.bit_depth = 12;
                        enc.chroma_sampling = color::ChromaSampling::Cs400;
                    }
                    _ => {}
                }

                if encoding == "bgr8" {
                    if let Some(buffer) = data.as_primitive_opt::<UInt8Type>() {
                        let buffer: Vec<u8> = buffer.values().to_vec();
                        let (y, u, v) = bgr8_to_yuv420(buffer, width, height);
                        send_yuv(
                            &y,
                            &u,
                            &v,
                            enc,
                            width,
                            height,
                            &mut node,
                            id,
                            &mut metadata,
                            &output_encoding,
                        );
                    }
                } else if encoding == "yuv420" {
                    if let Some(buffer) = data.as_primitive_opt::<UInt8Type>() {
                        let buffer = buffer.values(); //.to_vec();

                        let (y, u, v) = get_yuv_planes(buffer, width, height);
                        send_yuv(
                            &y,
                            &u,
                            &v,
                            enc,
                            width,
                            height,
                            &mut node,
                            id,
                            &mut metadata,
                            &output_encoding,
                        );
                    }
                } else if encoding == "mono16" || encoding == "z16" {
                    if let Some(buffer) = data.as_primitive_opt::<UInt16Type>() {
                        let mut buffer = buffer.values().to_vec();
                        if std::env::var("FILL_ZEROS")
                            .map(|s| s != "false")
                            .unwrap_or(true)
                        {
                            fill_zeros_toward_center_y_plane_in_place(&mut buffer, width, height);
                        }

                        let bytes: &[u8] = &bytemuck::cast_slice(&buffer);

                        let cfg = Config::new().with_encoder_config(enc.clone());
                        let mut ctx: Context<u16> = cfg.new_context().unwrap();
                        let mut f = ctx.new_frame();

                        let xdec = f.planes[0].cfg.xdec;
                        let stride = (width + xdec) >> xdec;
                        // Multiply by 2 the stride as it is going to be width * 2 as we're converting 16-bit to 2*8-bit.
                        f.planes[0].copy_from_raw_u8(bytes, stride * 2, 2);

                        match ctx.send_frame(f) {
                            Ok(_) => {}
                            Err(e) => match e {
                                EncoderStatus::EnoughData => {
                                    warn!("Unable to send frame ");
                                }
                                _ => {
                                    warn!("Unable to send frame ");
                                }
                            },
                        }
                        ctx.flush();
                        match ctx.receive_packet() {
                            Ok(pkt) => {
                                let data = pkt.data;
                                match output_encoding.as_str() {
                                    "avif" => {
                                        warn!("avif encoding not supported for mono16");
                                    }
                                    _ => {
                                        metadata.parameters.insert(
                                            "encoding".to_string(),
                                            Parameter::String("av1".to_string()),
                                        );
                                        let arrow = data.into_arrow();
                                        node.send_output(id, metadata.parameters, arrow)
                                            .context("could not send output")
                                            .unwrap();
                                    }
                                }
                            }
                            Err(e) => match e {
                                EncoderStatus::LimitReached => {}
                                EncoderStatus::Encoded => {}
                                EncoderStatus::NeedMoreData => {}
                                _ => {
                                    panic!("Unable to receive packet",);
                                }
                            },
                        }
                    }
                } else if encoding == "rgb8" {
                    if let Some(buffer) = data.as_primitive_opt::<UInt8Type>() {
                        let buffer: Vec<u8> = buffer.values().to_vec();
                        let buffer: Vec<u8> =
                            buffer.chunks(3).flat_map(|x| [x[2], x[1], x[0]]).collect();
                        let (y, u, v) = bgr8_to_yuv420(buffer, width, height);
                        send_yuv(
                            &y,
                            &u,
                            &v,
                            enc,
                            width,
                            height,
                            &mut node,
                            id,
                            &mut metadata,
                            &output_encoding,
                        );
                    }
                } else {
                    unimplemented!("We haven't worked on additional encodings.");
                }
            }
            Some(Event::Error(_e)) => {
                continue;
            }
            _ => break,
        };
    }

    Ok(())
}

#[cfg(feature = "python")]
use pyo3::{
    pyfunction, pymodule,
    types::{PyModule, PyModuleMethods},
    wrap_pyfunction, Bound, PyResult, Python,
};

#[cfg(feature = "python")]
#[pyfunction]
fn py_main(_py: Python) -> eyre::Result<()> {
    lib_main()
}

#[cfg(feature = "python")]
#[pymodule]
fn dora_rav1e(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
