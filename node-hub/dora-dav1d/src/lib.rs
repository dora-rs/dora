use std::env::var;

use dav1d::Settings;
use dora_node_api::{arrow::array::UInt8Array, DoraNode, Event, IntoArrow};
use eyre::{Context, Result};
use log::warn;

fn yuv420_to_bgr(
    y_plane: &[u8],
    u_plane: &[u8],
    v_plane: &[u8],
    width: u32,
    height: u32,
) -> Vec<u8> {
    let width = width as usize;
    let height = height as usize;
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

pub fn lib_main() -> Result<()> {
    let mut settings = Settings::new();
    // settings.set_n_threads(16);
    settings.set_max_frame_delay(1);
    let mut dec =
        dav1d::Decoder::with_settings(&settings).expect("failed to create decoder instance");

    let (mut node, mut events) =
        DoraNode::init_from_env().context("Could not initialize dora node")?;

    let output_encoding = var("ENCODING").unwrap_or("bgr8".to_string());

    loop {
        match events.recv() {
            Some(Event::Input {
                id,
                data,
                mut metadata,
            }) => {
                if let Some(data) = data.as_any().downcast_ref::<UInt8Array>() {
                    let data = data.values().clone();
                    let encoding = metadata
                        .parameters
                        .get("encoding")
                        .and_then(|p| match p {
                            dora_node_api::Parameter::String(s) => Some(s),
                            _ => None,
                        })
                        .map(|s| s.as_str())
                        .unwrap_or("av1");
                    if encoding != "av1" {
                        warn!("Unsupported encoding {}", encoding);
                        continue;
                    }
                    match dec.send_data(data, None, None, None) {
                        Err(e) => {
                            warn!("Error sending data to the decoder: {}", e);
                        }
                        Ok(()) => {
                            if let Ok(p) = dec.get_picture() {
                                match p.pixel_layout() {
                                    dav1d::PixelLayout::I420 => {
                                        let y = p.plane(dav1d::PlanarImageComponent::Y);
                                        let u = p.plane(dav1d::PlanarImageComponent::U);
                                        let v = p.plane(dav1d::PlanarImageComponent::V);
                                        match output_encoding.as_str() {
                                            "yuv420" => {
                                                let mut y = y.to_vec();
                                                let mut u = u.to_vec();
                                                let mut v = v.to_vec();
                                                y.append(&mut u);
                                                y.append(&mut v);
                                                let arrow = y.into_arrow();
                                                metadata.parameters.insert(
                                                    "encoding".to_string(),
                                                    dora_node_api::Parameter::String(
                                                        "yuv420".to_string(),
                                                    ),
                                                );
                                                metadata.parameters.insert(
                                                    "width".to_string(),
                                                    dora_node_api::Parameter::Integer(
                                                        p.width() as i64
                                                    ),
                                                );
                                                metadata.parameters.insert(
                                                    "height".to_string(),
                                                    dora_node_api::Parameter::Integer(
                                                        p.height() as i64
                                                    ),
                                                );

                                                node.send_output(id, metadata.parameters, arrow)
                                                    .unwrap();
                                            }
                                            "bgr8" => {
                                                let y = yuv420_to_bgr(
                                                    &y,
                                                    &u,
                                                    &v,
                                                    p.width(),
                                                    p.height(),
                                                );
                                                let arrow = y.into_arrow();
                                                metadata.parameters.insert(
                                                    "encoding".to_string(),
                                                    dora_node_api::Parameter::String(
                                                        "bgr8".to_string(),
                                                    ),
                                                );
                                                node.send_output(id, metadata.parameters, arrow)
                                                    .unwrap();
                                            }
                                            _ => {
                                                warn!(
                                                    "Unsupported output encoding {}",
                                                    output_encoding
                                                );
                                                continue;
                                            }
                                        }
                                    }
                                    dav1d::PixelLayout::I400 => {
                                        let y = p.plane(dav1d::PlanarImageComponent::Y);
                                        match p.bit_depth() {
                                            8 => {
                                                let y = y.to_vec();
                                                let arrow = y.into_arrow();
                                                metadata.parameters.insert(
                                                    "encoding".to_string(),
                                                    dora_node_api::Parameter::String(
                                                        "mono8".to_string(),
                                                    ),
                                                );
                                                node.send_output(id, metadata.parameters, arrow)
                                                    .unwrap();
                                            }
                                            10 | 12 => {
                                                let vec16: Vec<u16> =
                                                    bytemuck::cast_slice(&y).to_vec();
                                                let arrow = vec16.into_arrow();
                                                metadata.parameters.insert(
                                                    "encoding".to_string(),
                                                    dora_node_api::Parameter::String(
                                                        "mono16".to_string(),
                                                    ),
                                                );
                                                node.send_output(id, metadata.parameters, arrow)
                                                    .unwrap();
                                            }
                                            _ => {
                                                warn!("Unsupported bit depth {}", p.bit_depth());
                                                continue;
                                            }
                                        }
                                    }
                                    _ => {
                                        warn!("Unsupported pixel layout");
                                        continue;
                                    }
                                };
                            }
                        }
                    }
                } else {
                    warn!("Unsupported data type {}", data.data_type());
                    continue;
                }
            }
            None => break,
            Some(_) => break,
        }
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
fn dora_dav1d(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
