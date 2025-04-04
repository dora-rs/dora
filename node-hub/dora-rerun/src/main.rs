// main.rs - Final Integrated Version
use std::env;
use eyre::{Result, WrapErr};
use tracing_subscriber;
use dora_node_api::{Node, Event};
use arrow::array::{UInt8Array, Float32Array, StringArray};
use rerun::{
    RecordingStreamBuilder, TextLog, Image, Points3D, LineStrips3D,
    TensorData, TensorDimension, TensorBuffer, ArrowBuffer,
};

fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    // Spawn the Rerun viewer
    let mut rec = RecordingStreamBuilder::new("dora-rerun")
        .spawn()
        .wrap_err("Could not spawn Rerun visualization")?;
    
    // Create the Dora node instance
    let mut node = Node::new("dora-rerun");

    // Main event loop for processing incoming events
    for event in node {
        if let Event::Input { id, data } = event {
            let id_str = id.as_str();

            // Process text log messages
            if id_str.contains("textlog") {
                let buffer: StringArray = data.to_data().into();
                for opt_msg in buffer.iter() {
                    if let Some(msg) = opt_msg {
                        rec.log(id_str, &TextLog::new(msg))
                            .wrap_err("Could not log text message")?;
                    }
                }
            }
            // Process image inputs
            else if id_str.contains("image") {
                // Retrieve image dimensions from environment variables
                let prefix = id_str.to_uppercase(); // e.g., "IMAGE"
                let width: usize = env::var(format!("{}_WIDTH", prefix))
                    .wrap_err("Missing WIDTH env variable")?.parse()
                    .wrap_err("Invalid WIDTH value")?;
                let height: usize = env::var(format!("{}_HEIGHT", prefix))
                    .wrap_err("Missing HEIGHT env variable")?.parse()
                    .wrap_err("Invalid HEIGHT value")?;
                let depth: usize = env::var(format!("{}_DEPTH", prefix))
                    .wrap_err("Missing DEPTH env variable")?.parse()
                    .wrap_err("Invalid DEPTH value")?;

                let buffer: UInt8Array = data.to_data().into();
                let bytes: &[u8] = buffer.values();
                let tensor = TensorData::new(
                    vec![
                        TensorDimension::new("height", height),
                        TensorDimension::new("width", width),
                        TensorDimension::new("depth", depth),
                    ],
                    TensorBuffer::U8(ArrowBuffer::from(bytes)),
                );
                let image = Image::new(tensor);
                rec.log(id_str, &image)
                    .wrap_err("Could not log image")?;
            }
            // Process LiDAR point cloud inputs
            else if id_str.contains("pointcloud") {
                let buffer: Float32Array = data.to_data().into();
                let values: &[f32] = buffer.values();
                if values.len() % 3 != 0 {
                    tracing::warn!("Point cloud data length is not a multiple of 3");
                    continue;
                }
                let mut points = Vec::with_capacity(values.len() / 3);
                for chunk in values.chunks(3) {
                    let point: [f32; 3] = [chunk[0], chunk[1], chunk[2]];
                    points.push(point);
                }
                rec.log(id_str, &Points3D::new(points))
                    .wrap_err("Could not log point cloud")?;
            }
            // Process trajectory inputs
            else if id_str.contains("trajectory") {
                let buffer: Float32Array = data.to_data().into();
                let coords: &[f32] = buffer.values();
                if coords.len() % 3 != 0 {
                    tracing::warn!("Trajectory data length is not divisible by 3");
                    continue;
                }
                let mut waypoints = Vec::new();
                for chunk in coords.chunks(3) {
                    let point: [f32; 3] = [chunk[0], chunk[1], chunk[2]];
                    waypoints.push(point);
                }
                let line_strip = LineStrips3D::new([waypoints]);
                rec.log(id_str, &line_strip)
                    .wrap_err("Could not log trajectory")?;
            }
            // Process robot arm pose inputs
            else if id_str.contains("arm") {
                let buffer: Float32Array = data.to_data().into();
                let coords: &[f32] = buffer.values();
                if coords.len() % 3 != 0 {
                    tracing::warn!("Arm data length is not divisible by 3");
                    continue;
                }
                let mut joints = Vec::new();
                for chunk in coords.chunks(3) {
                    let joint: [f32; 3] = [chunk[0], chunk[1], chunk[2]];
                    joints.push(joint);
                }
                let arm_skeleton = LineStrips3D::new([joints]);
                rec.log(id_str, &arm_skeleton)
                    .wrap_err("Could not log arm pose")?;
            }
        }
    }

    Ok(())
}
