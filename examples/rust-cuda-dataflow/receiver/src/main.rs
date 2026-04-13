use arrow_rs_cuda::{CudaDeviceManager, CudaIpcMemHandle};
use dora_node_api::{
    arrow::array::{Array, Int8Array},
    DoraNode, Event, MetadataParameters, Parameter,
};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;

    let manager = CudaDeviceManager::instance()?;
    let ctx = manager.get_context(0)?;
    println!("CUDA context ready on device 0");

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata,
                data,
            } => {
                if id.as_str() != "cuda_data" {
                    continue;
                }

                // Extract the 64-byte IPC handle from the Arrow Int8 array
                let int8_array = data
                    .as_any()
                    .downcast_ref::<Int8Array>()
                    .ok_or_else(|| eyre::eyre!("expected Int8Array"))?;
                let values = int8_array.values();
                let handle_bytes = unsafe {
                    std::slice::from_raw_parts(values.as_ptr() as *const u8, values.len())
                };

                // Read metadata
                let size = match metadata.parameters.get("size") {
                    Some(Parameter::Integer(v)) => *v,
                    _ => eyre::bail!("missing or invalid 'size' metadata"),
                };
                let shape = match metadata.parameters.get("shape") {
                    Some(Parameter::ListInt(v)) => v.clone(),
                    _ => eyre::bail!("missing or invalid 'shape' metadata"),
                };
                let dtype = match metadata.parameters.get("dtype") {
                    Some(Parameter::String(v)) => v.clone(),
                    _ => eyre::bail!("missing or invalid 'dtype' metadata"),
                };

                println!(
                    "received CUDA IPC handle ({} bytes), buffer size={}, dtype={}, shape={:?}",
                    handle_bytes.len(),
                    size,
                    dtype,
                    shape,
                );

                // Open the IPC handle to get zero-copy access to sender's GPU memory
                let ipc_handle = CudaIpcMemHandle::from_buffer(handle_bytes)
                    .wrap_err("failed to create IPC handle from buffer")?;
                let cuda_buffer = ctx
                    .open_ipc_buffer(&ipc_handle)
                    .wrap_err("failed to open IPC buffer")?;

                // Copy first 10 int64 values to host and print
                let copy_bytes = std::cmp::min(size, 10 * 8) as usize;
                let mut host_buf = vec![0u8; copy_bytes];
                cuda_buffer
                    .copy_to_host(0, &mut host_buf)
                    .wrap_err("failed to copy to host")?;

                let num_values = copy_bytes / std::mem::size_of::<i64>();
                let values =
                    unsafe { std::slice::from_raw_parts(host_buf.as_ptr() as *const i64, num_values) };
                print!("first {} values:", num_values);
                for v in values {
                    print!(" {}", v);
                }
                println!();

                // Send "next" to trigger the sender again
                node.send_output(
                    "next".into(),
                    MetadataParameters::default(),
                    dora_node_api::arrow::array::UInt8Array::from(vec![0u8]),
                )?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    println!("receiver done");
    Ok(())
}
