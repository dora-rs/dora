#![deny(unsafe_op_in_unsafe_fn)]

use dora_node_api::{DoraNode, Input};
use eyre::Context;
use std::{ffi::c_void, ptr, slice};

struct DoraContext {
    node: &'static mut DoraNode,
    inputs: flume::Receiver<Input>,
}

/// Initializes a dora context from the environment variables that were set by
/// the dora-coordinator.
///
/// Returns a pointer to the dora context on success. This pointer can be
/// used to call dora API functions that expect a `context` argument. Any
/// other use is prohibited. To free the dora context when it is no longer
/// needed, use the [`free_dora_context`] function.
///
/// On error, a null pointer is returned.
#[no_mangle]
pub extern "C" fn init_dora_context_from_env() -> *mut c_void {
    let context = || {
        let node = DoraNode::init_from_env()?;
        let node = Box::leak(Box::new(node));
        let inputs = node.inputs()?;
        Result::<_, eyre::Report>::Ok(DoraContext { node, inputs })
    };
    let context = match context().context("failed to initialize node") {
        Ok(n) => n,
        Err(err) => {
            let err: eyre::Error = err;
            eprintln!("{err:?}");
            return ptr::null_mut();
        }
    };

    Box::into_raw(Box::new(context)).cast()
}

/// Frees the given dora context.
///
/// ## Safety
///
/// Only pointers created through [`init_dora_context_from_env`] are allowed
/// as arguments. Each context pointer must be freed exactly once. After
/// freeing, the pointer must not be used anymore.
#[no_mangle]
pub unsafe extern "C" fn free_dora_context(context: *mut c_void) {
    let context: Box<DoraContext> = unsafe { Box::from_raw(context.cast()) };
    // drop all fields except for `node`
    let DoraContext { node, .. } = *context;
    // convert the `'static` reference back to a Box, then drop it
    let _ = unsafe { Box::from_raw(node as *const DoraNode as *mut DoraNode) };
}

/// Waits for the next incoming input for the node.
///
/// Returns a pointer to the input on success. This pointer must not be used
/// directly. Instead, use the `read_dora_input_*` functions to read out the
/// ID and data of the input. When the input is not needed anymore, use
/// [`free_dora_input`] to free it again.
///
/// Returns a null pointer when all input streams were closed. This means that
/// no more input will be available. Nodes typically react by stopping.
///
/// ## Safety
///
/// The `context` argument must be a dora context created through
/// [`init_dora_context_from_env`]. The context must be still valid, i.e., not
/// freed yet.
#[no_mangle]
pub unsafe extern "C" fn dora_next_input(context: *mut c_void) -> *mut c_void {
    let context: &mut DoraContext = unsafe { &mut *context.cast() };
    match context.inputs.recv() {
        Ok(input) => Box::into_raw(Box::new(input)).cast(),
        Err(flume::RecvError::Disconnected) => ptr::null_mut(),
    }
}

/// Reads out the ID of the given input.
///
/// Writes the `out_ptr` and `out_len` with the start pointer and length of the
/// ID string of the input. The ID is guaranteed to be valid UTF-8.
///
/// ## Safety
///
/// The `input` argument must be a dora input received through
/// [`dora_next_input`]. The input must be still valid, i.e., not
/// freed yet. The returned `out_ptr` must not be used after
/// freeing the `input`, since it points directly into the input's
/// memory.
#[no_mangle]
pub unsafe extern "C" fn read_dora_input_id(
    input: *const (),
    out_ptr: *mut *const u8,
    out_len: *mut usize,
) {
    let input: &Input = unsafe { &*input.cast() };
    let id = input.id.as_str().as_bytes();
    let ptr = id.as_ptr();
    let len = id.len();
    unsafe {
        *out_ptr = ptr;
        *out_len = len;
    }
}

/// Reads out the data of the given input.
///
/// Writes the `out_ptr` and `out_len` with the start pointer and length of the
/// input's data array. The data array is a raw byte array, whose format
/// depends on the source operator/node.
///
/// ## Safety
///
/// The `input` argument must be a dora input received through
/// [`dora_next_input`]. The input must be still valid, i.e., not
/// freed yet. The returned `out_ptr` must not be used after
/// freeing the `input`, since it points directly into the input's
/// memory.
#[no_mangle]
pub unsafe extern "C" fn read_dora_input_data(
    input: *const (),
    out_ptr: *mut *const u8,
    out_len: *mut usize,
) {
    let input: &Input = unsafe { &*input.cast() };
    let data = &input.message.data;
    let ptr = data.as_ptr();
    let len = data.len();
    unsafe {
        *out_ptr = ptr;
        *out_len = len;
    }
}

/// Frees the given dora input.
///
/// ## Safety
///
/// Only pointers created through [`dora_next_input`] are allowed
/// as arguments. Each context pointer must be freed exactly once. After
/// freeing, the pointer and all derived pointers must not be used anymore.
/// This also applies to the `read_dora_input_*` functions, which return
/// pointers into the original input structure.
#[no_mangle]
pub unsafe extern "C" fn free_dora_input(input: *mut c_void) {
    let _: Box<Input> = unsafe { Box::from_raw(input.cast()) };
}

/// Sends the given output to subscribed dora nodes/operators.
///
/// The `id_ptr` and `id_len` fields must be the start pointer and length of an
/// UTF8-encoded string. The ID string must correspond to one of the node's
/// outputs specified in the dataflow YAML file.
///
/// The `data_ptr` and `data_len` fields must be the start pointer and length
/// a byte array. The dora API sends this data as-is, without any processing.
///
/// ## Safety
///
/// - The `id_ptr` and `id_len` fields must be the start pointer and length of an
///   UTF8-encoded string.
/// - The `data_ptr` and `data_len` fields must be the start pointer and length
///   a byte array.
#[no_mangle]
pub unsafe extern "C" fn dora_send_output(
    context: *mut c_void,
    id_ptr: *const u8,
    id_len: usize,
    data_ptr: *const u8,
    data_len: usize,
) -> isize {
    match unsafe { try_send_output(context, id_ptr, id_len, data_ptr, data_len) } {
        Ok(()) => 0,
        Err(err) => {
            eprintln!("{err:?}");
            -1
        }
    }
}

unsafe fn try_send_output(
    context: *mut c_void,
    id_ptr: *const u8,
    id_len: usize,
    data_ptr: *const u8,
    data_len: usize,
) -> eyre::Result<()> {
    let context: &mut DoraContext = unsafe { &mut *context.cast() };
    let id = std::str::from_utf8(unsafe { slice::from_raw_parts(id_ptr, id_len) })?;
    let output_id = id.to_owned().into();
    let data = unsafe { slice::from_raw_parts(data_ptr, data_len) };
    context.node.send_output(&output_id, &data.into())
}
