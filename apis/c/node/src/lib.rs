#![deny(unsafe_op_in_unsafe_fn)]

use dora_node_api::{DoraNode, Input};
use futures::{executor::block_on, Stream, StreamExt};
use std::{pin::Pin, ptr, slice};

struct DoraContext {
    node: &'static DoraNode,
    inputs: Pin<Box<dyn futures::Stream<Item = Input>>>,
}

#[no_mangle]
pub extern "C" fn init_dora_context_from_env() -> *mut () {
    let context = match block_on(async {
        let node = DoraNode::init_from_env().await?;
        let node = Box::leak(Box::new(node));
        let inputs: Pin<Box<dyn Stream<Item = Input>>> = Box::pin(node.inputs().await?);
        Ok(DoraContext { node, inputs })
    }) {
        Ok(n) => n,
        Err(err) => {
            let err: eyre::Error = err;
            eprintln!("{err:?}");
            return ptr::null_mut();
        }
    };

    Box::into_raw(Box::new(context)).cast()
}

#[no_mangle]
pub unsafe extern "C" fn free_dora_context(context: *mut ()) {
    let context: Box<DoraContext> = unsafe { Box::from_raw(context.cast()) };
    // drop all fields except for `node`
    let DoraContext { node, .. } = *context;
    // convert the `'static` reference back to a Box, then drop it
    let _ = unsafe { Box::from_raw(node as *const DoraNode as *mut DoraNode) };
}

#[no_mangle]
pub unsafe extern "C" fn dora_next_input(context: *mut ()) -> *mut () {
    let context: &mut DoraContext = unsafe { &mut *context.cast() };
    match block_on(context.inputs.next()) {
        Some(input) => Box::into_raw(Box::new(input)).cast(),
        None => ptr::null_mut(),
    }
}

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

#[no_mangle]
pub unsafe extern "C" fn read_dora_input_data(
    input: *const (),
    out_ptr: *mut *const u8,
    out_len: *mut usize,
) {
    let input: &Input = unsafe { &*input.cast() };
    let data = input.data.as_slice();
    let ptr = data.as_ptr();
    let len = data.len();
    unsafe {
        *out_ptr = ptr;
        *out_len = len;
    }
}

#[no_mangle]
pub unsafe extern "C" fn free_dora_input(input: *mut ()) {
    let _: Box<Input> = unsafe { Box::from_raw(input.cast()) };
}

#[no_mangle]
pub unsafe extern "C" fn dora_send_output(
    context: *mut (),
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
    context: *mut (),
    id_ptr: *const u8,
    id_len: usize,
    data_ptr: *const u8,
    data_len: usize,
) -> eyre::Result<()> {
    let context: &mut DoraContext = unsafe { &mut *context.cast() };
    let id = std::str::from_utf8(unsafe { slice::from_raw_parts(id_ptr, id_len) })?;
    let output_id = id.to_owned().into();
    let data = unsafe { slice::from_raw_parts(data_ptr, data_len) };
    block_on(context.node.send_output(&output_id, data))
}
