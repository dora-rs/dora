#![warn(unsafe_op_in_unsafe_fn)]

use std::{ffi::c_void, slice};

type OutputFn = unsafe extern "C" fn(
    id_start: *const u8,
    id_len: usize,
    data_start: *const u8,
    data_len: usize,
    output_context: *const c_void,
) -> isize;

#[no_mangle]
pub unsafe extern "C" fn dora_on_input(
    id_start: *const u8,
    id_len: usize,
    data_start: *const u8,
    data_len: usize,
    output: OutputFn,
    output_context: *const c_void,
) -> isize {
    let id = match std::str::from_utf8(unsafe { slice::from_raw_parts(id_start, id_len) }) {
        Ok(id) => id,
        Err(_) => return -1,
    };
    let data = unsafe { slice::from_raw_parts(data_start, data_len) };
    match on_input(id, data, output, output_context) {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

pub fn on_input(
    id: &str,
    data: &[u8],
    output: OutputFn,
    output_context: *const c_void,
) -> Result<(), ()> {
    println!(
        "operator got input `{id}` with value: {}",
        String::from_utf8_lossy(data)
    );
    Ok(())
}
