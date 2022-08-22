use std::{ffi::c_void, slice};

use crate::{DoraContext, DoraOperator};

pub type OutputFnRaw = unsafe extern "C" fn(
    metadata: *const c_void,
    data_start: *const u8,
    data_len: usize,
    dora_context: *const c_void,
) -> isize;

pub unsafe fn dora_init_operator<O: DoraOperator>(operator_context: *mut *mut c_void) -> isize {
    let operator: O = Default::default();
    let ptr: *mut O = Box::leak(Box::new(operator));
    let type_erased: *mut c_void = ptr.cast();
    unsafe { *operator_context = type_erased };
    0
}

pub unsafe fn dora_drop_operator<O>(operator_context: *mut c_void) {
    let raw: *mut O = operator_context.cast();
    unsafe { Box::from_raw(raw) };
}

pub unsafe fn dora_on_input<O: DoraOperator>(
    metadata: *const c_void,
    data_start: *const u8,
    data_len: usize,
    output_fn_raw: OutputFnRaw,
    dora_context: *const c_void,
    operator_context: *mut c_void,
) -> isize {
    let metadata = unsafe { &*metadata.cast() };
    let data = unsafe { slice::from_raw_parts(data_start, data_len) };
    let mut output_sender = DoraContext {
        output_fn_raw,
        dora_context,
    };

    let operator: &mut O = unsafe { &mut *operator_context.cast() };

    match operator.on_input(metadata, data, &mut output_sender) {
        Ok(status) => status as isize,
        Err(_) => -1,
    }
}
