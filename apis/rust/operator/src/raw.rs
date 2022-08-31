use crate::{DoraOperator, DoraOutputSender, DoraStatus};
use dora_operator_api_types::{DoraInitResult, DoraResult, Input, OnInputResult, SendOutput};
use std::ffi::c_void;

pub type OutputFnRaw = unsafe extern "C" fn(
    id_start: *const u8,
    id_len: usize,
    data_start: *const u8,
    data_len: usize,
    output_context: *const c_void,
) -> isize;

pub unsafe fn dora_init_operator<O: DoraOperator>() -> DoraInitResult {
    let operator: O = Default::default();
    let ptr: *mut O = Box::leak(Box::new(operator));
    let operator_context: *mut c_void = ptr.cast();
    DoraInitResult {
        result: DoraResult { error: None },
        operator_context,
    }
}

pub unsafe fn dora_drop_operator<O>(operator_context: *mut c_void) -> DoraResult {
    let raw: *mut O = operator_context.cast();
    unsafe { Box::from_raw(raw) };
    DoraResult { error: None }
}

pub unsafe fn dora_on_input<O: DoraOperator>(
    input: &Input,
    send_output: SendOutput,
    operator_context: *mut std::ffi::c_void,
) -> OnInputResult {
    let mut output_sender = DoraOutputSender(send_output);

    let operator: &mut O = unsafe { &mut *operator_context.cast() };
    let data = input.data.as_ref().as_slice();
    match operator.on_input(&input.id, &data, &mut output_sender) {
        Ok(status) => OnInputResult {
            result: DoraResult { error: None },
            status,
        },
        Err(error) => OnInputResult {
            result: DoraResult {
                error: Some(error.into()),
            },
            status: DoraStatus::Stop,
        },
    }
}
