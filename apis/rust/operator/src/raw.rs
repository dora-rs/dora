use crate::{DoraOperator, DoraOutputSender, DoraStatus, Event};
use dora_operator_api_types::{
    arrow, DoraInitResult, DoraResult, OnEventResult, RawEvent, SendOutput,
};
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
    drop(unsafe { Box::from_raw(raw) });
    DoraResult { error: None }
}

pub unsafe fn dora_on_event<O: DoraOperator>(
    event: &mut RawEvent,
    send_output: &SendOutput,
    operator_context: *mut std::ffi::c_void,
) -> OnEventResult {
    let mut output_sender = DoraOutputSender(send_output);

    let operator: &mut O = unsafe { &mut *operator_context.cast() };

    let event_variant = if let Some(input) = &mut event.input {
        let Some(data_array) = input.data_array.take() else {
            return OnEventResult {
                result: DoraResult::from_error("data already taken".to_string()),
                status: DoraStatus::Continue,
            };
        };
        let data = arrow::ffi::from_ffi(data_array, &input.schema);

        match data {
            Ok(data) => Event::Input {
                id: &input.id,
                data: arrow::array::make_array(data).into(),
            },
            Err(err) => Event::InputParseError {
                id: &input.id,
                error: format!("{err}"),
            },
        }
    } else if let Some(input_id) = &event.input_closed {
        Event::InputClosed { id: input_id }
    } else if event.stop {
        Event::Stop
    } else {
        // ignore unknown events
        return OnEventResult {
            result: DoraResult { error: None },
            status: DoraStatus::Continue,
        };
    };
    match operator.on_event(&event_variant, &mut output_sender) {
        Ok(status) => OnEventResult {
            result: DoraResult { error: None },
            status,
        },
        Err(error) => OnEventResult {
            result: DoraResult::from_error(error),
            status: DoraStatus::Stop,
        },
    }
}
