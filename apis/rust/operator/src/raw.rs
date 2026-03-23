use crate::{AdoraOperator, AdoraOutputSender, AdoraStatus, Event};
use adora_operator_api_types::{
    AdoraInitResult, AdoraResult, OnEventResult, RawEvent, SendOutput, arrow,
};
use std::ffi::c_void;

pub type OutputFnRaw = unsafe extern "C" fn(
    id_start: *const u8,
    id_len: usize,
    data_start: *const u8,
    data_len: usize,
    output_context: *const c_void,
) -> isize;

pub unsafe fn adora_init_operator<O: AdoraOperator>() -> AdoraInitResult {
    let operator: O = Default::default();
    let ptr: *mut O = Box::leak(Box::new(operator));
    let operator_context: *mut c_void = ptr.cast();
    AdoraInitResult {
        result: AdoraResult { error: None },
        operator_context,
    }
}

pub unsafe fn adora_drop_operator<O>(operator_context: *mut c_void) -> AdoraResult {
    let raw: *mut O = operator_context.cast();
    drop(unsafe { Box::from_raw(raw) });
    AdoraResult { error: None }
}

pub unsafe fn adora_on_event<O: AdoraOperator>(
    event: &mut RawEvent,
    send_output: &SendOutput,
    operator_context: *mut std::ffi::c_void,
) -> OnEventResult {
    let mut output_sender = AdoraOutputSender(send_output);

    let operator: &mut O = unsafe { &mut *operator_context.cast() };

    let event_variant = if let Some(input) = &mut event.input {
        let Some(data_array) = input.data_array.take() else {
            return OnEventResult {
                result: AdoraResult::from_error("data already taken".to_string()),
                status: AdoraStatus::Continue,
            };
        };
        let data = unsafe { arrow::ffi::from_ffi(data_array, &input.schema) };

        match data {
            Ok(data) => Event::Input {
                id: &input.id,
                metadata: &input.metadata,
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
            result: AdoraResult { error: None },
            status: AdoraStatus::Continue,
        };
    };
    match operator.on_event(&event_variant, &mut output_sender) {
        Ok(status) => OnEventResult {
            result: AdoraResult { error: None },
            status,
        },
        Err(error) => OnEventResult {
            result: AdoraResult::from_error(error),
            status: AdoraStatus::Stop,
        },
    }
}
