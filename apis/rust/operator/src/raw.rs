use crate::{DoraOperator, DoraOutputSender, DoraStatus, Event};
use dora_operator_api_types::{
    DoraInitResult, DoraResult, OnEventResult, RawEvent, SendOutput, arrow,
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
            result: DoraResult { error: None },
            status: DoraStatus::Continue,
        };
    };
    // Catch a panic in the user's `on_event`: this function is called directly
    // from the generated `extern "C" fn dora_on_event`, and unwinding across
    // that boundary is a forced process abort. Report it as an operator error
    // instead (dora-rs/dora#2027). The runtime treats any `error` as fatal
    // regardless of `status`, so the `Stop` below just matches the normal
    // error path. `AssertUnwindSafe` is sound here because the operator is torn
    // down after an error and never re-entered.
    //
    // Caveats: this only protects under `panic = "unwind"` (the default) — an
    // operator dylib built with `panic = "abort"` aborts before the unwind is
    // caught. The default panic hook still prints to stderr in addition to the
    // structured error returned here.
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        operator.on_event(&event_variant, &mut output_sender)
    }));
    match result {
        Ok(Ok(status)) => OnEventResult {
            result: DoraResult { error: None },
            status,
        },
        Ok(Err(error)) => OnEventResult {
            result: DoraResult::from_error(error),
            status: DoraStatus::Stop,
        },
        Err(panic) => OnEventResult {
            result: DoraResult::from_error(format!(
                "operator on_event panicked: {}",
                panic_message(&*panic)
            )),
            status: DoraStatus::Stop,
        },
    }
}

/// Extract a human-readable message from a caught panic payload.
fn panic_message(panic: &(dyn std::any::Any + Send)) -> String {
    if let Some(s) = panic.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = panic.downcast_ref::<String>() {
        s.clone()
    } else {
        "<non-string panic payload>".to_string()
    }
}
