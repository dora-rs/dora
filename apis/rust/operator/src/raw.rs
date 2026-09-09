//! Low-level FFI entry points backing the [`register_operator!`] macro.
//!
//! [`register_operator!`] generates the three `extern "C"` shims the dora
//! runtime calls — `dora_init_operator`, `dora_drop_operator`, and
//! `dora_on_event` — and forwards each to the matching function here,
//! monomorphized for the user's operator type. These functions are `unsafe`
//! because they move ownership of the operator across the C ABI boundary as a
//! raw `*mut c_void`; user code should never call them directly.
//!
//! [`register_operator!`]: crate::register_operator

use crate::{DoraOperator, DoraOutputSender, DoraStatus, Event};
use dora_operator_api_types::{
    DoraInitResult, DoraResult, OnEventResult, RawEvent, SendOutput, arrow,
};
use std::ffi::c_void;

/// Construct the operator (`O::default()`), box it, and hand ownership back to
/// the runtime as an opaque `operator_context` pointer.
///
/// # Safety
///
/// The returned `operator_context` points to a leaked `Box<O>`. The caller must
/// eventually pass it to [`dora_drop_operator::<O>`] exactly once (with the same
/// `O`) to reclaim it, and must not use it after that.
pub unsafe fn dora_init_operator<O: DoraOperator>() -> DoraInitResult {
    let operator: O = Default::default();
    let ptr: *mut O = Box::leak(Box::new(operator));
    let operator_context: *mut c_void = ptr.cast();
    DoraInitResult {
        result: DoraResult { error: None },
        operator_context,
    }
}

/// Reclaim and drop the operator behind `operator_context`.
///
/// # Safety
///
/// `operator_context` must be a pointer previously returned by
/// [`dora_init_operator::<O>`] for the same `O`, and must not have been dropped
/// already. After this call the pointer is dangling and must not be reused.
pub unsafe fn dora_drop_operator<O>(operator_context: *mut c_void) -> DoraResult {
    let raw: *mut O = operator_context.cast();
    drop(unsafe { Box::from_raw(raw) });
    DoraResult { error: None }
}

/// Dispatch one runtime event to the operator's [`DoraOperator::on_event`].
///
/// A panic in the user's `on_event` is caught and reported as an operator error
/// (with `DoraStatus::Stop`) rather than being allowed to unwind across the C
/// ABI boundary, which would abort the process.
///
/// # Safety
///
/// `operator_context` must be a valid pointer returned by
/// [`dora_init_operator::<O>`] for the same `O` and not yet dropped. `event` and
/// `send_output` must be valid for the duration of the call.
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
                data: dora_arrow_convert::internal::from_array_data(data),
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
