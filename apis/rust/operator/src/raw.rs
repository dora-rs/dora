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
    let mut output_sender = DoraOutputSender::new(send_output);

    let operator: &mut O = unsafe { &mut *operator_context.cast() };

    let event_variant = if let Some(input) = &mut event.input {
        output_sender.set_open_telemetry_context(&input.metadata.open_telemetry_context);
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

#[cfg(test)]
mod tests {
    use super::dora_on_event;
    use crate::{DoraOperator, DoraOutputSender, DoraStatus, Event};
    use dora_operator_api_types::{
        DoraResult, Input, Metadata, OnEventResult, Output, RawEvent, SendOutput,
        arrow::array::{Array, UInt8Array},
        safer_ffi::closure::ArcDynFn1,
    };
    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    struct EchoOperator;

    impl DoraOperator for EchoOperator {
        fn on_event(
            &mut self,
            event: &Event,
            output_sender: &mut DoraOutputSender,
        ) -> Result<DoraStatus, String> {
            if let Event::Input { .. } = event {
                output_sender.send("out".to_string(), UInt8Array::from(vec![1_u8, 2_u8, 3_u8]))?;
            }
            Ok(DoraStatus::Continue)
        }
    }

    #[test]
    fn forwards_open_telemetry_context_to_output_metadata() {
        let received_context = Arc::new(Mutex::new(String::new()));
        let context_target = Arc::clone(&received_context);
        let send_output = SendOutput {
            send_output: ArcDynFn1::new(Arc::new(move |output: Output| {
                *context_target.lock().expect("poisoned mutex") =
                    output.metadata.open_telemetry_context.to_string();
                DoraResult::SUCCESS
            })),
        };

        let input_array = UInt8Array::from(vec![1_u8, 2_u8]);
        let (data_array, schema) =
            dora_operator_api_types::arrow::ffi::to_ffi(&input_array.to_data())
                .expect("failed to convert input to FFI");
        let mut event = RawEvent {
            input: Some(
                Box::new(Input {
                    id: "in".to_string().into(),
                    data_array: Some(data_array),
                    schema,
                    metadata: Metadata {
                        open_telemetry_context: "traceparent-context".to_string().into(),
                    },
                })
                .into(),
            ),
            input_closed: None,
            stop: false,
            error: None,
        };

        let operator_context: *mut std::ffi::c_void = Box::into_raw(Box::new(EchoOperator)).cast();
        let OnEventResult { result, status } =
            unsafe { dora_on_event::<EchoOperator>(&mut event, &send_output, operator_context) };
        unsafe { drop(Box::from_raw(operator_context.cast::<EchoOperator>())) };

        assert!(result.error.is_none());
        assert_eq!(status as u8, DoraStatus::Continue as u8);
        assert_eq!(
            *received_context.lock().expect("poisoned mutex"),
            "traceparent-context"
        );
    }
}
