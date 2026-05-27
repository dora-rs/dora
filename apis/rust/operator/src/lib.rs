//! The operator API is a framework to implement dora operators.
//! The implemented operator will be managed by `dora`.
//!
//! This framework enable us to make optimisation and provide advanced features.
//! It is the recommended way of using `dora`.
//!
//! An operator requires to be registered and implement the `DoraOperator` trait.
//! It is composed of an `on_event` method that defines the behaviour
//! of the operator when there is an event such as receiving an input for example.
//!
//! Try it out with:
//!
//! ```bash
//! dora new op --kind operator
//! ```
//!

#![warn(unsafe_op_in_unsafe_fn)]
#![allow(clippy::missing_safety_doc)]

pub use dora_arrow_convert::*;
pub use dora_operator_api_macros::register_operator;
pub use dora_operator_api_types as types;
pub use types::DoraStatus;
use types::{
    Metadata, Output, SendOutput,
    arrow::{self, array::Array},
};

pub mod raw;

#[derive(Debug)]
#[non_exhaustive]
pub enum Event<'a> {
    Input {
        id: &'a str,
        metadata: &'a types::Metadata,
        data: ArrowData,
    },
    InputParseError {
        id: &'a str,
        error: String,
    },
    InputClosed {
        id: &'a str,
    },
    Stop,
}

pub trait DoraOperator: Default {
    #[allow(clippy::result_unit_err)] // we use a () error type only for testing
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, String>;
}

pub struct DoraOutputSender<'a>(&'a SendOutput);

impl DoraOutputSender<'_> {
    ///  Send an output from the operator:
    ///  - `id` is the `output_id` as defined in your dataflow.
    ///  - `data` is the data that should be sent
    pub fn send(&mut self, id: &str, data: impl Array) -> Result<(), String> {
        let (data_array, schema) =
            arrow::ffi::to_ffi(&data.into_data()).map_err(|err| err.to_string())?;
        let result = self.0.send_output.call(Output {
            id: id.to_owned().into(),
            data_array,
            schema,
            metadata: Metadata {
                open_telemetry_context: String::new().into(), // TODO
            },
        });
        result.into_result()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[derive(Default)]
    struct RecordingOperator {
        parse_errors: Vec<(String, String)>,
    }

    impl DoraOperator for RecordingOperator {
        fn on_event(
            &mut self,
            event: &Event,
            _output_sender: &mut DoraOutputSender,
        ) -> Result<DoraStatus, String> {
            if let Event::InputParseError { id, error } = event {
                self.parse_errors.push((id.to_string(), error.clone()));
            }
            Ok(DoraStatus::Continue)
        }
    }

    fn noop_send_output() -> SendOutput {
        use types::safer_ffi::closure::ArcDynFn1;
        SendOutput {
            send_output: ArcDynFn1::new(Arc::new(|_output| types::DoraResult::SUCCESS)),
        }
    }

    #[test]
    fn input_parse_error_dispatched_to_on_event() {
        let sender = noop_send_output();
        let mut output_sender = DoraOutputSender(&sender);
        let mut op = RecordingOperator::default();

        let event = Event::InputParseError {
            id: "my-input",
            error: "bad arrow data".to_string(),
        };
        let status = op.on_event(&event, &mut output_sender).unwrap();

        assert!(matches!(status, DoraStatus::Continue));
        assert_eq!(op.parse_errors.len(), 1);
        assert_eq!(op.parse_errors[0].0, "my-input");
        assert_eq!(op.parse_errors[0].1, "bad arrow data");
    }

    #[test]
    fn other_events_do_not_trigger_parse_error_handler() {
        let sender = noop_send_output();
        let mut output_sender = DoraOutputSender(&sender);
        let mut op = RecordingOperator::default();

        let status = op.on_event(&Event::Stop, &mut output_sender).unwrap();
        assert!(matches!(status, DoraStatus::Continue));
        assert!(op.parse_errors.is_empty());
    }

    #[test]
    fn multiple_parse_errors_accumulated_in_order() {
        let sender = noop_send_output();
        let mut output_sender = DoraOutputSender(&sender);
        let mut op = RecordingOperator::default();

        let ids: Vec<String> = (0..5u32).map(|i| format!("input-{i}")).collect();
        let errors: Vec<String> = (0..5u32).map(|i| format!("corrupt-{i}")).collect();
        for i in 0..5usize {
            let event = Event::InputParseError {
                id: &ids[i],
                error: errors[i].clone(),
            };
            op.on_event(&event, &mut output_sender).unwrap();
        }

        assert_eq!(op.parse_errors.len(), 5);
        for i in 0..5usize {
            assert_eq!(op.parse_errors[i].0, ids[i]);
            assert_eq!(op.parse_errors[i].1, errors[i]);
        }
    }

    #[test]
    fn empty_id_and_error_strings_are_preserved() {
        let sender = noop_send_output();
        let mut output_sender = DoraOutputSender(&sender);
        let mut op = RecordingOperator::default();

        let event = Event::InputParseError {
            id: "",
            error: String::new(),
        };
        op.on_event(&event, &mut output_sender).unwrap();

        assert_eq!(op.parse_errors.len(), 1);
        assert_eq!(op.parse_errors[0].0, "");
        assert_eq!(op.parse_errors[0].1, "");
    }

    #[test]
    fn parse_error_interleaved_with_other_events_only_parse_errors_recorded() {
        let sender = noop_send_output();
        let mut output_sender = DoraOutputSender(&sender);
        let mut op = RecordingOperator::default();

        op.on_event(&Event::Stop, &mut output_sender).unwrap();
        op.on_event(
            &Event::InputParseError {
                id: "x",
                error: "oops".to_string(),
            },
            &mut output_sender,
        )
        .unwrap();
        op.on_event(&Event::InputClosed { id: "y" }, &mut output_sender)
            .unwrap();
        op.on_event(
            &Event::InputParseError {
                id: "z",
                error: "bad".to_string(),
            },
            &mut output_sender,
        )
        .unwrap();

        assert_eq!(op.parse_errors.len(), 2);
        assert_eq!(op.parse_errors[0].0, "x");
        assert_eq!(op.parse_errors[1].0, "z");
    }

    #[test]
    fn send_output_is_send_sync() {
        // Compile-time proof that SendOutput (backed by ArcDynFn1) satisfies
        // the Send+Sync bounds required by the operator runtime, which hands
        // it across thread boundaries.
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<SendOutput>();
    }
}
