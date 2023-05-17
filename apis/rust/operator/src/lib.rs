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

pub use dora_operator_api_macros::register_operator;
pub use dora_operator_api_types as types;
pub use types::DoraStatus;
use types::{Metadata, Output, SendOutput};

pub mod raw;

#[derive(Debug)]
#[non_exhaustive]
pub enum Event<'a> {
    Input { id: &'a str, data: &'a [u8] },
    InputClosed { id: &'a str },
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
    pub fn send(&mut self, id: String, data: Vec<u8>) -> Result<(), String> {
        let result = self.0.send_output.call(Output {
            id: id.into(),
            data: data.into(),
            metadata: Metadata {
                open_telemetry_context: String::new().into(), // TODO
            },
        });
        match result.error {
            None => Ok(()),
            Some(error) => Err(error.into()),
        }
    }
}
