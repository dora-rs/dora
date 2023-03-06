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
