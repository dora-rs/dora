#![warn(unsafe_op_in_unsafe_fn)]
#![allow(clippy::missing_safety_doc)]

pub use dora_operator_api_macros::register_operator;
pub use dora_operator_api_types as types;
pub use types::DoraStatus;
use types::{Metadata, Output, OutputMetadata, PrepareOutput};

pub mod raw;

pub trait DoraOperator: Default {
    #[allow(clippy::result_unit_err)] // we use a () error type only for testing
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, String>;
}

pub struct DoraOutputSender<'a>(&'a PrepareOutput);

impl DoraOutputSender<'_> {
    pub fn prepare(&mut self, id: String, data_len: usize) -> Result<DoraOutput, String> {
        let result = self.0.prepare_output.call(OutputMetadata {
            id: id.into(),
            metadata: Metadata {
                open_telemetry_context: String::new().into(), // TODO
            },
            data_len,
        });
        match result.result.error {
            None => Ok(DoraOutput(result.output)),
            Some(error) => Err(error.into()),
        }
    }
}

#[must_use]
pub struct DoraOutput(Output);

impl DoraOutput {
    pub fn data_mut(&mut self) -> &mut [u8] {
        let raw = self.0.data_mut.call();
        unsafe { std::slice::from_raw_parts_mut(raw.ptr.as_ptr(), raw.len) }
    }

    pub fn send(mut self) -> Result<(), String> {
        let result = self.0.send.call();
        match result.error {
            None => Ok(()),
            Some(error) => Err(error.into()),
        }
    }
}
