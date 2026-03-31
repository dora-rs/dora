//! The operator API is a framework to implement adora operators.
//! The implemented operator will be managed by `adora`.
//!
//! This framework enable us to make optimisation and provide advanced features.
//! It is the recommended way of using `adora`.
//!
//! An operator requires to be registered and implement the `AdoraOperator` trait.
//! It is composed of an `on_event` method that defines the behaviour
//! of the operator when there is an event such as receiving an input for example.
//!
//! Try it out with:
//!
//! ```bash
//! adora new op --kind operator
//! ```
//!

#![warn(unsafe_op_in_unsafe_fn)]
#![allow(clippy::missing_safety_doc)]

pub use adora_arrow_convert::*;
pub use adora_operator_api_macros::register_operator;
pub use adora_operator_api_types as types;
pub use types::AdoraStatus;
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

pub trait AdoraOperator: Default {
    #[allow(clippy::result_unit_err)] // we use a () error type only for testing
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut AdoraOutputSender,
    ) -> Result<AdoraStatus, String>;
}

pub struct AdoraOutputSender<'a>(&'a SendOutput);

// Backward-compatible aliases for dora-hub operators.
// Trait and struct renames (DoraOperator, DoraOutputSender, DoraStatus) are
// provided via the `dora-operator-api` shim crate under apis/rust/compat/.
pub use AdoraStatus as DoraStatus;
pub type DoraOutputSender<'a> = AdoraOutputSender<'a>;

impl AdoraOutputSender<'_> {
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
