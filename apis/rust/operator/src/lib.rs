//! Create dora operators in Rust.

#![warn(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]
#![allow(clippy::missing_safety_doc)]

pub use dora_operator_api_macros::register_operator;
use raw::OutputFnRaw;
use std::ffi::c_void;

/// The raw FFI API that dora uses to invoke operators.
pub mod raw;

/// A dora operator that can be loaded by a dora runtime.
///
/// Implement this trait to create a dora operator.
pub trait DoraOperator: Default {
    /// Invoked on every incoming input.
    ///
    /// Contains the input `id` and the corresponding `data`, in form of a raw byte array. The
    /// `output_sender` argument allows the operator to send arbitrary outputs.
    ///
    /// The return value controls the operator execution. The dora runtime treats returned
    /// errors as fatal and stops the operator with an error code. To stop the operator
    /// with a success exit code, return `Ok(DoraStatus::Stop)`. To keep the operator running,
    /// return `Ok(DoraStatus::Continue)`.
    #[allow(clippy::result_unit_err)] // we use a () error type only for testing
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, ()>;
}

/// The return value of `DoraOperator::on_input`.
///
/// Signals to the dora runtime whether the operator should stop or continue execution.
#[repr(isize)]
pub enum DoraStatus {
    /// Continue execution and wait for the next input.
    Continue = 0,
    /// Stop the operator (with a success exit code).
    Stop = 1,
}

/// Allows operators to send dora outputs, which can be consumed by other operators or nodes.
pub struct DoraOutputSender {
    output_fn_raw: OutputFnRaw,
    output_context: *const c_void,
}

impl DoraOutputSender {
    /// Send a dora output with given `id`.
    ///
    /// The `id` must be one of the output IDs listed in the dataflow definition.
    ///
    /// Returns an abstract error code on error. (We plan to make this error more
    /// specific in a future release.)
    pub fn send(&mut self, id: &str, data: &[u8]) -> Result<(), isize> {
        let result = unsafe {
            (self.output_fn_raw)(
                id.as_ptr(),
                id.len(),
                data.as_ptr(),
                data.len(),
                self.output_context,
            )
        };
        match result {
            0 => Ok(()),
            other => Err(other),
        }
    }
}
