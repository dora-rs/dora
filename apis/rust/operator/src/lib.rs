#![warn(unsafe_op_in_unsafe_fn)]
#![allow(clippy::missing_safety_doc)]

pub use dora_operator_api_macros::register_operator;
use raw::OutputFnRaw;
use std::ffi::c_void;

pub mod raw;

pub trait DoraOperator: Default {
    #[allow(clippy::result_unit_err)] // we use a () error type only for testing
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, ()>;
}

#[repr(isize)]
pub enum DoraStatus {
    Continue = 0,
    Stop = 1,
}

pub struct DoraOutputSender {
    output_fn_raw: OutputFnRaw,
    output_context: *const c_void,
}

impl DoraOutputSender {
    pub fn send(&mut self, id: &str, data: &[u8]) -> Result<(), isize> {
        println!("operator sending output..");
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
