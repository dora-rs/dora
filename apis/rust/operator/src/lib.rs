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
        output_sender: &mut DoraContext,
    ) -> Result<DoraStatus, ()>;
}

#[repr(isize)]
pub enum DoraStatus {
    Continue = 0,
    Stop = 1,
}

pub struct DoraContext {
    output_fn_raw: OutputFnRaw,
    dora_context: *const c_void,
}

impl DoraContext {
    pub fn send_output(&mut self, id: &str, data: &[u8]) -> Result<(), isize> {
        let result = unsafe {
            (self.output_fn_raw)(
                id.as_ptr(),
                id.len(),
                data.as_ptr(),
                data.len(),
                self.dora_context,
            )
        };
        match result {
            0 => Ok(()),
            other => Err(other),
        }
    }

    pub fn opentelemetry_context(&self) -> &str {
        let mut ptr = std::ptr::null();
        let mut len = 0;
        unsafe { raw::dora_context_get_opentelemetry(self.dora_context, &mut ptr, &mut len) };
        assert!(!ptr.is_null());
        let bytes = unsafe { std::slice::from_raw_parts(ptr, len) };
        std::str::from_utf8(bytes).unwrap()
    }
}
