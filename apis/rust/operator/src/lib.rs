#![warn(unsafe_op_in_unsafe_fn)]
#![allow(clippy::missing_safety_doc)]

use std::ffi::c_void;

pub use dora_node_api::config::DataId;
pub use dora_node_api::Metadata;
pub use dora_operator_api_macros::register_operator;
use raw::OutputFnRaw;
pub mod raw;

pub trait DoraOperator: Default {
    #[allow(clippy::result_unit_err)] // we use a () error type only for testing
    fn on_input(
        &mut self,
        metadata: &Metadata,
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
    pub fn send_output(&self, metadata: Metadata, data: &[u8]) -> Result<(), isize> {
        let ptr: *mut Metadata = Box::leak(Box::new(metadata));
        let type_erased_ptr: *mut c_void = ptr.cast();
        let result = unsafe {
            (self.output_fn_raw)(
                type_erased_ptr,
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
}
