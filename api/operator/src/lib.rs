pub use dora_operator_api_macros::register_operator;
use raw::OutputFnRaw;
use std::ffi::c_void;

pub mod raw;

pub trait DoraOperator: Default {
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<(), ()>;
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
