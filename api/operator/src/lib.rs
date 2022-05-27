use std::ffi::c_void;

pub trait DoraOperator {
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<(), ()>;
}

pub struct DoraOutputSender {
    pub output_fn_raw: OutputFnRaw,
    pub output_context: *const c_void,
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

pub type OutputFnRaw = unsafe extern "C" fn(
    id_start: *const u8,
    id_len: usize,
    data_start: *const u8,
    data_len: usize,
    output_context: *const c_void,
) -> isize;
