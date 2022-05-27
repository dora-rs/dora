#![warn(unsafe_op_in_unsafe_fn)]

use std::{ffi::c_void, slice};

#[no_mangle]
pub unsafe extern "C" fn dora_init_operator(operator_context: *mut *mut ()) -> isize {
    let operator = Operator::default();
    let ptr: *mut Operator = Box::leak(Box::new(operator));
    let type_erased: *mut () = ptr.cast();
    unsafe { *operator_context = type_erased };
    0
}

#[no_mangle]
pub unsafe extern "C" fn dora_drop_operator(operator_context: *mut ()) {
    let raw: *mut Operator = operator_context.cast();
    unsafe { Box::from_raw(raw) };
}

type OutputFnRaw = unsafe extern "C" fn(
    id_start: *const u8,
    id_len: usize,
    data_start: *const u8,
    data_len: usize,
    output_context: *const c_void,
) -> isize;

#[no_mangle]
pub unsafe extern "C" fn dora_on_input(
    id_start: *const u8,
    id_len: usize,
    data_start: *const u8,
    data_len: usize,
    output_fn_raw: OutputFnRaw,
    output_context: *const c_void,
    operator_context: *mut (),
) -> isize {
    let id = match std::str::from_utf8(unsafe { slice::from_raw_parts(id_start, id_len) }) {
        Ok(id) => id,
        Err(_) => return -1,
    };
    let data = unsafe { slice::from_raw_parts(data_start, data_len) };
    let mut output_sender = DoraOutputSender {
        output_fn_raw,
        output_context,
    };

    let operator: &mut Operator = unsafe { &mut *operator_context.cast() };

    match operator.on_input(id, data, &mut output_sender) {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

struct DoraOutputSender {
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

#[derive(Debug, Default)]
struct Operator {
    time: Option<String>,
}

impl Operator {
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<(), ()> {
        match id {
            "time" => {
                let parsed = std::str::from_utf8(data).map_err(|_| ())?;
                self.time = Some(parsed.to_owned());
            }
            "random" => {
                let parsed = {
                    let data: [u8; 8] = data.try_into().map_err(|_| ())?;
                    u64::from_le_bytes(data)
                };
                if let Some(time) = &self.time {
                    let output = format!("state operator random value {parsed} at {time}");
                    output_sender
                        .send("timestamped-random", output.as_bytes())
                        .map_err(|_| ())?;
                }
            }
            other => eprintln!("ignoring unexpected input {other}"),
        }
        Ok(())
    }
}
