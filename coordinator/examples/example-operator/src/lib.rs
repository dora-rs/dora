#![warn(unsafe_op_in_unsafe_fn)]

use std::{ffi::c_void, slice, sync::Mutex};

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

    match on_input(id, data, &mut output_sender) {
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

fn on_input(id: &str, data: &[u8], output_sender: &mut DoraOutputSender) -> Result<(), ()> {
    static TIME: once_cell::sync::Lazy<Mutex<Option<String>>> =
        once_cell::sync::Lazy::new(|| Mutex::new(None));

    let mut time = TIME.lock().unwrap();

    match id {
        "time" => {
            let parsed = std::str::from_utf8(data).map_err(|_| ())?;
            *time = Some(parsed.to_owned());
        }
        "random" => {
            let parsed = {
                let data: [u8; 8] = data.try_into().map_err(|_| ())?;
                u64::from_le_bytes(data)
            };
            if let Some(time) = &*time {
                let output = format!("operator random value {parsed} at {time}");
                output_sender
                    .send("timestamped-random", output.as_bytes())
                    .map_err(|_| ())?;
            }
        }
        other => eprintln!("ignoring unexpected input {other}"),
    }
    Ok(())
}
