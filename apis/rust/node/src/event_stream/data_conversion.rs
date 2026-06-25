use std::{ptr::NonNull, sync::Arc};

use aligned_vec::{AVec, ConstAlign};
use dora_arrow_convert::IntoArrow;

use crate::arrow_utils::decode_arrow_ipc_zero_copy;

pub enum RawData {
    Empty,
    Vec(AVec<u8, ConstAlign<128>>),
}

impl RawData {
    pub fn into_arrow_array(self) -> eyre::Result<arrow::array::ArrayData> {
        match self {
            // A metadata-only message with no payload. Any real array is a
            // non-empty IPC stream, so an empty payload maps to the unit array.
            RawData::Empty => Ok(().into_arrow().into()),
            RawData::Vec(data) => {
                // Wrap the 128-byte-aligned `AVec` as an Arrow `Buffer` without
                // copying (the buffer aliases the allocation), then let the
                // zero-copy IPC decoder slice the array buffers out of it in
                // place (it realigns internally only if under-aligned).
                let ptr = NonNull::new(data.as_ptr() as *mut _).unwrap();
                let len = data.len();
                let raw_buffer = unsafe {
                    arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data))
                };
                decode_arrow_ipc_zero_copy(raw_buffer)
            }
        }
    }
}

impl std::fmt::Debug for RawData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
}
