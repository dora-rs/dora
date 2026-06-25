use std::{ptr::NonNull, sync::Arc};

use aligned_vec::{AVec, ConstAlign};
use dora_arrow_convert::IntoArrow;
use dora_message::metadata::ArrowTypeInfo;

use crate::arrow_utils::{buffer_into_arrow_array, decode_arrow_ipc_zero_copy};

pub enum RawData {
    Empty,
    Vec(AVec<u8, ConstAlign<128>>),
}

impl RawData {
    pub fn into_arrow_array(
        self,
        type_info: &ArrowTypeInfo,
        is_ipc: bool,
    ) -> eyre::Result<arrow::array::ArrayData> {
        // Wrap the payload in an Arrow `Buffer` once, then dispatch. For
        // `RawData::Vec` the buffer aliases the 128-byte-aligned `AVec`
        // allocation (no copy); both the raw and IPC decoders then read the
        // array buffers straight out of it.
        let raw_buffer = match self {
            // An empty payload still carries a meaningful `type_info`. Rebuild
            // from it with an empty backing buffer so the declared length is
            // preserved (e.g. `NullArray::new(n)` keeps length `n` instead of
            // collapsing to a `NullArray(0)`, dora-rs/dora#2083). An empty IPC
            // payload has no stream to decode, so map it to the unit array.
            RawData::Empty => {
                if is_ipc {
                    return Ok(().into_arrow().into());
                }
                let empty = arrow::buffer::Buffer::from_vec(Vec::<u8>::new());
                return buffer_into_arrow_array(&empty, type_info);
            }
            RawData::Vec(data) => {
                let ptr = NonNull::new(data.as_ptr() as *mut _).unwrap();
                let len = data.len();

                unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data)) }
            }
        };

        if is_ipc {
            // Zero-copy IPC decode: `StreamDecoder` slices the array buffers
            // out of `raw_buffer` in place when it is aligned (it always is for
            // the `AVec` allocation above), copying only if under-aligned.
            return decode_arrow_ipc_zero_copy(raw_buffer);
        }

        buffer_into_arrow_array(&raw_buffer, type_info)
    }
}

impl std::fmt::Debug for RawData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
}
