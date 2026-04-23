use std::{ptr::NonNull, sync::Arc};

use aligned_vec::{AVec, ConstAlign};
use dora_arrow_convert::IntoArrow;
use dora_message::metadata::ArrowTypeInfo;

use crate::arrow_utils::{buffer_into_arrow_array, decode_arrow_ipc};

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
        if is_ipc {
            return self.decode_ipc();
        }

        let raw_buffer = match self {
            RawData::Empty => return Ok(().into_arrow().into()),
            RawData::Vec(data) => {
                let ptr = NonNull::new(data.as_ptr() as *mut _).unwrap();
                let len = data.len();

                unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data)) }
            }
        };

        buffer_into_arrow_array(&raw_buffer, type_info)
    }

    fn decode_ipc(&self) -> eyre::Result<arrow::array::ArrayData> {
        let bytes: &[u8] = match self {
            RawData::Empty => return Ok(().into_arrow().into()),
            RawData::Vec(data) => data,
        };
        decode_arrow_ipc(bytes)
    }
}

impl std::fmt::Debug for RawData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
}
