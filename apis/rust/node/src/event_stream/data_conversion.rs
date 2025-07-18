use std::{ptr::NonNull, sync::Arc};

use aligned_vec::{AVec, ConstAlign};
use dora_arrow_convert::IntoArrow;
use dora_message::metadata::ArrowTypeInfo;
use eyre::Context;
use shared_memory_server::{Shmem, ShmemConf};

use crate::arrow_utils::buffer_into_arrow_array;

pub enum RawData {
    Empty,
    Vec(AVec<u8, ConstAlign<128>>),
    SharedMemory(SharedMemoryData),
}

impl RawData {
    pub fn into_arrow_array(
        self,
        type_info: &ArrowTypeInfo,
    ) -> eyre::Result<arrow::array::ArrayData> {
        let raw_buffer = match self {
            RawData::Empty => return Ok(().into_arrow().into()),
            RawData::Vec(data) => {
                let ptr = NonNull::new(data.as_ptr() as *mut _).unwrap();
                let len = data.len();

                unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data)) }
            }
            RawData::SharedMemory(data) => {
                let ptr = NonNull::new(data.data.as_ptr() as *mut _).unwrap();
                let len = data.data.len();

                unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data)) }
            }
        };

        buffer_into_arrow_array(&raw_buffer, type_info)
    }
}

impl std::fmt::Debug for RawData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
}

pub struct SharedMemoryData {
    pub data: MappedInputData,
    pub _drop: flume::Sender<()>,
}

pub struct MappedInputData {
    memory: Box<Shmem>,
    len: usize,
}

impl MappedInputData {
    pub(crate) unsafe fn map(shared_memory_id: &str, len: usize) -> eyre::Result<Self> {
        let memory = Box::new(
            ShmemConf::new()
                .os_id(shared_memory_id)
                .writable(false)
                .open()
                .wrap_err("failed to map shared memory input")?,
        );
        Ok(MappedInputData { memory, len })
    }
}

impl std::ops::Deref for MappedInputData {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { &self.memory.as_slice()[..self.len] }
    }
}

unsafe impl Send for MappedInputData {}
unsafe impl Sync for MappedInputData {}
