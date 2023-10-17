use std::{ptr::NonNull, sync::Arc};

use dora_arrow_convert::ArrowData;
use dora_core::{
    config::{DataId, OperatorId},
    message::{ArrowTypeInfo, BufferOffset, Metadata},
};
use eyre::{Context, Result};
use shared_memory_extended::{Shmem, ShmemConf};

#[derive(Debug)]
#[non_exhaustive]
pub enum Event {
    Stop,
    Reload {
        operator_id: Option<OperatorId>,
    },
    Input {
        id: DataId,
        metadata: Metadata,
        data: ArrowData,
    },
    InputClosed {
        id: DataId,
    },
    Error(String),
}

pub(super) enum Data {
    Vec(Vec<u8>),
    SharedMemory {
        data: MappedInputData,
        _drop: flume::Sender<()>,
    },
}

impl Data {
    pub fn into_arrow_array(
        self: Arc<Self>,
        type_info: &ArrowTypeInfo,
    ) -> Result<arrow::array::ArrayData> {
        let ptr = NonNull::new(self.as_ptr() as *mut _).unwrap();
        let len = self.len();

        let raw_buffer = unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, self) };

        buffer_into_arrow_array(&raw_buffer, type_info)
    }
}

fn buffer_into_arrow_array(
    raw_buffer: &arrow::buffer::Buffer,
    type_info: &ArrowTypeInfo,
) -> std::result::Result<arrow::array::ArrayData, eyre::Error> {
    let mut buffers = Vec::new();
    for BufferOffset { offset, len } in &type_info.buffer_offsets {
        buffers.push(raw_buffer.slice_with_length(*offset, *len));
    }

    let mut child_data = Vec::new();
    for child_type_info in &type_info.child_data {
        child_data.push(buffer_into_arrow_array(raw_buffer, child_type_info)?)
    }

    arrow::array::ArrayData::try_new(
        type_info.data_type.clone(),
        type_info.len,
        type_info
            .validity
            .clone()
            .map(arrow::buffer::Buffer::from_vec),
        type_info.offset,
        buffers,
        child_data,
    )
    .context("Error creating Arrow Array")
}

impl std::ops::Deref for Data {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        match self {
            Data::SharedMemory { data, .. } => data,
            Data::Vec(data) => data,
        }
    }
}

impl std::fmt::Debug for Data {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
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
