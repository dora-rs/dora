use std::{ptr::NonNull, sync::Arc};

use dora_core::{
    config::{DataId, OperatorId},
    message::Metadata,
};
use eyre::Context;
use shared_memory::{Shmem, ShmemConf};

#[derive(Debug)]
#[non_exhaustive]
pub enum Event {
    Stop,
    Reload {
        operator_id: Option<OperatorId>,
    },
    Input {
        id: DataId,
        metadata: Metadata<'static>,
        data: Option<Data>,
    },
    InputClosed {
        id: DataId,
    },
    Error(String),
}

pub enum Data {
    Vec(Vec<u8>),
    SharedMemory {
        data: MappedInputData,
        _drop: flume::Sender<()>,
    },
}
impl Data {
    pub fn into_arrow_array(
        self: Arc<Self>,
    ) -> Result<arrow::array::ArrayData, arrow::error::ArrowError> {
        let ptr = NonNull::new(self.as_ptr() as *mut _).unwrap();
        let len = self.len();

        let buffer = unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, self) };
        arrow::array::ArrayData::try_new(
            arrow::datatypes::DataType::UInt8,
            len,
            None,
            0,
            vec![buffer],
            vec![],
        )
    }
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
