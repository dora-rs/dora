use std::marker::PhantomData;

use dora_core::{
    config::{DataId, OperatorId},
    message::Metadata,
};
use eyre::Context;
use shared_memory::{Shmem, ShmemConf};

#[derive(Debug)]
#[non_exhaustive]
pub enum Event<'a> {
    Stop,
    Reload {
        operator_id: OperatorId,
    },
    Input {
        id: DataId,
        metadata: Metadata<'static>,
        data: Option<Data<'a>>,
    },
    InputClosed {
        id: DataId,
    },
    Error(String),
}

pub enum Data<'a> {
    Vec(Vec<u8>),
    SharedMemory {
        data: MappedInputData<'a>,
        _drop: std::sync::mpsc::Sender<()>,
    },
}

impl std::ops::Deref for Data<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        match self {
            Data::SharedMemory { data, .. } => data,
            Data::Vec(data) => data,
        }
    }
}

impl std::fmt::Debug for Data<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
}

pub struct MappedInputData<'a> {
    memory: Shmem,
    len: usize,
    _data: PhantomData<&'a [u8]>,
}

impl MappedInputData<'_> {
    pub(crate) unsafe fn map(shared_memory_id: &str, len: usize) -> eyre::Result<Self> {
        let memory = ShmemConf::new()
            .os_id(shared_memory_id)
            .open()
            .wrap_err("failed to map shared memory input")?;
        Ok(MappedInputData {
            memory,
            len,
            _data: PhantomData,
        })
    }
}

impl std::ops::Deref for MappedInputData<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { &self.memory.as_slice()[..self.len] }
    }
}
