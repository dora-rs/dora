use std::{panic::AssertUnwindSafe, ptr::NonNull, sync::Arc};

use aligned_vec::{AVec, ConstAlign};
use dora_arrow_convert::IntoArrow;
use dora_message::metadata::ArrowTypeInfo;
use zenoh::shm::ZShm;

use crate::arrow_utils::buffer_into_arrow_array;

/// Wrapper around ZShm that implements RefUnwindSafe, which is required by
/// Arrow's `Allocation` trait for `Buffer::from_custom_allocation`.
///
/// ZShm is `!RefUnwindSafe` because it contains `Arc<dyn Any + Send + Sync>`,
/// but for our use case (keeping SHM alive while Arrow references it) this is
/// safe — we only need the Drop impl to release the SHM reference.
struct ShmAllocation(#[allow(dead_code)] AssertUnwindSafe<ZShm>);

pub enum RawData {
    Empty,
    Vec(AVec<u8, ConstAlign<128>>),
    /// Zero-copy SHM buffer received from zenoh. The ZShm keeps the shared
    /// memory region alive while Arrow references the underlying bytes.
    ZenohShm(ZShm),
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
            RawData::ZenohShm(shm) => {
                let ptr = NonNull::new(shm.as_ptr() as *mut _).unwrap();
                let len = shm.len();
                let alloc = ShmAllocation(AssertUnwindSafe(shm));

                // Safety: ZShm derefs to &[u8] with a stable pointer and length.
                // The Arc<ShmAllocation> prevents deallocation while Arrow holds the buffer.
                unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(alloc)) }
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
