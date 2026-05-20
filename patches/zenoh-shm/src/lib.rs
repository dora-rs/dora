//
// Copyright (c) 2023 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

//! ⚠️ WARNING ⚠️
//!
//! This crate is intended for Zenoh's internal use.
//!
//! [Click here for Zenoh's documentation](https://docs.rs/zenoh/latest/zenoh)
use std::{any::Any, num::NonZeroUsize, sync::atomic::Ordering};

use api::{
    buffer::{
        traits::BufferRelayoutError,
        zshm::{zshm, ZShm},
        zshmmut::{zshmmut, ZShmMut},
    },
    common::types::ProtocolID,
    provider::memory_layout::MemoryLayout,
};
use metadata::descriptor::MetadataDescriptor;
use watchdog::confirmator::ConfirmedDescriptor;
use zenoh_buffers::ZSliceBuffer;

use crate::api::common::types::PtrInSegment;

#[macro_export]
macro_rules! tested_module {
    ($module:ident) => {
        #[cfg(feature = "test")]
        pub mod $module;
        #[cfg(not(feature = "test"))]
        mod $module;
    };
}

#[macro_export]
macro_rules! tested_crate_module {
    ($module:ident) => {
        #[cfg(feature = "test")]
        pub mod $module;
        #[cfg(not(feature = "test"))]
        pub(crate) mod $module;
    };
}

pub mod api;
mod cleanup;
pub mod header;
pub mod init;
pub mod metadata;
pub mod posix_shm;
pub mod reader;
pub mod version;
pub mod watchdog;
tested_crate_module!(shm);

/// Information about a [`ShmBufInner`].
///
/// This that can be serialized and can be used to retrieve the [`ShmBufInner`] in a remote process.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ShmBufInfo {
    /// Actual data length
    /// NOTE: data descriptor's len is >= of this len and describes the actual memory length
    /// dedicated in shared memory segment for this particular buffer.
    pub data_len: NonZeroUsize,

    /// Metadata descriptor
    pub metadata: MetadataDescriptor,
    /// Generation of the buffer
    pub generation: u32,
}

impl ShmBufInfo {
    pub fn new(
        data_len: NonZeroUsize,
        metadata: MetadataDescriptor,
        generation: u32,
    ) -> ShmBufInfo {
        ShmBufInfo {
            data_len,
            metadata,
            generation,
        }
    }
}

/// A zenoh buffer in shared memory.
pub struct ShmBufInner {
    pub(crate) metadata: ConfirmedDescriptor,
    pub(crate) buf: PtrInSegment,
    pub info: ShmBufInfo,
}

impl PartialEq for ShmBufInner {
    fn eq(&self, other: &Self) -> bool {
        // currently there is no API to resize an SHM buffer, but it is intended in the future,
        // so I add size comparison here to avoid future bugs :)
        self.buf == other.buf && self.info.data_len == other.info.data_len
    }
}
impl Eq for ShmBufInner {}

impl std::fmt::Debug for ShmBufInner {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ShmBufInner")
            .field("metadata", &self.metadata)
            .field("buf", &self.buf)
            .field("info", &self.info)
            .finish()
    }
}

impl ShmBufInner {
    pub fn protocol(&self) -> ProtocolID {
        self.metadata
            .owned
            .header()
            .protocol
            .load(Ordering::Relaxed)
    }

    /// # Safety
    /// This operation is unsafe because we cannot guarantee that there is no upper level
    /// ZSlice that have cached the size of underlying ShmBufInner. So this function should be
    /// used only if it is guaranteed that there is no such situation
    pub unsafe fn try_resize(&mut self, new_size: NonZeroUsize) -> Option<()> {
        if self.capacity() < new_size {
            return None;
        }

        self.info.data_len = new_size;

        Some(())
    }

    /// # Safety
    /// This operation is unsafe because we cannot guarantee that there is no upper level
    /// ZSlice that have cached the size of underlying ShmBufInner. So this function should be
    /// used only if it is guaranteed that there is no such situation
    pub unsafe fn try_relayout(
        &mut self,
        new_layout: MemoryLayout,
    ) -> Result<(), BufferRelayoutError> {
        let address = self.as_ref().as_ptr() as usize;
        if address % new_layout.alignment().get_alignment_value().get() != 0 {
            return Err(BufferRelayoutError::IncompatibleAlignment);
        }

        if self.capacity() < new_layout.size() {
            return Err(BufferRelayoutError::SizeTooBig);
        }

        self.info.data_len = new_layout.size();

        Ok(())
    }

    pub fn len(&self) -> NonZeroUsize {
        self.info.data_len
    }

    pub fn capacity(&self) -> NonZeroUsize {
        self.metadata.owned.header().len()
    }

    fn is_valid(&self) -> bool {
        let header = self.metadata.owned.header();

        !header.watchdog_invalidated.load(Ordering::SeqCst)
            && header.generation.load(Ordering::SeqCst) == self.info.generation
    }

    fn is_unique(&self) -> bool {
        self.ref_count() == 1
    }

    pub fn ref_count(&self) -> u32 {
        self.metadata.owned.header().refcount.load(Ordering::SeqCst)
    }

    /// Increments buffer's reference count
    ///
    /// # Safety
    /// You should understand what you are doing, as overestimation
    /// of the reference counter can lead to memory being stalled until
    /// recovered by watchdog subsystem or forcibly deallocated
    pub unsafe fn inc_ref_count(&self) {
        self.metadata
            .owned
            .header()
            .refcount
            .fetch_add(1, Ordering::SeqCst);
    }

    // PRIVATE:
    fn as_slice(&self) -> &[u8] {
        tracing::trace!("ShmBufInner::as_slice() == len = {:?}", self.info.data_len);
        unsafe { std::slice::from_raw_parts(self.buf.ptr(), self.info.data_len.get()) }
    }

    unsafe fn dec_ref_count(&self) {
        self.metadata
            .owned
            .header()
            .refcount
            .fetch_sub(1, Ordering::SeqCst);
    }

    /// Gets a mutable slice.
    ///
    /// # Safety
    /// This operation is marked unsafe since we cannot guarantee the single mutable reference
    /// across multiple processes. Thus if you use it, and you'll inevitable have to use it,
    /// you have to keep in mind that if you have multiple process retrieving a mutable slice
    /// you may get into concurrent writes. That said, if you have a serial pipeline and
    /// the buffer is flowing through the pipeline this will not create any issues.
    ///
    /// In short, whilst this operation is marked as unsafe, you are safe if you can
    /// guarantee that your in applications only one process at the time will actually write.
    unsafe fn as_mut_slice_inner(&mut self) -> &mut [u8] {
        std::slice::from_raw_parts_mut(self.buf.ptr_mut(), self.info.data_len.get())
    }
}

impl Drop for ShmBufInner {
    fn drop(&mut self) {
        // SAFETY: obviously, we need to decrement refcount when dropping ShmBufInner instance
        unsafe { self.dec_ref_count() };
    }
}

impl Clone for ShmBufInner {
    fn clone(&self) -> Self {
        // SAFETY: obviously, we need to increment refcount when cloning ShmBufInner instance
        unsafe { self.inc_ref_count() };
        ShmBufInner {
            metadata: self.metadata.clone(),
            buf: self.buf.clone(),
            info: self.info.clone(),
        }
    }
}

// Buffer impls
// - ShmBufInner
impl AsRef<[u8]> for ShmBufInner {
    fn as_ref(&self) -> &[u8] {
        self.as_slice()
    }
}

impl AsMut<[u8]> for ShmBufInner {
    fn as_mut(&mut self) -> &mut [u8] {
        unsafe { self.as_mut_slice_inner() }
    }
}

impl ZSliceBuffer for ShmBufInner {
    fn as_slice(&self) -> &[u8] {
        self.as_ref()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

impl ZShm {
    pub(crate) fn new(inner: ShmBufInner) -> Self {
        Self { inner }
    }
}

impl From<ShmBufInner> for ZShm {
    fn from(value: ShmBufInner) -> Self {
        Self::new(value)
    }
}

impl ZShmMut {
    pub(crate) unsafe fn new_unchecked(inner: ShmBufInner) -> Self {
        Self { inner }
    }
}

impl TryFrom<ShmBufInner> for ZShmMut {
    type Error = ShmBufInner;

    fn try_from(value: ShmBufInner) -> Result<Self, Self::Error> {
        match value.is_unique() && value.is_valid() {
            // SAFETY: we checked above
            true => Ok(unsafe { Self::new_unchecked(value) }),
            false => Err(value),
        }
    }
}

impl TryFrom<&mut ShmBufInner> for &mut zshmmut {
    type Error = ();

    fn try_from(value: &mut ShmBufInner) -> Result<Self, Self::Error> {
        match value.is_unique() && value.is_valid() {
            // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
            // to ShmBufInner type, so it is safe to transmute them in any direction
            true => Ok(unsafe { core::mem::transmute::<&mut ShmBufInner, &mut zshmmut>(value) }),
            false => Err(()),
        }
    }
}

impl From<&ShmBufInner> for &zshm {
    fn from(value: &ShmBufInner) -> Self {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute::<&ShmBufInner, &zshm>(value) }
    }
}

impl From<&mut ShmBufInner> for &mut zshm {
    fn from(value: &mut ShmBufInner) -> Self {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute::<&mut ShmBufInner, &mut zshm>(value) }
    }
}
