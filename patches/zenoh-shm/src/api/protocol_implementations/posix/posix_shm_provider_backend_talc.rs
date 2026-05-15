//
// Copyright (c) 2025 ZettaScale Technology
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

use std::{
    alloc::Layout,
    ptr::NonNull,
    slice,
    sync::{Arc, Mutex},
};

use talc::{ErrOnOom, Talc};
use zenoh_core::{zlock, Resolvable, Wait};
use zenoh_result::ZResult;

use super::posix_shm_segment::PosixShmSegment;
use crate::api::{
    common::{types::ProtocolID, with_id::WithProtocolID},
    protocol_implementations::posix::protocol_id::POSIX_PROTOCOL_ID,
    provider::{
        chunk::ChunkDescriptor,
        memory_layout::MemoryLayout,
        shm_provider_backend::ShmProviderBackend,
        types::{AllocAlignment, ChunkAllocResult, ZAllocError, ZLayoutError},
    },
};

/// Builder to create posix SHM provider
#[zenoh_macros::unstable_doc]
pub struct PosixShmProviderBackendTalcBuilder<Layout> {
    layout: Layout,
}

#[zenoh_macros::unstable_doc]
impl<Layout> Resolvable for PosixShmProviderBackendTalcBuilder<Layout> {
    type To = ZResult<PosixShmProviderBackendTalc>;
}

#[zenoh_macros::unstable_doc]
impl<Layout: TryInto<MemoryLayout>> Wait for PosixShmProviderBackendTalcBuilder<Layout>
where
    Layout::Error: Into<ZLayoutError>,
{
    fn wait(self) -> <Self as Resolvable>::To {
        PosixShmProviderBackendTalc::new(&self.layout.try_into().map_err(Into::into)?)
    }
}

/// A talc backend based on POSIX shared memory.
///
/// This is the default general-purpose backend shipped with Zenoh.
/// Talc allocator provides great performnce (2nd after `buddy_system_allocator`) while maintaining
/// excellent fragmentation resistance and memory utilization efficiency.
#[zenoh_macros::unstable_doc]
pub struct PosixShmProviderBackendTalc {
    segment: Arc<PosixShmSegment>,
    talc: Mutex<Talc<ErrOnOom>>,
    alignment: AllocAlignment,
}

impl PosixShmProviderBackendTalc {
    /// Get the builder to construct a new instance
    #[zenoh_macros::unstable_doc]
    pub fn builder<Layout>(layout: Layout) -> PosixShmProviderBackendTalcBuilder<Layout> {
        PosixShmProviderBackendTalcBuilder { layout }
    }

    fn new(layout: &MemoryLayout) -> ZResult<Self> {
        let segment = Arc::new(PosixShmSegment::create(layout.size())?);

        // because of platform specific, our shm segment is >= requested size, so in order to utilize
        // additional memory we re-layout the size
        let real_size = segment.segment.elem_count().get();
        let ptr = unsafe { segment.segment.elem_mut(0) };

        // Fix: align the Talc heap base to 64 bytes so that Talc internal
        // relative-alignment arithmetic produces absolutely-aligned pointers.
        //
        // Talc computes aligned sub-allocations relative to the heap base it
        // was given. On Jetson NX / L4T the Zenoh Posix SHM mmap path returns
        // a base that is only 16-byte aligned. A sub-allocation that is
        // "64-byte aligned within the heap" therefore has an absolute virtual
        // address that is only 16-byte aligned, causing aarch64 NEON SIGBUS on
        // Arrow buffers. Advancing the base to the next 64-byte boundary costs
        // at most 63 bytes of the pool and guarantees that every
        // AllocAlignment::new(6) request returns a truly 64-byte aligned ptr.
        const ARROW_ALIGN: usize = 64;
        let base_addr = ptr as usize;
        let padding = base_addr.wrapping_neg() & (ARROW_ALIGN - 1);
        let (aligned_ptr, usable_size) = if padding < real_size {
            (unsafe { ptr.add(padding) }, real_size - padding)
        } else {
            (ptr, real_size)
        };

        let mut talc = Talc::new(ErrOnOom);

        unsafe {
            talc.claim(slice::from_raw_parts_mut(aligned_ptr, usable_size).into())
                .map_err(|_| "Error initializing Talc backend!")?;
        }

        tracing::trace!(
            "Created PosixShmProviderBackendTalc id {}, layout {:?}",
            segment.segment.id(),
            layout
        );

        Ok(Self {
            segment,
            talc: Mutex::new(talc),
            alignment: layout.alignment(),
        })
    }
}

impl WithProtocolID for PosixShmProviderBackendTalc {
    fn id(&self) -> ProtocolID {
        POSIX_PROTOCOL_ID
    }
}

impl ShmProviderBackend for PosixShmProviderBackendTalc {
    fn alloc(&self, layout: &MemoryLayout) -> ChunkAllocResult {
        tracing::trace!("PosixShmProviderBackendTalc::alloc({:?})", layout);

        let alloc_layout = unsafe {
            Layout::from_size_align_unchecked(
                layout.size().get(),
                layout.alignment().get_alignment_value().get(),
            )
        };

        let alloc = {
            let mut lock = zlock!(self.talc);
            unsafe { lock.malloc(alloc_layout) }
        };

        match alloc {
            Ok(buf) => Ok(self.segment.clone().allocated_chunk(buf, layout)),
            Err(_) => Err(ZAllocError::OutOfMemory),
        }
    }

    fn free(&self, chunk: &ChunkDescriptor) {
        let alloc_layout = unsafe {
            Layout::from_size_align_unchecked(
                chunk.len.get(),
                self.alignment.get_alignment_value().get(),
            )
        };

        let ptr = unsafe { self.segment.segment.elem_mut(chunk.chunk) };

        unsafe { zlock!(self.talc).free(NonNull::new_unchecked(ptr), alloc_layout) };
    }

    fn defragment(&self) -> usize {
        0
    }

    fn available(&self) -> usize {
        0
    }

    fn layout_for(&self, layout: MemoryLayout) -> Result<MemoryLayout, ZLayoutError> {
        layout.extend(self.alignment)
    }
}
