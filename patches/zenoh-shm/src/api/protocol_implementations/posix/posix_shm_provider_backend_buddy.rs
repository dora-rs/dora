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
    sync::{Arc, Mutex},
};

use buddy_system_allocator::Heap;
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
pub struct PosixShmProviderBackendBuddyBuilder<Layout> {
    layout: Layout,
}

#[zenoh_macros::unstable_doc]
impl<Layout> Resolvable for PosixShmProviderBackendBuddyBuilder<Layout> {
    type To = ZResult<PosixShmProviderBackendBuddy>;
}

#[zenoh_macros::unstable_doc]
impl<Layout: TryInto<MemoryLayout>> Wait for PosixShmProviderBackendBuddyBuilder<Layout>
where
    Layout::Error: Into<ZLayoutError>,
{
    fn wait(self) -> <Self as Resolvable>::To {
        PosixShmProviderBackendBuddy::new(&self.layout.try_into().map_err(Into::into)?)
    }
}

/// A buddy_system_allocator backend based on POSIX shared memory.
///
/// buddy_system_allocator is the fastest allocator ever. The weak side is it's low memry efficiency and
/// higher fragmentation as opposed to talc (which is treated as the universal default for zenoh SHM)
#[zenoh_macros::unstable_doc]
pub struct PosixShmProviderBackendBuddy {
    segment: Arc<PosixShmSegment>,
    heap: Mutex<Heap<BUDDY_ORDER>>,
    alignment: AllocAlignment,
}

// see `buddy_system_allocator` doc for details
const BUDDY_ORDER: usize = 30;

impl PosixShmProviderBackendBuddy {
    /// Get the builder to construct a new instance
    #[zenoh_macros::unstable_doc]
    pub fn builder<Layout>(layout: Layout) -> PosixShmProviderBackendBuddyBuilder<Layout> {
        PosixShmProviderBackendBuddyBuilder { layout }
    }

    fn new(layout: &MemoryLayout) -> ZResult<Self> {
        let segment = Arc::new(PosixShmSegment::create(layout.size())?);

        // because of platform specific, our shm segment is >= requested size, so in order to utilize
        // additional memory we re-layout the size
        let real_size = segment.segment.elem_count().get();

        let mut heap = Heap::empty();

        unsafe { heap.init(segment.segment.elem_mut(0) as usize, real_size) };

        tracing::trace!(
            "Created PosixShmProviderBackendBuddy id {}, layout {:?}",
            segment.segment.id(),
            layout
        );

        Ok(Self {
            segment,
            heap: Mutex::new(heap),
            alignment: layout.alignment(),
        })
    }
}

impl WithProtocolID for PosixShmProviderBackendBuddy {
    fn id(&self) -> ProtocolID {
        POSIX_PROTOCOL_ID
    }
}

impl ShmProviderBackend for PosixShmProviderBackendBuddy {
    fn alloc(&self, layout: &MemoryLayout) -> ChunkAllocResult {
        tracing::trace!("PosixShmProviderBackendBuddy::alloc({:?})", layout);

        let alloc_layout = unsafe {
            Layout::from_size_align_unchecked(
                layout.size().get(),
                layout.alignment().get_alignment_value().get(),
            )
        };

        let alloc = {
            let mut lock = zlock!(self.heap);
            lock.alloc(alloc_layout)
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

        unsafe { zlock!(self.heap).dealloc(NonNull::new_unchecked(ptr), alloc_layout) };
    }

    fn defragment(&self) -> usize {
        0
    }

    fn available(&self) -> usize {
        let lock = zlock!(self.heap);
        lock.stats_total_bytes() - lock.stats_alloc_actual()
    }

    fn layout_for(&self, layout: MemoryLayout) -> Result<MemoryLayout, ZLayoutError> {
        layout.extend(self.alignment)
    }
}
