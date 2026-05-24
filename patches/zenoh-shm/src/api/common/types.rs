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

use std::{any::Any, sync::Arc};

/// Unique protocol identifier.
///
/// Here is a contract: it is up to user to make sure that incompatible ShmClient
/// and ShmProviderBackend implementations will never use the same ProtocolID
#[zenoh_macros::unstable_doc]
pub type ProtocolID = u32;

/// Unique segment identifier
#[zenoh_macros::unstable_doc]
pub type SegmentID = u32;

/// Chunk id within it's segment
#[zenoh_macros::unstable_doc]
pub type ChunkID = u32;

/// Pointer that owns it's SHM segment
#[zenoh_macros::unstable_doc]
#[derive(Clone)]
pub struct PtrInSegment {
    ptr: *mut u8,
    _segment: Arc<dyn Any + Send + Sync>,
}

impl PtrInSegment {
    pub fn new(ptr: *mut u8, _segment: Arc<dyn Any + Send + Sync>) -> Self {
        Self { ptr, _segment }
    }

    pub fn ptr(&self) -> *const u8 {
        self.ptr
    }

    /// # SAFETY
    ///
    /// This function is safe if there is guarantee that there is no concurrent access
    pub unsafe fn ptr_mut(&mut self) -> *mut u8 {
        self.ptr
    }
}

impl PartialEq for PtrInSegment {
    fn eq(&self, other: &Self) -> bool {
        // it is enough to compare addresses
        self.ptr == other.ptr
    }
}
impl Eq for PtrInSegment {}

impl std::fmt::Debug for PtrInSegment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PtrInSegment")
            .field("ptr", &self.ptr)
            .finish()
    }
}

unsafe impl Send for PtrInSegment {}
unsafe impl Sync for PtrInSegment {}
