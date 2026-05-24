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

use std::{
    num::NonZeroUsize,
    sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering},
};

use crate::api::provider::chunk::ChunkDescriptor;

// Chunk header
#[stabby::stabby]
#[derive(Debug)]
pub struct ChunkHeaderType {
    /*
    TODO: We don't really need 32 bits here, but access to 16-bit fields with 1 byte alignment is less performant on most of the platforms.
    We need to bench and select reasonable integer sizes here once we have an implementation to bench
    */
    pub refcount: AtomicU32,
    pub watchdog_invalidated: AtomicBool,
    pub generation: AtomicU32,

    /// Protocol identifier for particular SHM implementation
    pub protocol: AtomicU32,

    /// The data chunk descriptor
    segment: AtomicU32,
    chunk: AtomicU32,
    len: AtomicUsize,
}

impl ChunkHeaderType {
    pub fn len(&self) -> NonZeroUsize {
        // SAFETY: this is safe because Write access to self.len is available only from set_data_descriptor
        unsafe { NonZeroUsize::new_unchecked(self.len.load(Ordering::Relaxed)) }
    }

    pub fn set_data_descriptor(&self, descriptor: &ChunkDescriptor) {
        self.segment.store(descriptor.segment, Ordering::Relaxed);
        self.chunk.store(descriptor.chunk, Ordering::Relaxed);
        self.len.store(descriptor.len.into(), Ordering::Relaxed);
    }

    pub fn data_descriptor(&self) -> ChunkDescriptor {
        ChunkDescriptor::new(
            self.segment.load(Ordering::Relaxed),
            self.chunk.load(Ordering::Relaxed),
            self.len(),
        )
    }
}
