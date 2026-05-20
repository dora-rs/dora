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
    hash::Hash,
    ops::Deref,
    sync::{atomic::AtomicU64, Arc},
};

use super::segment::MetadataSegment;
use crate::header::chunk_header::ChunkHeaderType;

pub type MetadataSegmentID = u16;
pub type MetadataIndex = u16;

#[derive(Clone, Eq, Hash, PartialEq, PartialOrd, Ord, Debug)]
pub struct MetadataDescriptor {
    pub id: MetadataSegmentID,
    pub index: MetadataIndex,
}

impl From<&OwnedMetadataDescriptor> for MetadataDescriptor {
    fn from(item: &OwnedMetadataDescriptor) -> Self {
        let id = item.segment.data.id();
        let index = unsafe { item.segment.data.fast_index_compute(item.header) };

        Self { id, index }
    }
}

#[derive(Clone)]
pub struct OwnedWatchdog {
    watchdog_atomic: &'static AtomicU64,
    watchdog_mask: u64,
}

// The ordering strategy is important. See storage implementation for details
impl Ord for OwnedWatchdog {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match self
            .watchdog_atomic
            .as_ptr()
            .cmp(&other.watchdog_atomic.as_ptr())
        {
            core::cmp::Ordering::Equal => self.watchdog_mask.cmp(&other.watchdog_mask),
            ord => ord,
        }
    }
}

impl PartialOrd for OwnedWatchdog {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for OwnedWatchdog {
    fn eq(&self, other: &Self) -> bool {
        self.watchdog_atomic.as_ptr() == other.watchdog_atomic.as_ptr()
            && self.watchdog_mask == other.watchdog_mask
    }
}
impl Eq for OwnedWatchdog {}

impl OwnedWatchdog {
    pub fn new(watchdog_atomic: &'static AtomicU64, watchdog_mask: u64) -> Self {
        Self {
            watchdog_atomic,
            watchdog_mask,
        }
    }

    pub fn confirm(&self) {
        self.watchdog_atomic
            .fetch_or(self.watchdog_mask, std::sync::atomic::Ordering::SeqCst);
    }

    pub(crate) fn validate(&self) -> u64 {
        self.watchdog_atomic
            .fetch_and(!self.watchdog_mask, std::sync::atomic::Ordering::SeqCst)
            & self.watchdog_mask
    }
}

#[derive(Clone)]
pub struct OwnedMetadataDescriptor {
    pub(crate) segment: Arc<MetadataSegment>,
    header: &'static ChunkHeaderType,
    watchdog: OwnedWatchdog,
}

impl Hash for OwnedMetadataDescriptor {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        (self.header as *const ChunkHeaderType).hash(state);
    }
}

impl Deref for OwnedMetadataDescriptor {
    type Target = OwnedWatchdog;

    fn deref(&self) -> &Self::Target {
        &self.watchdog
    }
}

impl OwnedMetadataDescriptor {
    pub(crate) fn new(
        segment: Arc<MetadataSegment>,
        header: &'static ChunkHeaderType,
        watchdog: OwnedWatchdog,
    ) -> Self {
        Self {
            segment,
            header,
            watchdog,
        }
    }

    #[inline(always)]
    pub fn header(&self) -> &ChunkHeaderType {
        self.header
    }

    #[cfg(feature = "test")]
    pub fn test_validate(&self) -> u64 {
        self.validate()
    }
}

// The ordering strategy is important. See storage implementation for details
impl Ord for OwnedMetadataDescriptor {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.header as *const ChunkHeaderType).cmp(&(other.header as *const _))
    }
}

impl PartialOrd for OwnedMetadataDescriptor {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for OwnedMetadataDescriptor {
    fn eq(&self, other: &Self) -> bool {
        (self.header as *const ChunkHeaderType).eq(&(other.header as *const _))
    }
}
impl Eq for OwnedMetadataDescriptor {}

impl std::fmt::Debug for OwnedMetadataDescriptor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OwnedHeaderDescriptor")
            .field("header", &self.header)
            .finish()
    }
}
