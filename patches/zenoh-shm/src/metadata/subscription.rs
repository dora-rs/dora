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
    collections::BTreeMap,
    sync::{Arc, RwLock},
};

use static_init::dynamic;
use zenoh_core::{zread, zwrite};
use zenoh_result::ZResult;

use super::{
    descriptor::{MetadataDescriptor, MetadataSegmentID, OwnedMetadataDescriptor},
    segment::MetadataSegment,
};

#[dynamic(lazy, drop)]
pub static mut GLOBAL_METADATA_SUBSCRIPTION: Subscription = Subscription::new();

pub struct Subscription {
    linked_table: RwLock<BTreeMap<MetadataSegmentID, Arc<MetadataSegment>>>,
}

impl Subscription {
    fn new() -> Self {
        Self {
            linked_table: RwLock::default(),
        }
    }

    fn ensure_segment(&self, id: MetadataSegmentID) -> ZResult<Arc<MetadataSegment>> {
        // fastest path: try to utilize already existing segment
        {
            let guard = zread!(self.linked_table);
            if let Some(segment) = guard.get(&id) {
                return Ok(segment.clone());
            }
        }

        // try to create segment
        let mut guard = zwrite!(self.linked_table);
        // ensure segment
        Ok(match guard.entry(id) {
            std::collections::btree_map::Entry::Vacant(vacant) => {
                let segment = Arc::new(MetadataSegment::open(id)?);
                vacant.insert(segment.clone());
                segment
            }
            std::collections::btree_map::Entry::Occupied(occupied) => {
                // this is intentional
                occupied.get().clone()
            }
        })
    }

    pub fn link(&self, descriptor: &MetadataDescriptor) -> ZResult<OwnedMetadataDescriptor> {
        let segment = self.ensure_segment(descriptor.id)?;

        // construct owned descriptor
        // SAFETY: MetadataDescriptor source guarantees that descriptor.index is valid for segment
        let (header, watchdog) = unsafe { segment.data.fast_elem_compute(descriptor.index) };

        let owned_descriptor = OwnedMetadataDescriptor::new(segment, header, watchdog);
        Ok(owned_descriptor)
    }
}
