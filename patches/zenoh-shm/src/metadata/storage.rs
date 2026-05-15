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
    collections::VecDeque,
    sync::{Arc, Mutex},
};

use static_init::dynamic;
use zenoh_core::zlock;
use zenoh_result::ZResult;

use super::{
    allocated_descriptor::AllocatedMetadataDescriptor,
    descriptor::{MetadataIndex, OwnedMetadataDescriptor},
    segment::MetadataSegment,
};
use crate::api::provider::types::ZAllocError;

#[dynamic(lazy, drop)]
pub static mut GLOBAL_METADATA_STORAGE: MetadataStorage = MetadataStorage::new().unwrap();

pub struct MetadataStorage {
    available: Mutex<VecDeque<OwnedMetadataDescriptor>>,
}

impl MetadataStorage {
    fn new() -> ZResult<Self> {
        // See ordering implementation for OwnedMetadataDescriptor
        #[allow(clippy::mutable_key_type)]
        let mut initially_available = VecDeque::<OwnedMetadataDescriptor>::default();

        Self::add_segment(&mut initially_available)?;

        Ok(Self {
            available: Mutex::new(initially_available),
        })
    }

    // See ordering implementation for OwnedMetadataDescriptor
    #[allow(clippy::mutable_key_type)]
    fn add_segment(collection: &mut VecDeque<OwnedMetadataDescriptor>) -> ZResult<()> {
        let segment = Arc::new(MetadataSegment::create()?);

        for index in 0..segment.data.count() {
            let (header, watchdog) =
                unsafe { segment.data.fast_elem_compute(index as MetadataIndex) };
            let descriptor = OwnedMetadataDescriptor::new(segment.clone(), header, watchdog);

            // init generation (this is not really necessary, but we do)
            descriptor
                .header()
                .generation
                .store(0, std::sync::atomic::Ordering::SeqCst);

            collection.push_back(descriptor);
        }
        Ok(())
    }

    pub fn allocate(&self) -> Result<AllocatedMetadataDescriptor, ZAllocError> {
        let mut guard = zlock!(self.available);
        let descriptor = match guard.pop_front() {
            Some(val) => val,
            None => {
                Self::add_segment(&mut guard)?;
                guard.pop_front().ok_or(ZAllocError::Other)?
            }
        };
        drop(guard);

        Ok(AllocatedMetadataDescriptor::new(descriptor))
    }

    pub fn reclaim(&self, descriptor: OwnedMetadataDescriptor) {
        // header deallocated - increment it's generation to invalidate any existing references
        descriptor
            .header()
            .generation
            .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        let mut guard = self.available.lock().unwrap();
        guard.push_back(descriptor);
    }
}
