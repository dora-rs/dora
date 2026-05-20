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

use std::ops::Deref;

use super::{descriptor::OwnedMetadataDescriptor, storage::GLOBAL_METADATA_STORAGE};
use crate::watchdog::validator::GLOBAL_VALIDATOR;

#[derive(Debug)]
pub struct AllocatedMetadataDescriptor {
    descriptor: OwnedMetadataDescriptor,
}

impl AllocatedMetadataDescriptor {
    pub fn new(descriptor: OwnedMetadataDescriptor) -> Self {
        //initialize header fields
        let header = descriptor.header();
        header
            .refcount
            .store(1, std::sync::atomic::Ordering::SeqCst);
        header
            .watchdog_invalidated
            .store(false, std::sync::atomic::Ordering::SeqCst);

        // reset watchdog on allocation
        descriptor.validate();

        Self { descriptor }
    }
}

impl Drop for AllocatedMetadataDescriptor {
    fn drop(&mut self) {
        GLOBAL_VALIDATOR.read().remove(self.descriptor.clone());
        GLOBAL_METADATA_STORAGE
            .read()
            .reclaim(self.descriptor.clone());
    }
}

impl Deref for AllocatedMetadataDescriptor {
    type Target = OwnedMetadataDescriptor;

    fn deref(&self) -> &Self::Target {
        &self.descriptor
    }
}
