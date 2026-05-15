//
// Copyright (c) 2024 ZettaScale Technology
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

use std::{collections::HashSet, ops::DerefMut, sync::Mutex};

use static_init::dynamic;
use zenoh_core::zlock;

use crate::{posix_shm::cleanup::cleanup_orphaned_segments, shm::Segment};

/// A global cleanup, that is guaranteed to be dropped at normal program exit and that will
/// execute all registered cleanup routines at this moment
#[dynamic(lazy, drop)]
pub(crate) static mut CLEANUP: Cleanup = Cleanup::new();

/// An RAII object that calls all registered routines upon destruction
pub(crate) struct Cleanup {
    cleanups: Mutex<HashSet<u64>>,
}

impl Cleanup {
    fn new() -> Self {
        // on first cleanup subsystem touch we perform zenoh segment cleanup
        cleanup_orphaned_segments();
        Self {
            cleanups: Default::default(),
        }
    }

    pub(crate) fn register_cleanup<ID: crate::shm::SegmentID>(&self, id: ID) {
        let mut lock = zlock!(self.cleanups);
        lock.insert(id.into());
    }

    pub(crate) fn unregister_cleanup<ID: crate::shm::SegmentID>(&self, id: ID) {
        let mut lock = zlock!(self.cleanups);
        lock.remove(&id.into());
    }

    fn cleanup(&self) {
        let ids = {
            let mut lock = zlock!(self.cleanups);
            std::mem::take(lock.deref_mut())
        };

        for id in ids {
            Segment::ensure_not_persistent(id);
        }
    }
}

impl Drop for Cleanup {
    fn drop(&mut self) {
        // on finalization stage we perform zenoh segment cleanup
        cleanup_orphaned_segments();
        self.cleanup();
    }
}
