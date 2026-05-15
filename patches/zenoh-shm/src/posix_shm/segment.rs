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

use std::{fmt::Debug, num::NonZeroUsize};

use rand::Rng;
use zenoh_result::{bail, ZResult};

use crate::{cleanup::CLEANUP, shm};

const SEGMENT_DEDICATE_TRIES: usize = 100;

/// Segment of shared memory identified by an ID
pub struct Segment<ID>
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    ID: shm::SegmentID,
{
    shmem: shm::Segment<ID>,
}

impl<ID> Drop for Segment<ID>
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    ID: shm::SegmentID,
{
    fn drop(&mut self) {
        // this drop might be executed inside static drop callstack, so statics might not be available here
        if let Ok(val) = CLEANUP.try_read() {
            // Unregister unnecessary cleanup routine as Segment will be unlinked here
            val.unregister_cleanup(self.id());
        }
    }
}

impl<ID> Debug for Segment<ID>
where
    ID: Debug,
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    ID: shm::SegmentID,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Segment")
            .field("shmem", &self.shmem.as_ptr())
            .finish()
    }
}

impl<ID> Segment<ID>
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    ID: shm::SegmentID,
{
    // Automatically generate free id and create a new segment identified by this id
    pub fn create(len: NonZeroUsize) -> ZResult<Self> {
        for _ in 0..SEGMENT_DEDICATE_TRIES {
            // Generate random id
            let id: ID = rand::thread_rng().gen();

            // Try to create a new segment identified by prefix and generated id.
            // If creation fails because segment already exists for this id,
            // the creation attempt will be repeated with another id
            match shm::Segment::create(id, len) {
                Ok(shmem) => {
                    // Register cleanup routine to make sure Segment will be unlinked on exit
                    CLEANUP.read().register_cleanup(id);

                    tracing::debug!("Created SHM segment, len: {len}, id: {id}");
                    return Ok(Segment { shmem });
                }
                Err(shm::SegmentCreateError::SegmentExists) => {}
                Err(shm::SegmentCreateError::OsError(e)) => {
                    bail!("Unable to create POSIX shm segment: OS error {}", e)
                }
            }
        }
        bail!("Unable to dedicate POSIX shm segment file after {SEGMENT_DEDICATE_TRIES} tries!");
    }

    // Open an existing segment identified by id
    pub fn open(id: ID) -> ZResult<Self> {
        // Open SHM segment
        let shmem = match shm::Segment::open(id) {
            Ok(val) => val,
            Err(shm::SegmentOpenError::InvalidatedSegment) => {
                bail!("Unable to open POSIX shm segment: segment is invalid!");
            }
            Err(shm::SegmentOpenError::OsError(e)) => {
                bail!("Unable to open POSIX shm segment: OS error {}", e);
            }
        };

        // Register cleanup routine to make sure Segment will be unlinked on exit
        CLEANUP.read().register_cleanup(id);

        tracing::debug!("Opened SHM segment, id: {id}");

        Ok(Self { shmem })
    }

    pub fn as_ptr(&self) -> *mut u8 {
        self.shmem.as_ptr()
    }

    /// Returns the length of this [`Segment<ID>`].
    /// NOTE: one some platforms (at least windows) the returned len will be the actual length of an shm segment
    /// (a required len rounded up to the nearest multiply of page size), on other (at least linux and macos) this
    /// returns a value requested upon segment creation
    pub fn len(&self) -> NonZeroUsize {
        self.shmem.len()
    }

    pub fn id(&self) -> ID {
        self.shmem.id()
    }
}
