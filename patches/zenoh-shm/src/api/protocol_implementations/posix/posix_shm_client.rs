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

use std::sync::Arc;

use zenoh_result::ZResult;

use super::posix_shm_segment::PosixShmSegment;
use crate::api::{
    client::{shm_client::ShmClient, shm_segment::ShmSegment},
    common::{
        types::{ProtocolID, SegmentID},
        with_id::WithProtocolID,
    },
    protocol_implementations::posix::protocol_id::POSIX_PROTOCOL_ID,
};

/// Client factory implementation for particular shared memory protocol
#[zenoh_macros::unstable_doc]
#[derive(Debug)]
pub struct PosixShmClient;

impl WithProtocolID for PosixShmClient {
    fn id(&self) -> ProtocolID {
        POSIX_PROTOCOL_ID
    }
}

impl ShmClient for PosixShmClient {
    /// Attach to particular shared memory segment
    #[zenoh_macros::unstable_doc]
    fn attach(&self, segment: SegmentID) -> ZResult<Arc<dyn ShmSegment>> {
        Ok(Arc::new(PosixShmSegment::open(segment)?))
    }
}
