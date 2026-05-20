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

use std::{fmt::Debug, sync::Arc};

use zenoh_result::ZResult;

use super::shm_segment::ShmSegment;
use crate::api::common::{types::SegmentID, with_id::WithProtocolID};

/// ShmClient - client factory implementation for particular shared memory protocol
#[zenoh_macros::unstable_doc]
pub trait ShmClient: Debug + Send + Sync + WithProtocolID {
    /// Attach to particular shared memory segment
    #[zenoh_macros::unstable_doc]
    fn attach(&self, segment: SegmentID) -> ZResult<Arc<dyn ShmSegment>>;
}
