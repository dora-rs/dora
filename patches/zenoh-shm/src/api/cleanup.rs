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

use crate::posix_shm::cleanup::cleanup_orphaned_segments;

/// Linux: Trigger cleanup for orphaned SHM segments
///
/// If process that created named SHM segment crashes or exits by a signal, the segment persists in the system
/// disregarding if it is used by other Zenoh processes or not. This is the detail of POSIX specification for
/// shared memory that is hard to bypass. To deal with this we developed a cleanup routine that enumerates all
/// segments and tries to find processes that are using it. If no such process found, segment will be removed.
/// There is no ideal signal to trigger this cleanup, so by default, zenoh triggers it in the following moments:
/// - first POSIX SHM segment creation
/// - process exit via exit() call or return from maint function
/// It is OK to additionally trigger this function at any time, but be aware that this can be costly.
///
/// For non-linux platforms this function currently does nothing
#[zenoh_macros::unstable_doc]
pub fn cleanup_orphaned_shm_segments() {
    cleanup_orphaned_segments();
}
