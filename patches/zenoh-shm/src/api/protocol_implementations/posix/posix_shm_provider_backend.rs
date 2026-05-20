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

use crate::api::protocol_implementations::posix::posix_shm_provider_backend_talc::{
    PosixShmProviderBackendTalc, PosixShmProviderBackendTalcBuilder,
};

pub type PosixShmProviderBackendBuilder<Layout> = PosixShmProviderBackendTalcBuilder<Layout>;

pub type PosixShmProviderBackend = PosixShmProviderBackendTalc;
