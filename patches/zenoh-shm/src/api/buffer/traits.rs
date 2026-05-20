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
use std::num::NonZeroUsize;

use stabby::{abi::typenum2::B0, IStable};

use crate::api::provider::memory_layout::MemoryLayout;

/// # Safety
/// This trait is unsafe because it allows types to be used in shared memory.
/// It is the responsibility of the implementer to ensure that the type is safe to use in shared memory.
/// There are some safe blanket implementations below
pub unsafe trait ResideInShm: Send {}

unsafe impl<T: Send + IStable<ContainsIndirections = B0>> ResideInShm for T {}

/// Errors for buffer relayouting operation.
#[zenoh_macros::unstable_doc]
#[derive(Debug)]
pub enum BufferRelayoutError {
    IncompatibleAlignment,
    SizeTooBig,
}

#[zenoh_macros::unstable_doc]
pub trait ShmBuf<T: ?Sized>: Sized + AsRef<T> {
    #[zenoh_macros::unstable_doc]
    fn is_valid(&self) -> bool;
}

#[zenoh_macros::unstable_doc]
pub trait ShmBufUnsafeMut<T: ?Sized>: ShmBuf<T> {
    #[zenoh_macros::unstable_doc]
    /// Get unchecked mutable access to buffer's memory.
    ///
    /// This is unsafe yet very powerful API for building concurrent access logic around SHM buffer contents.
    /// For safe version please see `ShmBufMut` trait.
    ///
    /// # Safety
    ///
    /// Safe if multiple conditions are met:
    /// - user code guarantees no data race across all applications that share the buffer
    /// - the buffer is not being concurrently sent to the outside of SHM domain
    /// - the buffer is valid
    unsafe fn as_mut_unchecked(&mut self) -> &mut T;
}

#[zenoh_macros::unstable_doc]
pub trait ShmBufMut<T: ?Sized>: ShmBuf<T> + AsMut<T> {}

#[zenoh_macros::unstable_doc]
pub trait OwnedShmBuf<T: ?Sized>: ShmBuf<T> {
    #[zenoh_macros::unstable_doc]
    fn try_resize(&mut self, new_size: NonZeroUsize) -> Option<()>;

    #[zenoh_macros::unstable_doc]
    fn try_relayout(&mut self, new_layout: MemoryLayout) -> Result<(), BufferRelayoutError>;
}

#[zenoh_macros::unstable_doc]
pub trait ShmBufIntoImmut<T: ?Sized>: ShmBuf<T> {
    type ImmutBuf: ShmBuf<T>;

    #[zenoh_macros::unstable_doc]
    fn into_immut(self) -> Self::ImmutBuf;
}
