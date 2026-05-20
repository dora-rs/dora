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

use core::ops::Deref;
use std::{
    borrow::{Borrow, BorrowMut},
    num::NonZeroUsize,
};

use zenoh_buffers::{ZBuf, ZSlice};

use super::{
    traits::{BufferRelayoutError, OwnedShmBuf, ShmBuf},
    zshmmut::{zshmmut, ZShmMut},
};
use crate::{
    api::{buffer::traits::ShmBufUnsafeMut, provider::memory_layout::MemoryLayout},
    ShmBufInner,
};

/// An immutable SHM buffer
#[zenoh_macros::unstable_doc]
#[repr(transparent)]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ZShm {
    pub(crate) inner: ShmBufInner,
}

impl ShmBuf<[u8]> for ZShm {
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }
}

impl ShmBufUnsafeMut<[u8]> for ZShm {
    unsafe fn as_mut_unchecked(&mut self) -> &mut [u8] {
        self.inner.as_mut_slice_inner()
    }
}

impl OwnedShmBuf<[u8]> for ZShm {
    fn try_resize(&mut self, new_size: NonZeroUsize) -> Option<()> {
        // Safety: this is safe because ZShm is an owned representation of SHM buffer and thus
        // is guaranteed not to be wrapped into ZSlice (see ShmBufInner::try_resize comment)
        unsafe { self.inner.try_resize(new_size) }
    }

    fn try_relayout(&mut self, new_layout: MemoryLayout) -> Result<(), BufferRelayoutError> {
        // Safety: this is safe because ZShm is an owned representation of SHM buffer and thus
        // is guaranteed not to be wrapped into ZSlice (see ShmBufInner::try_relayout comment)
        unsafe { self.inner.try_relayout(new_layout) }
    }
}

impl PartialEq<&zshm> for ZShm {
    fn eq(&self, other: &&zshm) -> bool {
        self.inner == other.inner
    }
}

impl PartialEq<&zshmmut> for ZShm {
    fn eq(&self, other: &&zshmmut) -> bool {
        self.inner == other.inner
    }
}

impl PartialEq<ZShmMut> for ZShm {
    fn eq(&self, other: &ZShmMut) -> bool {
        self.inner == other.inner
    }
}

impl Borrow<zshm> for ZShm {
    fn borrow(&self) -> &zshm {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute(self) }
    }
}

impl BorrowMut<zshm> for ZShm {
    fn borrow_mut(&mut self) -> &mut zshm {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute(self) }
    }
}

impl Deref for ZShm {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.inner.as_ref()
    }
}

impl AsRef<[u8]> for ZShm {
    fn as_ref(&self) -> &[u8] {
        self
    }
}

impl From<ZShm> for ZSlice {
    fn from(value: ZShm) -> Self {
        value.inner.into()
    }
}

impl From<ZShm> for ZBuf {
    fn from(value: ZShm) -> Self {
        value.inner.into()
    }
}

impl TryFrom<ZShm> for ZShmMut {
    type Error = ZShm;

    fn try_from(value: ZShm) -> Result<Self, Self::Error> {
        match value.inner.is_unique() && value.inner.is_valid() {
            // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
            // to ShmBufInner type, so it is safe to transmute them in any direction
            true => Ok(unsafe { std::mem::transmute::<ZShm, ZShmMut>(value) }),
            false => Err(value),
        }
    }
}

/// A borrowed immutable SHM buffer
#[zenoh_macros::unstable_doc]
#[derive(Debug, PartialEq, Eq)]
#[allow(non_camel_case_types)]
#[repr(transparent)]
pub struct zshm {
    pub(crate) inner: ShmBufInner,
}

impl ShmBuf<[u8]> for &zshm {
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }
}

impl ShmBuf<[u8]> for &mut zshm {
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }
}

impl ShmBufUnsafeMut<[u8]> for &mut zshm {
    unsafe fn as_mut_unchecked(&mut self) -> &mut [u8] {
        self.inner.as_mut_slice_inner()
    }
}

impl Deref for zshm {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.inner.as_ref()
    }
}

impl AsRef<[u8]> for zshm {
    fn as_ref(&self) -> &[u8] {
        self
    }
}

impl PartialEq<ZShm> for &zshm {
    fn eq(&self, other: &ZShm) -> bool {
        self.inner == other.inner
    }
}

impl PartialEq<ZShmMut> for &zshm {
    fn eq(&self, other: &ZShmMut) -> bool {
        self.inner == other.inner
    }
}

impl PartialEq<&zshmmut> for &zshm {
    fn eq(&self, other: &&zshmmut) -> bool {
        self.inner == other.inner
    }
}

impl ToOwned for zshm {
    type Owned = ZShm;

    fn to_owned(&self) -> Self::Owned {
        ZShm::new(self.inner.clone())
    }
}

impl From<&zshmmut> for &zshm {
    fn from(value: &zshmmut) -> Self {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute::<&zshmmut, &zshm>(value) }
    }
}

impl From<&mut zshmmut> for &mut zshm {
    fn from(value: &mut zshmmut) -> Self {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute::<&mut zshmmut, &mut zshm>(value) }
    }
}
