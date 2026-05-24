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

use core::ops::{Deref, DerefMut};
use std::{
    borrow::{Borrow, BorrowMut},
    num::NonZeroUsize,
};

use zenoh_buffers::{ZBuf, ZSlice};

use super::{
    traits::{BufferRelayoutError, OwnedShmBuf, ShmBuf, ShmBufMut},
    zshm::{zshm, ZShm},
};
use crate::{
    api::{
        buffer::traits::{ShmBufIntoImmut, ShmBufUnsafeMut},
        provider::memory_layout::MemoryLayout,
    },
    ShmBufInner,
};

/// A mutable SHM buffer
#[zenoh_macros::unstable_doc]
#[derive(Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct ZShmMut {
    pub(crate) inner: ShmBufInner,
}

impl ShmBuf<[u8]> for ZShmMut {
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }
}

impl ShmBufUnsafeMut<[u8]> for ZShmMut {
    unsafe fn as_mut_unchecked(&mut self) -> &mut [u8] {
        self.inner.as_mut_slice_inner()
    }
}

impl ShmBufIntoImmut<[u8]> for ZShmMut {
    type ImmutBuf = ZShm;

    fn into_immut(self) -> Self::ImmutBuf {
        self.into()
    }
}

impl ShmBufMut<[u8]> for ZShmMut {}

impl OwnedShmBuf<[u8]> for ZShmMut {
    fn try_resize(&mut self, new_size: NonZeroUsize) -> Option<()> {
        // Safety: this is safe because ZShmMut is an owned representation of SHM buffer and thus
        // is guaranteed not to be wrapped into ZSlice (see ShmBufInner::try_resize comment)
        unsafe { self.inner.try_resize(new_size) }
    }

    fn try_relayout(&mut self, new_layout: MemoryLayout) -> Result<(), BufferRelayoutError> {
        // Safety: this is safe because ZShmMut is an owned representation of SHM buffer and thus
        // is guaranteed not to be wrapped into ZSlice (see ShmBufInner::try_resize comment)
        unsafe { self.inner.try_relayout(new_layout) }
    }
}

impl PartialEq<zshmmut> for &ZShmMut {
    fn eq(&self, other: &zshmmut) -> bool {
        self.inner == other.inner
    }
}

impl Borrow<zshm> for ZShmMut {
    fn borrow(&self) -> &zshm {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute(self) }
    }
}

impl BorrowMut<zshm> for ZShmMut {
    fn borrow_mut(&mut self) -> &mut zshm {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute(self) }
    }
}

impl Borrow<zshmmut> for ZShmMut {
    fn borrow(&self) -> &zshmmut {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute(self) }
    }
}

impl BorrowMut<zshmmut> for ZShmMut {
    fn borrow_mut(&mut self) -> &mut zshmmut {
        // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
        // to ShmBufInner type, so it is safe to transmute them in any direction
        unsafe { core::mem::transmute(self) }
    }
}

impl Deref for ZShmMut {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.inner.as_ref()
    }
}

impl DerefMut for ZShmMut {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.as_mut()
    }
}

impl AsRef<[u8]> for ZShmMut {
    fn as_ref(&self) -> &[u8] {
        self
    }
}

impl AsMut<[u8]> for ZShmMut {
    fn as_mut(&mut self) -> &mut [u8] {
        self
    }
}

impl From<ZShmMut> for ZShm {
    fn from(value: ZShmMut) -> Self {
        ZShm::new(value.inner)
    }
}

impl From<ZShmMut> for ZSlice {
    fn from(value: ZShmMut) -> Self {
        value.inner.into()
    }
}

impl From<ZShmMut> for ZBuf {
    fn from(value: ZShmMut) -> Self {
        value.inner.into()
    }
}

/// A borrowed mutable SHM buffer
#[zenoh_macros::unstable_doc]
#[derive(Debug, PartialEq, Eq)]
#[allow(non_camel_case_types)]
#[repr(transparent)]
pub struct zshmmut {
    pub(crate) inner: ShmBufInner,
}

impl PartialEq<ZShmMut> for &zshmmut {
    fn eq(&self, other: &ZShmMut) -> bool {
        self.inner == other.inner
    }
}

impl ShmBuf<[u8]> for &zshmmut {
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }
}

impl ShmBuf<[u8]> for &mut zshmmut {
    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }
}

impl ShmBufUnsafeMut<[u8]> for &mut zshmmut {
    unsafe fn as_mut_unchecked(&mut self) -> &mut [u8] {
        self.inner.as_mut_slice_inner()
    }
}

impl<'a> ShmBufIntoImmut<[u8]> for &'a zshmmut {
    type ImmutBuf = &'a zshm;

    fn into_immut(self) -> Self::ImmutBuf {
        self.into()
    }
}

impl<'a> ShmBufIntoImmut<[u8]> for &'a mut zshmmut {
    type ImmutBuf = &'a mut zshm;

    fn into_immut(self) -> Self::ImmutBuf {
        self.into()
    }
}

impl ShmBufMut<[u8]> for &mut zshmmut {}

impl Deref for zshmmut {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.inner.as_ref()
    }
}

impl DerefMut for zshmmut {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.as_mut()
    }
}

impl AsRef<[u8]> for zshmmut {
    fn as_ref(&self) -> &[u8] {
        self
    }
}

impl AsMut<[u8]> for zshmmut {
    fn as_mut(&mut self) -> &mut [u8] {
        self
    }
}

impl TryFrom<&zshm> for &zshmmut {
    type Error = ();

    fn try_from(value: &zshm) -> Result<Self, Self::Error> {
        match value.inner.is_unique() && value.inner.is_valid() {
            true => {
                // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
                // to ShmBufInner type, so it is safe to transmute them in any direction
                Ok(unsafe { core::mem::transmute::<&zshm, &zshmmut>(value) })
            }
            false => Err(()),
        }
    }
}

impl TryFrom<&mut zshm> for &mut zshmmut {
    type Error = ();

    fn try_from(value: &mut zshm) -> Result<Self, Self::Error> {
        match value.inner.is_unique() && value.inner.is_valid() {
            true => {
                // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
                // to ShmBufInner type, so it is safe to transmute them in any direction
                Ok(unsafe { core::mem::transmute::<&mut zshm, &mut zshmmut>(value) })
            }
            false => Err(()),
        }
    }
}

impl TryFrom<&mut ZShm> for &mut zshmmut {
    type Error = ();

    fn try_from(value: &mut ZShm) -> Result<Self, Self::Error> {
        match value.inner.is_unique() && value.inner.is_valid() {
            true => {
                // SAFETY: ZShm, ZShmMut, zshm and zshmmut are #[repr(transparent)]
                // to ShmBufInner type, so it is safe to transmute them in any direction
                Ok(unsafe { core::mem::transmute::<&mut ZShm, &mut zshmmut>(value) })
            }
            false => Err(()),
        }
    }
}
