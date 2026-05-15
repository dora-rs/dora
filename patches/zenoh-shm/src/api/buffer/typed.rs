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

use std::{
    marker::PhantomData,
    mem,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
};

use zenoh_result::{bail, ZResult};

use crate::api::buffer::{
    traits::{ResideInShm, ShmBuf, ShmBufIntoImmut, ShmBufMut, ShmBufUnsafeMut},
    zshm::{zshm, ZShm},
    zshmmut::{zshmmut, ZShmMut},
};

/// Wrapper for SHM buffer types that is used for safe typed access to SHM data
pub struct Typed<T, Buf> {
    buf: Buf,
    _phantom: PhantomData<T>,
}

impl<T, Buf: Clone> Clone for Typed<T, Buf> {
    fn clone(&self) -> Self {
        Self {
            buf: self.buf.clone(),
            _phantom: PhantomData,
        }
    }
}

impl<T, Buf> Typed<T, Buf> {
    /// Mark a buffer as typed.
    ///
    /// # Safety
    ///
    /// The buffer layout must be compatible with `T` layout, its bytes should be initialized and respect `T` invariants, unless `T` is [`MaybeUninit`].
    pub unsafe fn new_unchecked(buf: Buf) -> Self {
        Self {
            buf,
            _phantom: PhantomData,
        }
    }

    /// Get the underlying SHM buffer
    pub fn inner(this: &Self) -> &Buf {
        &this.buf
    }

    /// Get the underlying SHM buffer
    pub fn inner_mut(this: &mut Self) -> &mut Buf {
        &mut this.buf
    }

    /// Convert into underlying SHM buffer
    pub fn into_inner(this: Self) -> Buf {
        this.buf
    }
}

impl<T, Buf> Typed<MaybeUninit<T>, Buf> {
    /// Mark a buffer as typed, checking its layout.
    ///
    pub fn new(buf: Buf) -> ZResult<Self>
    where
        Buf: AsRef<[u8]>,
    {
        let slice = buf.as_ref();
        if slice.len() != mem::size_of::<T>() {
            bail!(
                "Slice length does not match type size: expected {}, got {}",
                mem::size_of::<T>(),
                slice.len()
            );
        }
        if (slice.as_ptr() as usize) % mem::align_of::<T>() != 0 {
            bail!(
                "Slice alignment does not match type alignment: expected {}, got {}",
                mem::align_of::<T>(),
                1 << (slice.as_ptr() as usize).trailing_zeros()
            );
        }
        // SAFETY: the layout has been checked, and type is `MaybeUninit`
        Ok(unsafe { Self::new_unchecked(buf) })
    }
    /// Assumes the underlying data is initialized.
    ///
    /// # Safety
    ///
    /// `T` bytes must have been properly initialized.
    pub unsafe fn assume_init(self) -> Typed<T, Buf> {
        Typed {
            buf: self.buf,
            _phantom: PhantomData,
        }
    }

    /// Initializes the underlying data.
    pub fn initialize(mut self, value: T) -> Typed<T, Buf>
    where
        Buf: ShmBufMut<[u8]>,
    {
        // SAFETY: this is safe because we check transmute safety when constructing self
        unsafe { self.buf.as_mut().as_mut_ptr().cast::<T>().write(value) };
        // SAFETY: the data has been initialized
        unsafe { self.assume_init() }
    }
}

impl<T> From<Typed<T, ZShmMut>> for Typed<T, ZShm> {
    fn from(value: Typed<T, ZShmMut>) -> Self {
        Self {
            buf: value.buf.into(),
            _phantom: PhantomData,
        }
    }
}

impl<T> TryFrom<Typed<T, ZShm>> for Typed<T, ZShmMut> {
    type Error = Typed<T, ZShm>;

    fn try_from(value: Typed<T, ZShm>) -> Result<Self, Self::Error> {
        Ok(Self {
            buf: value.buf.try_into().map_err(|e| Typed::<T, ZShm> {
                buf: e,
                _phantom: PhantomData,
            })?,
            _phantom: PhantomData,
        })
    }
}

impl<'a, T> TryFrom<Typed<T, &'a zshm>> for Typed<T, &'a zshmmut> {
    type Error = ();

    fn try_from(value: Typed<T, &'a zshm>) -> Result<Self, Self::Error> {
        Ok(Self {
            buf: value.buf.try_into()?,
            _phantom: PhantomData,
        })
    }
}

impl<'a, T> TryFrom<Typed<T, &'a mut zshm>> for Typed<T, &'a mut zshmmut> {
    type Error = ();

    fn try_from(value: Typed<T, &'a mut zshm>) -> Result<Self, Self::Error> {
        Ok(Self {
            buf: value.buf.try_into()?,
            _phantom: PhantomData,
        })
    }
}

impl<'a, T> TryFrom<Typed<T, &'a mut ZShm>> for Typed<T, &'a mut zshmmut> {
    type Error = ();

    fn try_from(value: Typed<T, &'a mut ZShm>) -> Result<Self, Self::Error> {
        Ok(Self {
            buf: value.buf.try_into()?,
            _phantom: PhantomData,
        })
    }
}

impl<T> From<Typed<T, &zshmmut>> for Typed<T, &zshm> {
    fn from(value: Typed<T, &zshmmut>) -> Self {
        Self {
            buf: value.buf.into(),
            _phantom: PhantomData,
        }
    }
}

impl<T> From<Typed<T, &mut zshmmut>> for Typed<T, &mut zshm> {
    fn from(value: Typed<T, &mut zshmmut>) -> Self {
        Self {
            buf: value.buf.into(),
            _phantom: PhantomData,
        }
    }
}

impl<T: ResideInShm, Buf: ShmBuf<[u8]>> Deref for Typed<T, Buf> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        // SAFETY: this is safe because we check transmute safety when constructing self
        unsafe { &*(self.buf.as_ref().as_ptr() as *const T) }
    }
}

impl<T: ResideInShm, Buf: ShmBuf<[u8]>> AsRef<T> for Typed<T, Buf> {
    fn as_ref(&self) -> &T {
        self
    }
}

impl<T: ResideInShm, Buf: ShmBufMut<[u8]>> DerefMut for Typed<T, Buf> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: this is safe because we check transmute safety when constructing self
        unsafe { &mut *(self.buf.as_mut().as_mut_ptr() as *mut T) }
    }
}

impl<T: ResideInShm, Buf: ShmBufMut<[u8]>> AsMut<T> for Typed<T, Buf> {
    fn as_mut(&mut self) -> &mut T {
        self
    }
}

impl<T: ResideInShm, Buf: ShmBuf<[u8]>> ShmBuf<T> for Typed<T, Buf> {
    fn is_valid(&self) -> bool {
        self.buf.is_valid()
    }
}

impl<T: ResideInShm, Buf: ShmBufMut<[u8]>> ShmBufMut<T> for Typed<T, Buf> {}

impl<T: ResideInShm, Buf: ShmBufUnsafeMut<[u8]>> ShmBufUnsafeMut<T> for Typed<T, Buf> {
    unsafe fn as_mut_unchecked(&mut self) -> &mut T {
        &mut *(self.buf.as_mut_unchecked().as_mut_ptr() as *mut T)
    }
}

impl<T: ResideInShm, Buf: ShmBufIntoImmut<[u8]>> ShmBufIntoImmut<T> for Typed<T, Buf> {
    type ImmutBuf = Typed<T, Buf::ImmutBuf>;

    fn into_immut(self) -> Self::ImmutBuf {
        Typed {
            buf: self.buf.into_immut(),
            _phantom: PhantomData,
        }
    }
}
