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

use std::{fmt::Display, marker::PhantomData, mem, num::NonZeroUsize};

use crate::api::provider::types::{AllocAlignment, ZLayoutError};

/// Memory layout representation: alignment and size aligned for this alignment
#[zenoh_macros::unstable_doc]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MemoryLayout {
    size: NonZeroUsize,
    alignment: AllocAlignment,
}

impl From<&MemoryLayout> for MemoryLayout {
    fn from(other: &MemoryLayout) -> Self {
        *other
    }
}

impl Display for MemoryLayout {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!(
            "[size={},alignment={}]",
            self.size, self.alignment
        ))
    }
}

impl MemoryLayout {
    /// Try to create a new memory layout.
    ///
    /// # Errors
    ///
    /// This function will return an error if zero size have passed or if the provided size is not the multiply of the alignment.
    #[zenoh_macros::unstable_doc]
    pub fn new<T>(size: T, alignment: AllocAlignment) -> Result<Self, ZLayoutError>
    where
        T: TryInto<NonZeroUsize>,
    {
        let Ok(size) = size.try_into() else {
            return Err(ZLayoutError::IncorrectLayoutArgs);
        };

        // size of a layout must be a multiple of its alignment!
        match size.get() % alignment.get_alignment_value() {
            0 => Ok(Self { size, alignment }),
            _ => Err(ZLayoutError::IncorrectLayoutArgs),
        }
    }

    /// #SAFETY: this is safe if size is a multiply of alignment
    /// Note: not intended for public APIs as it is really very unsafe
    const unsafe fn new_unchecked(size: NonZeroUsize, alignment: AllocAlignment) -> Self {
        Self { size, alignment }
    }

    /// Creates a new `MemoryLayout` for type.
    #[allow(path_statements)]
    #[zenoh_macros::unstable_doc]
    pub const fn for_type<T: Sized>() -> Self {
        struct ZstCheck<T>(PhantomData<T>);
        impl<T> ZstCheck<T> {
            const NOT_ZST: () = assert!(mem::size_of::<T>() != 0, "`T` must not be a ZST");
        }
        ZstCheck::<T>::NOT_ZST;
        // SAFETY: invariant checked above
        let size = unsafe { NonZeroUsize::new_unchecked(mem::size_of::<T>()) };
        let alignment = AllocAlignment::for_type::<T>();
        unsafe { Self::new_unchecked(size, alignment) }
    }

    /// Creates a new `MemoryLayout` for value type.
    #[zenoh_macros::unstable_doc]
    pub const fn for_value<T: Sized>(_: &T) -> Self {
        Self::for_type::<T>()
    }

    #[zenoh_macros::unstable_doc]
    pub fn size(&self) -> NonZeroUsize {
        self.size
    }

    #[zenoh_macros::unstable_doc]
    pub fn alignment(&self) -> AllocAlignment {
        self.alignment
    }

    /// Realign the layout for new alignment. The alignment must be >= of the existing one.
    ///
    /// # Examples
    ///
    /// ```
    /// use zenoh_shm::api::provider::types::AllocAlignment;
    /// use zenoh_shm::api::provider::memory_layout::MemoryLayout;
    ///
    /// // 8 bytes with 4-byte alignment
    /// let layout4b = MemoryLayout::new(8, AllocAlignment::new(2).unwrap()).unwrap();
    ///
    /// // Try to realign with 2-byte alignment
    /// let layout2b = layout4b.extend(AllocAlignment::new(1).unwrap());
    /// assert!(layout2b.is_err()); // fails because new alignment must be >= old
    ///
    /// // Try to realign with 8-byte alignment
    /// let layout8b = layout4b.extend(AllocAlignment::new(3).unwrap());
    /// assert!(layout8b.is_ok()); // ok
    /// ```
    #[zenoh_macros::unstable_doc]
    pub fn extend(&self, new_alignment: AllocAlignment) -> Result<MemoryLayout, ZLayoutError> {
        if new_alignment < self.alignment {
            return Err(ZLayoutError::IncorrectLayoutArgs);
        }
        let new_size = new_alignment.align_size(self.size);
        MemoryLayout::new(new_size, new_alignment)
    }
}

impl TryFrom<NonZeroUsize> for MemoryLayout {
    type Error = ZLayoutError;

    fn try_from(value: NonZeroUsize) -> Result<Self, Self::Error> {
        MemoryLayout::new(value, AllocAlignment::ALIGN_1_BYTE)
    }
}

impl TryFrom<usize> for MemoryLayout {
    type Error = ZLayoutError;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        MemoryLayout::new(value, AllocAlignment::ALIGN_1_BYTE)
    }
}

impl TryFrom<(NonZeroUsize, AllocAlignment)> for MemoryLayout {
    type Error = ZLayoutError;

    fn try_from(value: (NonZeroUsize, AllocAlignment)) -> Result<Self, Self::Error> {
        MemoryLayout::new(value.0, value.1)
    }
}

impl TryFrom<(usize, AllocAlignment)> for MemoryLayout {
    type Error = ZLayoutError;

    fn try_from(value: (usize, AllocAlignment)) -> Result<Self, Self::Error> {
        MemoryLayout::new(value.0, value.1)
    }
}

/// A statically-known layout with type information.
///
/// Used in context of typed operations.
///
/// Statically-known layouts are always correct, zero-sized & zero-cost.
#[zenoh_macros::unstable_doc]
pub struct TypedLayout<T> {
    _phantom: PhantomData<T>,
}

impl<T> Default for TypedLayout<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> TypedLayout<T> {
    /// Creates a new `TypedLayout` for type
    #[zenoh_macros::unstable_doc]
    pub fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    /// Creates a new `TypedLayout` for value type
    #[zenoh_macros::unstable_doc]
    pub fn for_value(_: &T) -> Self {
        Self::new()
    }
}

impl<T> Clone for TypedLayout<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for TypedLayout<T> {}
