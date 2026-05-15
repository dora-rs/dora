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
    mem::size_of,
    num::NonZeroUsize,
    ops::{Deref, DerefMut},
};

// use stabby::IStable;
use zenoh_result::ZResult;

use super::segment::Segment;
use crate::shm;

/// An SHM segment that contains data structure
#[derive(Debug)]
pub struct StructInSHM<ID, Elem>
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    ID: shm::SegmentID,
{
    inner: Segment<ID>,
    _phantom: PhantomData<Elem>,
}

unsafe impl<ID, Elem: Sync> Sync for StructInSHM<ID, Elem>
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    ID: shm::SegmentID,
{
}
unsafe impl<ID, Elem: Send> Send for StructInSHM<ID, Elem>
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    ID: shm::SegmentID,
{
}

impl<ID, Elem> StructInSHM<ID, Elem>
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    // Elem: IStable<ContainsIndirections = stabby::abi::B0>, // TODO: stabby does not support IStable for big arrays
    ID: shm::SegmentID,
{
    // Perform compile time check that Elem is not a ZST
    const _S: () = if size_of::<Elem>() == 0 {
        panic!("Elem is a ZST. ZSTs are not allowed");
    };

    pub fn create() -> ZResult<Self> {
        let alloc_size = NonZeroUsize::try_from(size_of::<Elem>())?;
        let inner = Segment::create(alloc_size)?;
        Ok(Self {
            inner,
            _phantom: PhantomData,
        })
    }

    pub fn open(id: ID) -> ZResult<Self> {
        let inner = Segment::open(id)?;
        Ok(Self {
            inner,
            _phantom: PhantomData,
        })
    }

    pub fn id(&self) -> ID {
        self.inner.id()
    }

    /// Retrieves mut element
    pub fn elem_mut(&self) -> *mut Elem {
        self.inner.as_ptr() as *mut Elem
    }
}

impl<ID, Elem> Deref for StructInSHM<ID, Elem>
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    // Elem: IStable<ContainsIndirections = stabby::abi::B0>, // TODO: stabby does not support IStable for big arrays
    ID: shm::SegmentID,
{
    type Target = Elem;

    fn deref(&self) -> &Self::Target {
        unsafe { &*(self.inner.as_ptr() as *const Elem) }
    }
}

impl<ID, Elem> DerefMut for StructInSHM<ID, Elem>
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
    // Elem: IStable<ContainsIndirections = stabby::abi::B0>, // TODO: stabby does not support IStable for big arrays
    ID: shm::SegmentID,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut *(self.inner.as_ptr() as *mut Elem) }
    }
}
