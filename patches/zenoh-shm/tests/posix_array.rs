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

use std::{fmt::Debug, mem::size_of, num::NonZeroUsize};

use num_traits::{AsPrimitive, PrimInt, Unsigned};
use zenoh_shm::posix_shm::array::ArrayInSHM;

pub mod common;

type TestSegmentID = u32;

#[derive(Debug)]
#[stabby::stabby]
struct TestElem {
    value: u32,
}

impl TestElem {
    fn fill(&mut self, counter: &mut u32) {
        self.value = *counter;
        *counter += 1;
    }

    fn validate(&self, counter: &mut u32) {
        assert_eq!(self.value, *counter);
        *counter += 1;
    }
}

fn validate_array<ElemIndex>(
    created_array: &mut ArrayInSHM<TestSegmentID, TestElem, ElemIndex>,
    opened_array: &ArrayInSHM<TestSegmentID, TestElem, ElemIndex>,
    expected_elem_count: NonZeroUsize,
) where
    ElemIndex: Unsigned + PrimInt + 'static + AsPrimitive<usize>,
    isize: AsPrimitive<ElemIndex>,
    usize: AsPrimitive<ElemIndex>,
{
    assert!(created_array.elem_count() >= expected_elem_count);
    assert!(opened_array.elem_count() >= expected_elem_count);
    assert!(opened_array.elem_count() == created_array.elem_count());

    let mut fill_ctr = 0;
    let mut validate_ctr = 0;

    // first of all, fill and validate elements sequentially
    for i in 0..created_array.elem_count().get() {
        unsafe {
            let elem1 = &mut *created_array.elem_mut(i.as_());
            let elem2 = &*opened_array.elem(i.as_());

            elem1.fill(&mut fill_ctr);
            elem2.validate(&mut validate_ctr);
        }
    }

    // then fill all the elements...
    for i in 0..created_array.elem_count().get() {
        unsafe {
            let elem1 = &mut *created_array.elem_mut(i.as_());
            elem1.fill(&mut fill_ctr);
        }
    }

    // ...and validate all the elements
    for i in 0..opened_array.elem_count().get() {
        unsafe {
            let elem2 = &*opened_array.elem(i.as_());
            elem2.validate(&mut validate_ctr);
        }
    }
}

fn test_array<ElemIndex>()
where
    ElemIndex: Unsigned + PrimInt + 'static + AsPrimitive<usize>,
    isize: AsPrimitive<ElemIndex>,
    usize: AsPrimitive<ElemIndex>,
{
    // Estimate elem count to test
    // NOTE: for index sizes <= 16 bit we use the whole index range to test,
    // and for bigger indexes we use limited index range
    let elem_count = NonZeroUsize::new({
        match size_of::<ElemIndex>() > size_of::<u16>() {
            true => 100,
            false => ElemIndex::max_value().as_() + 1,
        }
    })
    .unwrap();

    let mut new_arr: ArrayInSHM<TestSegmentID, TestElem, ElemIndex> =
        ArrayInSHM::create(elem_count).expect("error creating new array!");

    let opened_arr: ArrayInSHM<_, TestElem, ElemIndex> =
        ArrayInSHM::open(new_arr.id()).expect("error opening existing array!");

    validate_array(&mut new_arr, &opened_arr, elem_count);
}

/// MEMORY CHECKS ///

#[test]
fn arr_u8_index_memory_test() {
    test_array::<u8>();
}

#[test]
fn arr_u16_index_memory_test() {
    test_array::<u16>();
}

#[test]
fn arr_u32_index_memory_test() {
    test_array::<u32>();
}

/// ELEM COUNT CHECKS ///
fn test_invalid_elem_index<ElemIndex>()
where
    ElemIndex: Unsigned + PrimInt + 'static + AsPrimitive<usize> + Debug,
    isize: AsPrimitive<ElemIndex>,
    usize: AsPrimitive<ElemIndex>,
{
    let invalid_elem_count = ElemIndex::max_value().as_() + 2;

    let _ = ArrayInSHM::<TestSegmentID, TestElem, ElemIndex>::create(
        invalid_elem_count.try_into().unwrap(),
    )
    .expect_err(
        format!("must fail: element count {invalid_elem_count} is out of range for ElemIndex!")
            .as_str(),
    );
}

#[test]
fn arr_u8_index_invalid_elem_count() {
    test_invalid_elem_index::<u8>();
}

#[test]
fn arr_u16_index_invalid_elem_count() {
    test_invalid_elem_index::<u16>();
}

#[test]
fn arr_u32_index_invalid_elem_count() {
    test_invalid_elem_index::<u32>();
}
