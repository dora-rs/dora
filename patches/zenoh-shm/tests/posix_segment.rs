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
#![cfg(feature = "test")]
use std::{num::NonZeroUsize, slice};

use zenoh_shm::{posix_shm::segment::Segment, shm::SegmentID};

pub mod common;
use common::validate_memory;

fn validate_segment<ID: SegmentID>(
    created_segment: &Segment<ID>,
    opened_segment: &Segment<ID>,
    expected_elem_count: NonZeroUsize,
) where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
{
    assert!(created_segment.len() >= expected_elem_count);
    assert!(opened_segment.len() >= expected_elem_count);
    assert!(opened_segment.len() == created_segment.len());

    let ptr1 = created_segment.as_ptr();
    let ptr2 = opened_segment.as_ptr();

    let slice1 = unsafe { slice::from_raw_parts_mut(ptr1, created_segment.len().get()) };
    let slice2 = unsafe { slice::from_raw_parts(ptr2, opened_segment.len().get()) };

    validate_memory(slice1, slice2);
}

fn test_segment<ID: SegmentID>()
where
    rand::distributions::Standard: rand::distributions::Distribution<ID>,
{
    let elem_count = 900.try_into().unwrap();

    let created_segment: Segment<ID> =
        Segment::create(elem_count).expect("error creating new segment");

    let opened_segment_instance_1 =
        Segment::open(created_segment.id()).expect("error opening existing segment!");

    validate_segment(&created_segment, &opened_segment_instance_1, elem_count);

    let opened_segment_instance_2 =
        Segment::open(created_segment.id()).expect("error opening existing segment!");

    validate_segment(&created_segment, &opened_segment_instance_1, elem_count);
    validate_segment(&created_segment, &opened_segment_instance_2, elem_count);

    drop(opened_segment_instance_1);
    validate_segment(&created_segment, &opened_segment_instance_2, elem_count);
}

/// UNSIGNED ///

#[test]
fn segment_u8_id() {
    test_segment::<u8>()
}

#[test]
fn segment_u16_id() {
    test_segment::<u16>()
}

#[test]
fn segment_u32_id() {
    test_segment::<u32>()
}

#[test]
fn segment_u64_id() {
    test_segment::<u64>()
}

// TODO: this is not yet supported (produces too long shm name for Mac),
// but we don't really need this
//#[test]
//fn segment_u128_id() {
//    test_segment::<u128>()
//}

/// Behaviour checks ///

#[test]
fn segment_open() {
    let created_segment: Segment<u8> =
        Segment::create(900.try_into().unwrap()).expect("error creating new segment");

    let _opened_segment =
        Segment::open(created_segment.id()).expect("error opening existing segment!");
}

#[test]
fn segment_open_error() {
    let id = {
        let created_segment: Segment<u8> =
            Segment::create(900.try_into().unwrap()).expect("error creating new segment");
        created_segment.id()
    };

    let _opened_segment = Segment::open(id).expect_err("must fail: opened not existing segment!");
}
