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
#![cfg(feature = "test")]
use zenoh_shm::shm::Segment;

#[test]
fn create() {
    let id = (line!() as u64) + ((std::process::id() as u64) << 32);
    let len = 1024.try_into().unwrap();
    let created_segment = Segment::create(id, len).unwrap();
    assert!(created_segment.len() >= len);
}

#[test]
fn create_concurrent() {
    let id = (line!() as u64) + ((std::process::id() as u64) << 32);
    let len = 1024.try_into().unwrap();
    let created_segment = Segment::create(id, len).unwrap();
    assert!(created_segment.len() >= len);
    assert!(Segment::create(id, len).is_err());
}

#[test]
fn create_and_open() {
    let id = (line!() as u64) + ((std::process::id() as u64) << 32);
    let len = 1024.try_into().unwrap();
    let created_segment = Segment::create(id, len).unwrap();
    let opened_segment = Segment::open(id).unwrap();
    assert!(created_segment.len() >= len);
    assert!(opened_segment.len() >= len);
}

#[test]
fn create_and_open_and_reopen() {
    let id = (line!() as u64) + ((std::process::id() as u64) << 32);
    let len = 1024.try_into().unwrap();
    let created_segment = Segment::create(id, len).unwrap();
    let opened_segment = Segment::open(id).unwrap();
    let opened_segment2 = Segment::open(id).unwrap();
    assert!(created_segment.len() >= len);
    assert!(opened_segment.len() >= len);
    assert!(opened_segment2.len() >= len);
}

#[test]
fn create_and_open_and_reopen_and_open_closed() {
    let id = (line!() as u64) + ((std::process::id() as u64) << 32);
    let len = 1024.try_into().unwrap();
    let created_segment = Segment::create(id, len).unwrap();
    let opened_segment = Segment::open(id).unwrap();
    assert!(created_segment.len() >= len);
    assert!(opened_segment.len() >= len);

    drop(created_segment);

    let opened_segment2 = Segment::open(id).unwrap();
    assert!(opened_segment2.len() >= len);
}

#[test]
fn no_persistency() {
    let id = (line!() as u64) + ((std::process::id() as u64) << 32);
    let len = 1024.try_into().unwrap();
    let created_segment = Segment::create(id, len).unwrap();
    assert!(created_segment.len() >= len);
    drop(created_segment);

    assert!(Segment::open(id).is_err());
}

#[test]
fn recreate_many_times() {
    let id = (line!() as u64) + ((std::process::id() as u64) << 32);
    let len = 1024.try_into().unwrap();

    for _ in 0..100 {
        let created_segment = Segment::create(id, len).unwrap();
        let opened_segment = Segment::open(id).unwrap();
        assert!(created_segment.len() >= len);
        assert!(opened_segment.len() >= len);
    }

    assert!(Segment::open(id).is_err());
}

#[test]
fn expect_no_overcommit() {
    let id = (line!() as u64) + ((std::process::id() as u64) << 32);
    let len = (1024 * 1024 * 1024 * 1024 * 1024).try_into().unwrap(); // 1 PB of memory
    assert!(Segment::create(id, len).is_err());
}
