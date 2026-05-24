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
use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use zenoh_shm::watchdog::periodic_task::{PeriodicTask, TaskWakeReason};

pub mod common;
use common::CpuLoad;

const TASK_PERIOD: Duration = Duration::from_millis(50);
const TASK_DELTA: Duration = Duration::from_millis(5);
const TEST_TASK: Duration = Duration::from_millis(10);

fn intensive_payload(duration: Duration) -> impl Fn(TaskWakeReason) + Send + 'static {
    move |_| {
        let start = Instant::now();
        while start.elapsed() < duration {
            for _i in 0..100 {}
        }
    }
}

fn blocking_payload(duration: Duration) -> impl Fn(TaskWakeReason) + Send + 'static {
    move |_| {
        std::thread::sleep(duration);
    }
}

fn check_duration(duration: &Duration) {
    let min = TASK_PERIOD - TASK_DELTA;
    let max = TASK_PERIOD + TASK_DELTA;

    assert!(min <= *duration && *duration <= max);
}

fn make_task<F>(task_payload: F) -> (PeriodicTask, Arc<Mutex<Vec<Duration>>>)
where
    F: Fn(TaskWakeReason) + Send + 'static,
{
    let intervals = Arc::new(Mutex::new(vec![]));

    let c_intervals = intervals.clone();
    let mut start: Option<Instant> = None;
    let task = PeriodicTask::new("test".to_owned(), TASK_PERIOD, move |reason| {
        if let Some(val) = &start {
            let elapsed = val.elapsed();
            c_intervals.lock().unwrap().push(elapsed);
        }
        start = Some(Instant::now());
        task_payload(reason);
    });

    (task, intervals)
}

#[test]
#[ignore]
fn periodic_task_create() {
    let (_task, _intervals) = make_task(|_| {});
}

fn check_task<F>(task_payload: F)
where
    F: Fn(TaskWakeReason) + Send + 'static,
{
    let n = 100;
    let (task, intervals) = make_task(task_payload);

    std::thread::sleep(TASK_PERIOD * n);
    drop(task);

    let guard = intervals.lock().unwrap();
    for duration in &*guard {
        check_duration(duration);
    }
}

#[test]
#[ignore]
fn periodic_task_lightweight() {
    check_task(|_| {});
}

#[test]
#[ignore]
fn periodic_task_blocking() {
    check_task(blocking_payload(TEST_TASK));
}

#[test]
#[ignore]
fn periodic_task_intensive() {
    check_task(intensive_payload(TEST_TASK));
}

#[test]
#[ignore]
fn periodic_task_low_load_lightweight() {
    let _load = CpuLoad::low();
    check_task(|_| {});
}

#[test]
#[ignore]
fn periodic_task_low_load_blocking() {
    let _load = CpuLoad::low();
    check_task(blocking_payload(TEST_TASK));
}

#[test]
#[ignore]
fn periodic_task_low_load_intensive() {
    let _load = CpuLoad::low();
    check_task(intensive_payload(TEST_TASK));
}

#[test]
#[ignore]
fn periodic_task_optimal_high_load_lightweight() {
    let _load = CpuLoad::optimal_high();
    check_task(|_| {});
}

#[test]
#[ignore]
fn periodic_task_optimal_high_load_blocking() {
    let _load = CpuLoad::optimal_high();
    check_task(blocking_payload(TEST_TASK));
}

#[test]
#[ignore]
fn periodic_task_optimal_high_load_intensive() {
    let _load = CpuLoad::optimal_high();
    check_task(intensive_payload(TEST_TASK));
}

#[test]
#[ignore]
fn periodic_task_excessive_load_lightweight() {
    let _load = CpuLoad::excessive();
    check_task(|_| {});
}

#[test]
#[ignore]
fn periodic_task_excessive_load_blocking() {
    let _load = CpuLoad::excessive();
    check_task(blocking_payload(TEST_TASK));
}

#[test]
#[ignore]
fn periodic_task_excessive_load_intensive() {
    let _load = CpuLoad::excessive();
    check_task(intensive_payload(TEST_TASK));
}
