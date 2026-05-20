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
use std::{ops::Deref, sync::atomic::Ordering::Relaxed, time::Duration};

use rand::Rng;
use tracing::error;
use zenoh_result::ZResult;

pub mod common;
use common::{execute_concurrent, CpuLoad};
use zenoh_shm::{
    api::provider::types::ZAllocError,
    metadata::{
        descriptor::MetadataDescriptor, storage::GLOBAL_METADATA_STORAGE,
        subscription::GLOBAL_METADATA_SUBSCRIPTION,
    },
    watchdog::{confirmator::GLOBAL_CONFIRMATOR, validator::GLOBAL_VALIDATOR},
};

fn metadata_alloc_fn(
) -> impl Fn(usize, usize) -> Result<(), ZAllocError> + Clone + Send + Sync + 'static {
    |_task_index: usize, _iteration: usize| {
        let _allocated_metadata = GLOBAL_METADATA_STORAGE.read().allocate()?;
        Ok(())
    }
}

#[test]
fn metadata_alloc() {
    execute_concurrent(1, 1000, metadata_alloc_fn());
}

#[test]
fn metadata_alloc_concurrent() {
    execute_concurrent(100, 1000, metadata_alloc_fn());
}

fn metadata_link_fn(
) -> impl Fn(usize, usize) -> Result<(), ZAllocError> + Clone + Send + Sync + 'static {
    |_task_index: usize, _iteration: usize| {
        let allocated_metadata = GLOBAL_METADATA_STORAGE.read().allocate()?;
        let descr = MetadataDescriptor::from(allocated_metadata.deref());
        let _linked_metadata = GLOBAL_METADATA_SUBSCRIPTION.read().link(&descr)?;
        Ok(())
    }
}

#[test]
fn metadata_link() {
    execute_concurrent(1, 1000, metadata_link_fn());
}

#[test]
fn metadata_link_concurrent() {
    execute_concurrent(100, 1000, metadata_link_fn());
}

fn metadata_link_failure_fn(
) -> impl Fn(usize, usize) -> Result<(), ZAllocError> + Clone + Send + Sync + 'static {
    |_task_index: usize, _iteration: usize| {
        let allocated_metadata = GLOBAL_METADATA_STORAGE.read().allocate()?;
        let descr = MetadataDescriptor::from(allocated_metadata.deref());
        drop(allocated_metadata);

        // Some comments on this behaviour...
        // Even though the allocated_metadata is dropped, its SHM segment still exists in GLOBAL_METADATA_STORAGE,
        // so there is no way to detect that metadata is "deallocated" and the code below succeeds. The invalidation
        // functionality is implemented on higher level by means of a generation mechanism that protects from both metadata
        // and watchdog link-to-deallocated issues. This generation mechanism depends on the behaviour below, so
        // everything is fair :)
        let _linked_metadata = GLOBAL_METADATA_SUBSCRIPTION.read().link(&descr)?;
        Ok(())
    }
}

#[test]
fn metadata_link_failure() {
    execute_concurrent(1, 1000, metadata_link_failure_fn());
}

#[test]
fn metadata_link_failure_concurrent() {
    execute_concurrent(100, 1000, metadata_link_failure_fn());
}

fn metadata_check_memory_fn(parallel_tasks: usize, iterations: usize) {
    let task_fun = |_task_index: usize, _iteration: usize| -> Result<(), ZAllocError> {
        let allocated_metadata = GLOBAL_METADATA_STORAGE.read().allocate()?;
        let descr = MetadataDescriptor::from(allocated_metadata.deref());
        let linked_metadata = GLOBAL_METADATA_SUBSCRIPTION.read().link(&descr)?;

        let mut rng = rand::thread_rng();
        let allocated = allocated_metadata.header();
        let linked = linked_metadata.header();
        for _ in 0..100 {
            let gen = rng.gen();
            allocated.generation.store(gen, Relaxed);
            assert_eq!(gen, linked.generation.load(Relaxed));

            let rc = rng.gen();
            allocated.refcount.store(rc, Relaxed);
            assert_eq!(rc, linked.refcount.load(Relaxed));

            let watchdog_inv = rng.gen();
            allocated.watchdog_invalidated.store(watchdog_inv, Relaxed);
            assert_eq!(watchdog_inv, linked.watchdog_invalidated.load(Relaxed));

            assert_eq!(gen, linked.generation.load(Relaxed));
            assert_eq!(rc, linked.refcount.load(Relaxed));
            assert_eq!(watchdog_inv, linked.watchdog_invalidated.load(Relaxed));
        }
        Ok(())
    };
    execute_concurrent(parallel_tasks, iterations, task_fun);
}

#[test]
fn metadata_check_memory() {
    metadata_check_memory_fn(1, 1000);
}

#[test]
fn metadata_check_memory_concurrent() {
    metadata_check_memory_fn(100, 100);
}

const VALIDATION_PERIOD: Duration = Duration::from_millis(100);
const CONFIRMATION_PERIOD: Duration = Duration::from_millis(50);

fn watchdog_confirmed_fn(
) -> impl Fn(usize, usize) -> Result<(), ZAllocError> + Clone + Send + Sync + 'static {
    |_task_index: usize, _iteration: usize| {
        let allocated = GLOBAL_METADATA_STORAGE.read().allocate()?;
        let confirmed = GLOBAL_CONFIRMATOR.read().add(allocated.clone());

        // check that the confirmed watchdog stays valid
        for i in 0..10 {
            std::thread::sleep(VALIDATION_PERIOD);
            let valid = confirmed.owned.test_validate() != 0;
            if !valid {
                error!("Invalid watchdog, iteration {i}");
                return Err(ZAllocError::Other);
            }
        }
        Ok(())
    }
}

#[test]
#[ignore]
fn watchdog_confirmed() {
    execute_concurrent(1, 10, watchdog_confirmed_fn());
}

#[test]
#[ignore]
fn watchdog_confirmed_concurrent() {
    execute_concurrent(1000, 10, watchdog_confirmed_fn());
}

// TODO: confirmation to dangling watchdog actually writes to potentially-existing
// other watchdog instance from other test running in the same process and changes it's behaviour,
// so we cannot run dangling test in parallel with anything else
#[test]
#[ignore]
fn watchdog_confirmed_dangling() {
    let allocated = GLOBAL_METADATA_STORAGE
        .read()
        .allocate()
        .expect("error allocating watchdog!");
    let confirmed = GLOBAL_CONFIRMATOR.read().add(allocated.clone());
    drop(allocated);

    // confirm dangling (not allocated) watchdog
    for _ in 0..10 {
        std::thread::sleep(VALIDATION_PERIOD);
        confirmed.owned.confirm();
    }
}

fn watchdog_validated_fn(
) -> impl Fn(usize, usize) -> Result<(), ZAllocError> + Clone + Send + Sync + 'static {
    |_task_index: usize, _iteration: usize| {
        let allocated = GLOBAL_METADATA_STORAGE.read().allocate()?;
        let confirmed = GLOBAL_CONFIRMATOR.read().add(allocated.clone());

        GLOBAL_VALIDATOR.read().add(allocated.clone());

        // check that the watchdog stays valid as it is confirmed
        for i in 0..10 {
            std::thread::sleep(VALIDATION_PERIOD);
            if allocated
                .header()
                .watchdog_invalidated
                .load(std::sync::atomic::Ordering::SeqCst)
            {
                error!("Invalid watchdog, iteration {i}");
                return Err(ZAllocError::Other);
            }
        }

        // Worst-case timings:
        // validation:       |___________|___________|___________|___________|
        // confirmation:    __|_____|_____|_____|_____|
        // drop(confirmed):                            ^
        // It means that the worst-case latency for the watchdog to become invalid is VALIDATION_PERIOD*2

        // check that the watchdog becomes invalid once we stop it's confirmation
        drop(confirmed);
        std::thread::sleep(VALIDATION_PERIOD * 3 + CONFIRMATION_PERIOD);
        assert!(allocated
            .header()
            .watchdog_invalidated
            .load(std::sync::atomic::Ordering::SeqCst));

        Ok(())
    }
}

#[test]
#[ignore]
fn watchdog_validated() {
    execute_concurrent(1, 10, watchdog_validated_fn());
}

#[test]
#[ignore]
fn watchdog_validated_concurrent() {
    execute_concurrent(1000, 10, watchdog_validated_fn());
}

fn watchdog_validated_invalid_without_confirmator_fn(
) -> impl Fn(usize, usize) -> ZResult<()> + Clone + Send + Sync + 'static {
    |_task_index: usize, _iteration: usize| -> ZResult<()> {
        let allocated = GLOBAL_METADATA_STORAGE
            .read()
            .allocate()
            .expect("error allocating watchdog!");

        assert!(allocated.test_validate() == 0);

        // add watchdog to validator
        GLOBAL_VALIDATOR.read().add(allocated.clone());

        // check that the watchdog becomes invalid because we do not confirm it
        std::thread::sleep(VALIDATION_PERIOD * 2 + CONFIRMATION_PERIOD);
        assert!(allocated
            .header()
            .watchdog_invalidated
            .load(std::sync::atomic::Ordering::SeqCst));

        Ok(())
    }
}

#[test]
#[ignore]
fn watchdog_validated_invalid_without_confirmator() {
    execute_concurrent(1, 10, watchdog_validated_invalid_without_confirmator_fn());
}

#[test]
#[ignore]
fn watchdog_validated_invalid_without_confirmator_concurrent() {
    execute_concurrent(
        1000,
        10,
        watchdog_validated_invalid_without_confirmator_fn(),
    );
}

fn watchdog_validated_additional_confirmation_fn(
) -> impl Fn(usize, usize) -> ZResult<()> + Clone + Send + Sync + 'static {
    |_task_index: usize, _iteration: usize| -> ZResult<()> {
        let allocated = GLOBAL_METADATA_STORAGE
            .read()
            .allocate()
            .expect("error allocating watchdog!");
        let confirmed = GLOBAL_CONFIRMATOR.read().add(allocated.clone());

        // add watchdog to validator
        GLOBAL_VALIDATOR.read().add(allocated.clone());

        // make additional confirmations
        for _ in 0..100 {
            std::thread::sleep(VALIDATION_PERIOD / 10);
            confirmed.owned.confirm();
        }

        // check that the watchdog stays valid as we stop additional confirmation
        std::thread::sleep(VALIDATION_PERIOD * 10);
        assert!(!allocated
            .header()
            .watchdog_invalidated
            .load(std::sync::atomic::Ordering::SeqCst));

        // Worst-case timings:
        // validation:       |___________|___________|___________|___________|
        // confirmation:    __|_____|_____|_____|_____|
        // drop(confirmed):                            ^
        // It means that the worst-case latency for the watchdog to become invalid is VALIDATION_PERIOD*2

        // check that the watchdog becomes invalid once we stop it's regular confirmation
        drop(confirmed);
        std::thread::sleep(VALIDATION_PERIOD * 2 + CONFIRMATION_PERIOD);
        // check that invalidation event happened!
        assert!(allocated
            .header()
            .watchdog_invalidated
            .load(std::sync::atomic::Ordering::SeqCst));

        Ok(())
    }
}

#[test]
#[ignore]
fn watchdog_validated_additional_confirmation() {
    execute_concurrent(1, 10, watchdog_validated_additional_confirmation_fn());
}

#[test]
#[ignore]
fn watchdog_validated_additional_confirmation_concurrent() {
    execute_concurrent(1000, 10, watchdog_validated_additional_confirmation_fn());
}

fn watchdog_validated_overloaded_system_fn(
) -> impl Fn(usize, usize) -> ZResult<()> + Clone + Send + Sync + 'static {
    |_task_index: usize, _iteration: usize| -> ZResult<()> {
        let allocated = GLOBAL_METADATA_STORAGE
            .read()
            .allocate()
            .expect("error allocating watchdog!");
        let confirmed = GLOBAL_CONFIRMATOR.read().add(allocated.clone());

        // add watchdog to validator
        GLOBAL_VALIDATOR.read().add(allocated.clone());

        // check that the watchdog stays valid
        std::thread::sleep(VALIDATION_PERIOD * 10);
        assert!(!allocated
            .header()
            .watchdog_invalidated
            .load(std::sync::atomic::Ordering::SeqCst));

        // Worst-case timings:
        // validation:       |___________|___________|___________|___________|
        // confirmation:    __|_____|_____|_____|_____|
        // drop(confirmed):                            ^
        // It means that the worst-case latency for the watchdog to become invalid is VALIDATION_PERIOD*2

        // check that the watchdog becomes invalid once we stop it's regular confirmation
        drop(confirmed);
        std::thread::sleep(VALIDATION_PERIOD * 2 + CONFIRMATION_PERIOD);
        // check that invalidation event happened!
        assert!(allocated
            .header()
            .watchdog_invalidated
            .load(std::sync::atomic::Ordering::SeqCst));

        Ok(())
    }
}

#[test]
#[ignore]
fn watchdog_validated_low_load() {
    let _load = CpuLoad::low();
    execute_concurrent(1000, 10, watchdog_validated_overloaded_system_fn());
}

#[test]
#[ignore]
fn watchdog_validated_high_load() {
    let _load = CpuLoad::optimal_high();
    execute_concurrent(1000, 10, watchdog_validated_overloaded_system_fn());
}

#[test]
#[ignore]
fn watchdog_validated_overloaded_system() {
    let _load = CpuLoad::excessive();
    execute_concurrent(1000, 10, watchdog_validated_overloaded_system_fn());
}
