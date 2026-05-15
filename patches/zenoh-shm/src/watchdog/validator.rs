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

use std::{
    collections::BTreeSet,
    sync::{atomic::AtomicI32, Arc},
    time::Duration,
};

use static_init::dynamic;

use super::periodic_task::PeriodicTask;
use crate::{
    metadata::descriptor::OwnedMetadataDescriptor, watchdog::periodic_task::TaskWakeReason,
};

#[dynamic(lazy, drop)]
pub static mut GLOBAL_VALIDATOR: WatchdogValidator =
    WatchdogValidator::new(Duration::from_millis(100));

enum Transaction {
    Add(OwnedMetadataDescriptor),
    Remove(OwnedMetadataDescriptor),
}

// TODO: optimize validation by packing descriptors
pub struct WatchdogValidator {
    cap: Arc<AtomicI32>,
    sender: crossbeam_channel::Sender<Transaction>,
    task: PeriodicTask,
}

impl WatchdogValidator {
    pub fn new(interval: Duration) -> Self {
        // Our channel here may change it's size (that size that is not capacity) from 0 to
        // some large number very frequently.
        // In order to reduce the number of allocations\frees we use bounded channel as this is
        // much faster in this scenario (profiled).
        // I believe it is much better to use growing channel that will never (or rarely) shrink,
        // but I didn't find good one.
        // Validator wakes every 100 milliseconds by default - and on each wakeup it empties the channel.
        // I assume that our max SHM INCOMING messaging rate may be around 5M msgs*sec, so it might
        // give 500K of msgs per validator wakeup. In order not to store 500K of cells in bounded channel
        // (that will break all the assumptions if we suddenly jump above 500K msgs per 100 milliseconds)
        // I select some reasonable and reliable enough capacity - and once our channel will reach half of it's
        // capacity, I trigger additional validator task wakeup - just to collect the channel contents.
        let channel_size = 65536 * 2;
        let (sender, receiver) = crossbeam_channel::bounded::<Transaction>(channel_size);

        let cap = Arc::new(AtomicI32::new((channel_size / 2) as i32));

        // See ordering implementation for OwnedMetadataDescriptor
        #[allow(clippy::mutable_key_type)]
        let mut watchdogs = BTreeSet::default();
        let c_cap = cap.clone();
        let task = PeriodicTask::new(
            "Watchdog Validator".to_owned(),
            interval,
            move |wake_reason| {
                // See ordering implementation for OwnedMetadataDescriptor
                #[allow(clippy::mutable_key_type)]
                fn collect_transactions(
                    receiver: &crossbeam_channel::Receiver<Transaction>,
                    storage: &mut BTreeSet<OwnedMetadataDescriptor>,
                    cap: &Arc<AtomicI32>,
                ) {
                    while let Ok(transaction) = receiver.try_recv() {
                        cap.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        match transaction {
                            Transaction::Add(descriptor) => {
                                let _old = storage.insert(descriptor);
                                #[cfg(feature = "test")]
                                assert!(_old);
                            }
                            Transaction::Remove(descriptor) => {
                                let _ = storage.remove(&descriptor);
                            }
                        }
                    }
                }

                collect_transactions(&receiver, &mut watchdogs, &c_cap);

                if wake_reason == TaskWakeReason::Timeout {
                    watchdogs.retain(|watchdog| {
                        let old_val = watchdog.validate();
                        if old_val == 0 {
                            watchdog
                                .header()
                                .watchdog_invalidated
                                .store(true, std::sync::atomic::Ordering::Relaxed);
                            return false;
                        }
                        true
                    });
                }
            },
        );

        Self { cap, sender, task }
    }

    pub fn add(&self, watchdog: OwnedMetadataDescriptor) {
        self.make_transaction(Transaction::Add(watchdog));
    }

    pub fn remove(&self, watchdog: OwnedMetadataDescriptor) {
        self.make_transaction(Transaction::Remove(watchdog));
    }

    fn make_transaction(&self, transaction: Transaction) {
        if self.cap.fetch_sub(1, std::sync::atomic::Ordering::Relaxed) == 0 {
            self.task.kick();
        }

        self.sender.send(transaction).unwrap();
    }
}
