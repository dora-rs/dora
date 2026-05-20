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
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};

use thread_priority::ThreadBuilder;
#[cfg(unix)]
use thread_priority::{
    set_current_thread_priority, RealtimeThreadSchedulePolicy, ThreadPriority, ThreadPriorityValue,
    ThreadSchedulePolicy::Realtime,
};

#[derive(PartialEq, Eq)]
pub enum TaskWakeReason {
    Timeout,
    Kick,
}

#[derive(Debug)]
pub struct PeriodicTask {
    running: Arc<AtomicBool>,
    send: std::sync::mpsc::Sender<()>,
}

impl Drop for PeriodicTask {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        self.kick();
    }
}

impl PeriodicTask {
    pub fn kicker(&self) -> std::sync::mpsc::Sender<()> {
        self.send.clone()
    }

    pub fn kick(&self) {
        let _ = self.send.send(());
    }

    pub fn new<F>(name: String, interval: Duration, mut f: F) -> Self
    where
        F: FnMut(TaskWakeReason) + Send + 'static,
    {
        let (send, recv) = std::sync::mpsc::channel::<()>();

        let running = Arc::new(AtomicBool::new(true));

        let c_running = running.clone();

        #[cfg(unix)]
        let builder = ThreadBuilder::default()
            .name(name)
            .policy(Realtime(RealtimeThreadSchedulePolicy::Fifo))
            .priority(ThreadPriority::Min);

        // TODO: deal with windows realtime scheduling
        #[cfg(windows)]
        let builder = ThreadBuilder::default().name(name);

        let _ = builder.spawn(move |result| {
            if let Err(e) = result {
                let mut err = format!(
                    "{:?}: error setting scheduling priority for thread: {:?}, will run with ",
                    std::thread::current().name(),
                    e
                );
                #[cfg(windows)]
                {
                    err.push_str("the default one. ");
                }
                #[cfg(unix)]
                {
                    for priority in (ThreadPriorityValue::MIN..ThreadPriorityValue::MAX).rev() {
                        if let Ok(p) = priority.try_into() {
                            if set_current_thread_priority(ThreadPriority::Crossplatform(p)).is_ok()
                            {
                                err.push_str(&format!("priority {priority}. "));
                                break;
                            }
                        }
                    }
                }
                err.push_str("This is not an hard error and it can be safely ignored under normal operating conditions. \
                Though the SHM subsystem may experience some timeouts in case of an heavy congested system where this watchdog thread may not be scheduled at the required frequency.");
                tracing::warn!("{}", err);
            }

            //TODO: need mlock here!

            while c_running.load(Ordering::Relaxed) {
                let cycle_start = std::time::Instant::now();

                f(TaskWakeReason::Timeout);

                // sleep for next iteration
                let elapsed = cycle_start.elapsed();
                if elapsed < interval {
                    let mut sleep_interval = interval - elapsed;
                    while recv.recv_timeout (sleep_interval).is_ok() {
                        f(TaskWakeReason::Kick);
                        let elapsed = cycle_start.elapsed();
                        if elapsed < interval {
                            sleep_interval = interval - elapsed;
                        } else {
                            break;
                        }
                    }
                } else {
                    let err = format!("{:?}: timer overrun", std::thread::current().name());
                    #[cfg(not(feature = "test"))]
                    tracing::error!("{err}");
                    #[cfg(feature = "test")]
                    panic!("{err}");
                }
            }
        });

        Self { running, send }
    }
}
