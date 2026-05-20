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
    fmt::Debug,
    sync::{atomic::AtomicBool, Arc},
    thread::JoinHandle,
};

use zenoh_result::ZResult;

pub fn validate_memory(mem1: &mut [u8], mem2: &[u8]) {
    assert!(mem1.len() == mem2.len());
    for cycle in 0..255u8 {
        // sequentially fill segment1 with values checking segment2 having these changes
        for i in 0..mem1.len() {
            mem1[i] = cycle;
            assert!(mem2[i] == cycle);
        }

        // check the whole segment2 having proper values
        for i in mem2 {
            assert!(*i == cycle);
        }
    }
}

pub fn execute_concurrent<TaskFun, Terror>(
    concurrent_tasks: usize,
    iterations: usize,
    task_fun: TaskFun,
) where
    TaskFun: Fn(usize, usize) -> Result<(), Terror> + Clone + Send + Sync + 'static,
    Terror: Debug,
{
    let mut tasks = vec![];
    for task_index in 0..concurrent_tasks {
        let c_task_fun = task_fun.clone();
        let task_handle = std::thread::spawn(move || {
            for iteration in 0..iterations {
                if let Err(e) = c_task_fun(task_index, iteration) {
                    panic!("task {task_index}: iteration {iteration}: {e:?}")
                }
            }
        });
        tasks.push(task_handle);
    }
    for task in tasks {
        task.join().expect("Error joining thread!");
    }
}

pub fn load_fn(
    working: Arc<AtomicBool>,
) -> impl Fn(usize, usize) -> ZResult<()> + Clone + Send + Sync + 'static {
    move |_task_index: usize, _iteration: usize| -> ZResult<()> {
        while working.load(std::sync::atomic::Ordering::SeqCst) {}
        Ok(())
    }
}

pub struct CpuLoad {
    handle: Option<JoinHandle<()>>,
    flag: Arc<AtomicBool>,
}

impl Drop for CpuLoad {
    fn drop(&mut self) {
        self.flag.store(false, std::sync::atomic::Ordering::SeqCst);
        let _ = self.handle.take().unwrap().join();
    }
}

impl CpuLoad {
    pub fn excessive() -> Self {
        Self::new(1000)
    }

    #[cfg(feature = "test")]
    pub fn optimal_high() -> Self {
        Self::new(num_cpus::get())
    }

    pub fn low() -> Self {
        Self::new(1)
    }

    fn new(thread_count: usize) -> Self {
        let flag = Arc::new(AtomicBool::new(true));

        let c_flag = flag.clone();
        let handle = Some(std::thread::spawn(move || {
            execute_concurrent(thread_count, 1, load_fn(c_flag));
        }));

        Self { handle, flag }
    }
}
