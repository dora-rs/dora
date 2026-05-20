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

use zenoh_core::Wait;
use zenoh_shm::api::{
    client::shm_client::ShmClient,
    protocol_implementations::posix::{
        posix_shm_client::PosixShmClient,
        posix_shm_provider_backend_binary_heap::PosixShmProviderBackendBinaryHeap,
        posix_shm_provider_backend_buddy::PosixShmProviderBackendBuddy,
        posix_shm_provider_backend_talc::PosixShmProviderBackendTalc,
    },
    provider::{
        memory_layout::MemoryLayout, shm_provider_backend::ShmProviderBackend,
        types::AllocAlignment,
    },
};

static BUFFER_NUM: usize = 100;
static BUFFER_SIZE: usize = 1000;

#[test]
fn posix_shm_provider_create() {
    let size = 1024;
    let backend = PosixShmProviderBackendBinaryHeap::builder(size)
        .wait()
        .expect("Error creating PosixShmProviderBackend!");
    assert!(backend.available() >= size);
}

#[test]
fn posix_shm_provider_alloc() {
    let backend = PosixShmProviderBackendBinaryHeap::builder(1024)
        .wait()
        .expect("Error creating PosixShmProviderBackend!");

    let layout = MemoryLayout::new(100, AllocAlignment::default()).unwrap();

    let _buf = backend
        .alloc(&layout)
        .expect("PosixShmProviderBackend: error allocating buffer");
}

#[test]
fn posix_shm_provider_open() {
    let backend = PosixShmProviderBackendBinaryHeap::builder(1024)
        .wait()
        .expect("Error creating PosixShmProviderBackend!");

    let layout = MemoryLayout::new(100, AllocAlignment::default()).unwrap();

    let buf = backend
        .alloc(&layout)
        .expect("PosixShmProviderBackend: error allocating buffer");

    let client = PosixShmClient {};

    let _segment = client
        .attach(buf.descriptor.segment)
        .expect("Error attaching to segment");
}

#[test]
fn posix_shm_provider_binary_heap_allocator() {
    // size to allocate in the provider
    let size_to_alloc = BUFFER_SIZE * BUFFER_NUM;

    let backend = PosixShmProviderBackendBinaryHeap::builder(size_to_alloc)
        .wait()
        .expect("Error creating PosixShmProviderBackend!");

    // the real size of memory available in the provider
    let real_size = backend.available();
    assert!(real_size >= size_to_alloc);

    // the real number of buffers allocatable in the provider
    let real_num = real_size / BUFFER_SIZE;
    assert!(real_num >= BUFFER_NUM);

    // the remainder in the provider
    let remainder = real_size - real_num * BUFFER_SIZE;
    assert!(remainder < BUFFER_SIZE);

    let layout = MemoryLayout::new(BUFFER_SIZE, AllocAlignment::default()).unwrap();

    // exhaust memory by allocating it all
    let mut buffers = vec![];
    for _ in 0..real_num {
        let buf = backend
            .alloc(&layout)
            .expect("PosixShmProviderBackend: error allocating buffer");
        buffers.push(buf);
    }

    for _ in 0..real_num {
        // there is nothing to allocate at this point
        assert_eq!(backend.available(), remainder);
        assert!(backend.alloc(&layout).is_err());

        // free buffer
        let to_free = buffers.pop().unwrap().descriptor;
        backend.free(&to_free);

        // allocate new one
        let buf = backend
            .alloc(&layout)
            .expect("PosixShmProviderBackend: error allocating buffer");
        buffers.push(buf);
    }

    // free buffers
    while let Some(buffer) = buffers.pop() {
        backend.free(&buffer.descriptor);
    }

    // confirm that allocator is free
    assert_eq!(backend.available(), real_size);
}

#[test]
fn posix_shm_provider_buddy_allocator() {
    // size to allocate in the provider
    let size_to_alloc = BUFFER_SIZE * BUFFER_NUM;

    let backend = PosixShmProviderBackendBuddy::builder(size_to_alloc)
        .wait()
        .expect("Error creating PosixShmProviderBackend!");

    let layout = MemoryLayout::new(BUFFER_SIZE, AllocAlignment::default()).unwrap();

    // exhaust memory by allocating it all
    let mut buffers = vec![];
    while let Ok(buf) = backend.alloc(&layout) {
        buffers.push(buf);
    }

    for _ in 0..100 {
        // there is nothing to allocate at this point
        assert!(backend.alloc(&layout).is_err());

        // free buffer
        let to_free = buffers.pop().unwrap().descriptor;
        backend.free(&to_free);

        // allocate new one
        let buf = backend
            .alloc(&layout)
            .expect("PosixShmProviderBackend: error allocating buffer");
        buffers.push(buf);
    }

    // free buffers
    while let Some(buffer) = buffers.pop() {
        backend.free(&buffer.descriptor);
    }
}

#[test]
fn posix_shm_provider_talc_allocator() {
    // size to allocate in the provider
    let size_to_alloc = BUFFER_SIZE * BUFFER_NUM;

    let backend = PosixShmProviderBackendTalc::builder(size_to_alloc)
        .wait()
        .expect("Error creating PosixShmProviderBackend!");

    let layout = MemoryLayout::new(BUFFER_SIZE, AllocAlignment::default()).unwrap();

    // exhaust memory by allocating it all
    let mut buffers = vec![];
    while let Ok(buf) = backend.alloc(&layout) {
        buffers.push(buf);
    }

    for _ in 0..100 {
        // there is nothing to allocate at this point
        assert!(backend.alloc(&layout).is_err());

        // free buffer
        let to_free = buffers.pop().unwrap().descriptor;
        backend.free(&to_free);

        // allocate new one
        let buf = backend
            .alloc(&layout)
            .expect("PosixShmProviderBackend: error allocating buffer");
        buffers.push(buf);
    }

    // free buffers
    while let Some(buffer) = buffers.pop() {
        backend.free(&buffer.descriptor);
    }
}
