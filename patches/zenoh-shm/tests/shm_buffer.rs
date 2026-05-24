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

use std::mem::MaybeUninit;

use zenoh_core::Wait;
use zenoh_shm::api::{
    buffer::{
        traits::{ResideInShm, ShmBufUnsafeMut},
        typed::Typed,
        zshm::ZShm,
        zshmmut::ZShmMut,
    },
    provider::{memory_layout::TypedLayout, shm_provider::ShmProviderBuilder},
};

#[repr(C, align(1))]
#[stabby::stabby]
struct SharedByteData {
    data: [u8; 64],
}

fn make_shm_buffer() -> ZShmMut {
    let provider = ShmProviderBuilder::default_backend(65536).wait().unwrap();
    provider.alloc(64).wait().unwrap()
}

fn make_typed_shm_buffer<T: ResideInShm>() -> Typed<MaybeUninit<T>, ZShmMut> {
    let provider = ShmProviderBuilder::default_backend(65536).wait().unwrap();
    provider.alloc(TypedLayout::<T>::new()).wait().unwrap()
}

fn fill_and_check(buf: &mut [u8], val: u8) {
    buf.fill(val);
    for d in buf {
        assert!(*d == val)
    }
}

fn validate_shm_slice(buf: &mut [u8]) {
    for i in 0..10 {
        fill_and_check(buf, i);
    }
}

fn validate_raw_buffer_consistency(buffer: &mut ZShmMut) {
    validate_shm_slice(buffer.as_mut());
}

fn validate_typed_buffer_consistency(buffer: &mut impl AsMut<MaybeUninit<SharedByteData>>) {
    validate_shm_slice(&mut buffer.as_mut().write(SharedByteData { data: [0; 64] }).data);
}

fn validate_typed_to_raw_buffer_consistency(buffer: &mut Typed<MaybeUninit<SharedByteData>, ZShm>) {
    let mut raw = Typed::inner(buffer).clone();

    let raw_mut = unsafe { raw.as_mut_unchecked() };
    for i in 0..10 {
        raw_mut.fill(i);
        for val in &unsafe { buffer.assume_init_ref() }.data {
            assert!(*val == i)
        }
    }
}

#[test]
fn shm_buffer_alloc_typed() {
    let mut buffer = make_typed_shm_buffer::<SharedByteData>();
    validate_typed_buffer_consistency(&mut buffer);

    let mut buffer = Typed::into_inner(buffer);
    validate_raw_buffer_consistency(&mut buffer);

    let mut buffer = Typed::<MaybeUninit<SharedByteData>, _>::new(buffer).unwrap();
    validate_typed_buffer_consistency(&mut buffer);
}

#[test]
fn typed_to_raw() {
    let mut buffer = make_typed_shm_buffer::<SharedByteData>().into();
    validate_typed_to_raw_buffer_consistency(&mut buffer);
}

#[test]
fn shm_buffer_morph() {
    let mut buffer = make_shm_buffer();
    validate_raw_buffer_consistency(&mut buffer);

    let mut buffer = Typed::<MaybeUninit<SharedByteData>, _>::new(buffer).unwrap();
    validate_typed_buffer_consistency(&mut buffer);

    let mut buffer = Typed::into_inner(buffer);
    validate_raw_buffer_consistency(&mut buffer);
}
