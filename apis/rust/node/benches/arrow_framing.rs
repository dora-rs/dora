//! Encode/decode benchmarks for Dora's Arrow framing.
//!
//! Encode strategies (all write into a **pre-allocated, pre-faulted, reused**
//! destination, so first-touch page faults don't confound the measurement):
//!   * `raw`          — `copy_array_into_sample` (custom raw buffer layout)
//!   * `ipc_fast`     — `ipc_encode::encode_ipc_into` (1-copy IPC fast path)
//!   * `ipc_official` — `encode_arrow_ipc` (`StreamWriter`, stages a body `Vec`)
//!
//! Decode strategies:
//!   * `raw`                        — `buffer_into_arrow_array`
//!   * `ipc_streamreader_copy`      — `decode_arrow_ipc` (`StreamReader`, copies)
//!   * `ipc_streamdecoder_zerocopy` — `decode_arrow_ipc_zero_copy` (aliases input)
//!
//! Run with: `cargo bench -p dora-node-api --bench arrow_framing`

use std::ptr::NonNull;
use std::sync::Arc;

use aligned_vec::{AVec, ConstAlign};
use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use dora_node_api::arrow::array::{Array, ArrayData, Float32Array};
use dora_node_api::arrow::buffer::Buffer;
use dora_node_api::arrow_utils::ipc_encode::{encode_ipc_into, ipc_fast_path_len};
use dora_node_api::arrow_utils::{
    buffer_into_arrow_array, copy_array_into_sample, decode_arrow_ipc, decode_arrow_ipc_zero_copy,
    encode_arrow_ipc, required_data_size,
};

/// Copy `bytes` into a 128-byte-aligned Arrow buffer, mirroring how Dora's
/// receive path backs payloads (`AVec<u8, ConstAlign<128>>` / page-aligned
/// Zenoh SHM). This is the precondition under which the IPC `StreamDecoder`
/// path aliases the input instead of realigning it.
fn aligned_buffer_from(bytes: &[u8]) -> Buffer {
    let mut aligned: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, bytes.len());
    aligned.copy_from_slice(bytes);
    let ptr = NonNull::new(aligned.as_ptr() as *mut u8).unwrap();
    let len = aligned.len();
    // SAFETY: ptr/len describe `aligned`'s allocation; the Arc keeps it alive.
    unsafe { Buffer::from_custom_allocation(ptr, len, Arc::new(aligned)) }
}

fn make_array(num_elements: usize) -> ArrayData {
    let v: Vec<f32> = (0..num_elements).map(|i| i as f32).collect();
    Float32Array::from(v).into_data()
}

/// Allocate a zeroed buffer and touch every page so reuse in `b.iter` is warm.
fn prefaulted(len: usize) -> Vec<u8> {
    let mut buf = vec![0u8; len];
    buf.iter_mut().for_each(|b| *b = 0);
    buf
}

fn bench_framing(c: &mut Criterion) {
    // f32 element counts -> payloads of ~1 KiB, ~64 KiB, ~4 MiB.
    let element_counts = [256usize, 16_384, 1_048_576];

    for &n in &element_counts {
        let data = make_array(n);
        let payload_bytes = (n * std::mem::size_of::<f32>()) as u64;

        // -------- encode (reused, pre-faulted destinations) --------
        let mut enc = c.benchmark_group("encode");
        enc.throughput(Throughput::Bytes(payload_bytes));
        enc.bench_with_input(BenchmarkId::new("raw", payload_bytes), &data, |b, data| {
            let mut dst = prefaulted(required_data_size(data));
            b.iter(|| {
                let info = copy_array_into_sample(&mut dst, data);
                black_box(&info);
            })
        });
        enc.bench_with_input(
            BenchmarkId::new("ipc_fast", payload_bytes),
            &data,
            |b, data| {
                let mut dst = prefaulted(ipc_fast_path_len(data).expect("fast-path eligible"));
                b.iter(|| {
                    encode_ipc_into(data, &mut dst).unwrap();
                    black_box(dst[0]);
                })
            },
        );
        enc.bench_with_input(
            BenchmarkId::new("ipc_official", payload_bytes),
            &data,
            |b, data| b.iter(|| black_box(encode_arrow_ipc(data).unwrap())),
        );
        enc.finish();

        // -------- decode (pre-encode the inputs, reuse aligned buffers) --------
        let total = required_data_size(&data);
        let mut sample = vec![0u8; total];
        let type_info = copy_array_into_sample(&mut sample, &data);
        let raw_buf = aligned_buffer_from(&sample);

        let ipc_bytes = encode_arrow_ipc(&data).unwrap();
        let ipc_aligned = aligned_buffer_from(&ipc_bytes);

        let mut dec = c.benchmark_group("decode");
        dec.throughput(Throughput::Bytes(payload_bytes));
        dec.bench_with_input(BenchmarkId::new("raw", payload_bytes), &(), |b, _| {
            b.iter(|| black_box(buffer_into_arrow_array(&raw_buf, &type_info).unwrap()))
        });
        dec.bench_with_input(
            BenchmarkId::new("ipc_streamreader_copy", payload_bytes),
            &(),
            |b, _| b.iter(|| black_box(decode_arrow_ipc(&ipc_bytes).unwrap())),
        );
        dec.bench_with_input(
            BenchmarkId::new("ipc_streamdecoder_zerocopy", payload_bytes),
            &(),
            |b, _| b.iter(|| black_box(decode_arrow_ipc_zero_copy(ipc_aligned.clone()).unwrap())),
        );
        dec.finish();
    }
}

criterion_group!(benches, bench_framing);
criterion_main!(benches);
