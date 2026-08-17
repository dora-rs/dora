//! Encode/decode benchmarks for Dora's Arrow IPC framing.
//!
//! Encode strategies (all write into a **pre-allocated, pre-faulted, reused**
//! destination, so first-touch page faults don't confound the measurement):
//!   * `ipc_fast`     — `ipc_encode::encode_ipc_into` (1-copy IPC fast path)
//!   * `ipc_official` — `encode_arrow_ipc` (`StreamWriter`, stages a body `Vec`)
//!
//! Decode strategies:
//!   * `ipc_streamreader_copy`      — `decode_arrow_ipc` (`StreamReader`, copies)
//!   * `ipc_streamdecoder_zerocopy` — `decode_arrow_ipc_zero_copy` (aliases input)
//!
//! Run with: `cargo bench -p dora-node-api --bench arrow_framing`

use aligned_vec::{AVec, ConstAlign};
use arrow::array::Float32Array;
use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use dora_node_api::DoraArray;
use dora_node_api::arrow_utils::ipc_encode::{encode_ipc_into, ipc_fast_path_len};
use dora_node_api::arrow_utils::{
    IpcPayload, decode_arrow_ipc, decode_arrow_ipc_zero_copy, encode_arrow_ipc,
};

/// Copy `bytes` into a 128-byte-aligned Arrow buffer, mirroring how Dora's
/// receive path backs payloads (`AVec<u8, ConstAlign<128>>` / page-aligned
/// Zenoh SHM). This is the precondition under which the IPC `StreamDecoder`
/// path aliases the input instead of realigning it.
fn aligned_buffer_from(bytes: &[u8]) -> IpcPayload {
    let mut aligned: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, bytes.len());
    aligned.copy_from_slice(bytes);
    IpcPayload::from_aligned_vec(aligned)
}

fn make_array(num_elements: usize) -> DoraArray {
    let v: Vec<f32> = (0..num_elements).map(|i| i as f32).collect();
    DoraArray::from_array(Float32Array::from(v))
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
        let ipc_bytes = encode_arrow_ipc(&data).unwrap();
        let ipc_aligned = aligned_buffer_from(&ipc_bytes);

        let mut dec = c.benchmark_group("decode");
        dec.throughput(Throughput::Bytes(payload_bytes));
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
