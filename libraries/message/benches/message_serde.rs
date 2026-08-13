use aligned_vec::AVec;
use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};

use dora_message::{
    common::DataMessage, id::DataId, metadata::Metadata, node_to_daemon::DaemonRequest,
};

fn make_metadata(_payload_size: usize) -> Metadata {
    let clock = uhlc::HLC::default();
    Metadata::new(clock.new_timestamp())
}

fn make_send_message(payload_size: usize) -> DaemonRequest {
    let data = vec![0u8; payload_size];
    DaemonRequest::SendMessage {
        output_id: DataId::from("output".to_string()),
        metadata: make_metadata(payload_size),
        data: Some(DataMessage::Vec(AVec::from_slice(128, &data))),
    }
}

fn bench_encode(c: &mut Criterion) {
    let mut group = c.benchmark_group("encode");
    for &size in &[64, 4096, 65536, 1_048_576] {
        let msg = make_send_message(size);
        // The production path (`daemon_connection::tcp`): pre-sized buffer.
        group.bench_with_input(BenchmarkId::new("presized", size), &msg, |b, msg| {
            b.iter(|| {
                let msg = black_box(msg);
                black_box(dora_message::encode_presized(msg, msg.encode_size_hint()).unwrap());
            });
        });
        // The same encoding into a buffer that grows from empty. Keeps the
        // pre-sizing win measurable in-tree: if this arm stops being slower,
        // `encode_presized` has stopped earning its complexity.
        group.bench_with_input(BenchmarkId::new("grow_from_empty", size), &msg, |b, msg| {
            b.iter(|| black_box(postcard::to_stdvec(black_box(msg)).unwrap()));
        });
    }
    group.finish();
}

fn bench_decode(c: &mut Criterion) {
    let mut group = c.benchmark_group("decode");
    for &size in &[64, 4096, 65536, 1_048_576] {
        let bytes = dora_message::encode(&make_send_message(size)).unwrap();
        group.bench_with_input(BenchmarkId::new("SendMessage", size), &bytes, |b, bytes| {
            b.iter(|| {
                let msg: DaemonRequest = dora_message::decode(black_box(bytes)).unwrap();
                black_box(msg);
            });
        });
    }
    group.finish();
}

fn bench_metadata_clone(c: &mut Criterion) {
    let mut group = c.benchmark_group("metadata_clone");
    for &size in &[64, 4096, 65536] {
        let metadata = make_metadata(size);
        group.bench_with_input(BenchmarkId::new("Metadata", size), &metadata, |b, m| {
            b.iter(|| {
                let cloned = black_box(m).clone();
                black_box(cloned);
            });
        });
    }
    group.finish();
}

criterion_group!(benches, bench_encode, bench_decode, bench_metadata_clone,);
criterion_main!(benches);
