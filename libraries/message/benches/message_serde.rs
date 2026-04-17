use aligned_vec::AVec;
use arrow_schema::DataType;
use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};

use dora_message::{
    common::DataMessage,
    id::DataId,
    metadata::{ArrowTypeInfo, Metadata},
    node_to_daemon::DaemonRequest,
};

fn make_metadata(payload_size: usize) -> Metadata {
    let clock = uhlc::HLC::default();
    let type_info = ArrowTypeInfo {
        data_type: DataType::UInt8,
        len: payload_size,
        null_count: 0,
        validity: None,
        offset: 0,
        buffer_offsets: vec![],
        child_data: vec![],
        field_names: None,
        schema_hash: None,
    };
    Metadata::new(clock.new_timestamp(), type_info)
}

fn make_send_message(payload_size: usize) -> DaemonRequest {
    let data = vec![0u8; payload_size];
    DaemonRequest::SendMessage {
        output_id: DataId::from("output".to_string()),
        metadata: make_metadata(payload_size),
        data: Some(DataMessage::Vec(AVec::from_slice(128, &data))),
    }
}

fn bench_bincode_serialize(c: &mut Criterion) {
    let mut group = c.benchmark_group("bincode_serialize");
    for &size in &[64, 4096, 65536, 1_048_576] {
        let msg = make_send_message(size);
        group.bench_with_input(BenchmarkId::new("SendMessage", size), &msg, |b, msg| {
            b.iter(|| {
                let bytes = bincode::serialize(black_box(msg)).unwrap();
                black_box(bytes);
            });
        });
    }
    group.finish();
}

fn bench_bincode_deserialize(c: &mut Criterion) {
    let mut group = c.benchmark_group("bincode_deserialize");
    for &size in &[64, 4096, 65536, 1_048_576] {
        let msg = make_send_message(size);
        let bytes = bincode::serialize(&msg).unwrap();
        group.bench_with_input(BenchmarkId::new("SendMessage", size), &bytes, |b, bytes| {
            b.iter(|| {
                let msg: DaemonRequest = bincode::deserialize(black_box(bytes)).unwrap();
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

criterion_group!(
    benches,
    bench_bincode_serialize,
    bench_bincode_deserialize,
    bench_metadata_clone,
);
criterion_main!(benches);
