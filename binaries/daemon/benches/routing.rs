use adora_daemon::bench_support;
use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};

fn bench_routing(c: &mut Criterion) {
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();

    let mut group = c.benchmark_group("daemon_routing");

    for &fan_out in &[1, 4, 8] {
        for &payload_size in &[64, 4096, 65536] {
            group.bench_function(
                BenchmarkId::new(format!("fan{fan_out}"), payload_size),
                |b| {
                    let (mut df, clock, mut receivers) = bench_support::setup_routing(fan_out);
                    let fixture = bench_support::make_fixture(&clock, payload_size);

                    b.iter(|| {
                        rt.block_on(async {
                            bench_support::route_message(&mut df, &fixture, &clock).await;
                        });

                        // Drain receivers to prevent unbounded growth
                        for rx in &mut receivers {
                            while rx.try_recv().is_ok() {}
                        }

                        black_box(());
                    });
                },
            );
        }
    }
    group.finish();
}

criterion_group!(benches, bench_routing);
criterion_main!(benches);
