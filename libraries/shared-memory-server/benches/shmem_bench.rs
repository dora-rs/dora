use criterion::{Criterion, black_box, criterion_group, criterion_main};
use shared_memory_server::{ShmemClient, ShmemConf, ShmemServer};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

const SHMEM_SIZE: usize = 64 * 1024; // 64 KB

fn bench_round_trip(c: &mut Criterion) {
    let mut group = c.benchmark_group("shmem_round_trip");

    for payload_size in [64, 1024, 4096, 16384] {
        group.bench_function(format!("{payload_size}B"), |b| {
            let server_mem = ShmemConf::new().size(SHMEM_SIZE).create().unwrap();
            let os_id = server_mem.get_os_id().to_string();

            let mut server: ShmemServer<Vec<u8>, Vec<u8>> =
                unsafe { ShmemServer::new(server_mem) }.unwrap();

            // Use channels to drive client from the main thread
            let (req_tx, req_rx) = mpsc::channel::<Vec<u8>>();
            let (reply_tx, reply_rx) = mpsc::channel::<Vec<u8>>();

            // Client opens shmem inside the thread (Shmem is !Send)
            let client_os_id = os_id.clone();
            let handle = thread::spawn(move || {
                let client_mem = ShmemConf::new().os_id(&client_os_id).open().unwrap();
                let mut client: ShmemClient<Vec<u8>, Vec<u8>> =
                    unsafe { ShmemClient::new(client_mem, Some(Duration::from_secs(5))) }.unwrap();

                while let Ok(payload) = req_rx.recv() {
                    let reply = client.request(&payload).unwrap();
                    reply_tx.send(reply).unwrap();
                }
            });

            let payload = vec![42u8; payload_size];
            let reply_payload = vec![0u8; payload_size];

            // Warm up: one round-trip so client thread is waiting
            req_tx.send(payload.clone()).unwrap();
            // Server must enter listen() before client sends (edge-triggered events)
            let _msg = server.listen().unwrap().unwrap();
            server.send_reply(&reply_payload).unwrap();
            let _warmup = reply_rx.recv().unwrap();

            b.iter(|| {
                req_tx.send(payload.clone()).unwrap();
                let msg = server.listen().unwrap().unwrap();
                black_box(&msg);
                server.send_reply(black_box(&reply_payload)).unwrap();
                let reply = reply_rx.recv().unwrap();
                black_box(&reply);
            });

            drop(req_tx);
            handle.join().unwrap();
        });
    }
    group.finish();
}

criterion_group!(benches, bench_round_trip);
criterion_main!(benches);
