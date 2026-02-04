use std::{
    process::Command,
    time::{Duration, Instant},
};

use eyre::{Context, ContextCompat, eyre};
use shared_memory_server::{ShmemChannel, ShmemConf};

fn main() -> eyre::Result<()> {
    let mut args = std::env::args();
    let executable = args.next().wrap_err("no arg 0")?;
    let arg = args.next();

    match arg.as_deref() {
        Some("client") => client(args.next().wrap_err("no shmem id")?)?,
        None => server(executable)?,
        Some(other) => eyre::bail!("unexpected argument `{other}`"),
    }

    Ok(())
}

fn server(executable: String) -> eyre::Result<()> {
    let shmem = ShmemConf::new()
        .size(4096)
        .create()
        .wrap_err("failed to create shmem region")?;
    let shmem_id = shmem.get_os_id().to_owned();
    let mut server =
        unsafe { ShmemChannel::new_server(shmem) }.wrap_err("failed to create ShmemServer")?;

    let mut client = Command::new(executable);
    client.arg("client").arg(shmem_id);
    let mut client_handle = client.spawn().wrap_err("failed to spawn client process")?;

    server_loop(&mut server).wrap_err("server loop failed")?;

    let status = client_handle
        .wait()
        .wrap_err("failed to wait for client process")?;

    if status.success() {
        Ok(())
    } else {
        Err(eyre!("client failed"))
    }
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
enum Request {
    Ping,
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
enum Reply {
    Pong,
}

fn server_loop(server: &mut ShmemChannel) -> eyre::Result<()> {
    while let Some(request) = server
        .receive(None)
        .wrap_err("failed to receive next message")?
    {
        let request = bincode::deserialize::<Request>(&request)?;
        match request {
            Request::Ping => server
                .send(&bincode::serialize(&Reply::Pong)?)
                .wrap_err("failed to send reply")?,
        }
    }
    Ok(())
}

fn client(shmem_id: String) -> eyre::Result<()> {
    let shmem = ShmemConf::new()
        .os_id(shmem_id)
        .open()
        .wrap_err("failed to open shmem region")?;
    let mut client =
        unsafe { ShmemChannel::new_client(shmem) }.wrap_err("failed to create ShmemClient")?;

    client_loop(&mut client).wrap_err("client loop failed")?;

    Ok(())
}

fn client_loop(client: &mut ShmemChannel) -> eyre::Result<()> {
    let mut latencies = Vec::new();
    for _ in 0..10_000_000 {
        let start = Instant::now();
        client
            .send(&bincode::serialize(&Request::Ping)?)
            .wrap_err("ping failed")?;
        let reply = client.receive(Some(Duration::from_secs(2)))?.unwrap();
        match bincode::deserialize::<Reply>(&reply)? {
            Reply::Pong => {
                latencies.push(start.elapsed());
            }
        }
    }

    let n = latencies.len();
    let avg_latency = latencies.iter().copied().sum::<Duration>() / n as u32;
    let min_latency = latencies.iter().min().unwrap();
    let max_latency = latencies.iter().max().unwrap();
    println!("average latency: {avg_latency:?} (min: {min_latency:?}, max: {max_latency:?})");

    let mut longest: Vec<_> = latencies.iter().enumerate().map(|(i, d)| (d, i)).collect();
    longest.sort_unstable_by(|a, b| b.cmp(a));

    println!("\nlongest iterations:");
    for (duration, index) in &longest[..10] {
        println!("  {index}: {duration:?}")
    }

    Ok(())
}
