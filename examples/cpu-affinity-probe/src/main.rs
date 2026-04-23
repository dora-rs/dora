//! Diagnostic node for regression-testing `cpu_affinity`.
//!
//! On Linux, queries its own scheduler affinity mask via `sched_getaffinity`
//! and prints it as a comma-separated list of core IDs prefixed with
//! `AFFINITY_MASK:`. The e2e test greps for that marker line and compares
//! the mask against the value configured in the fixture YAML.
//!
//! On non-Linux, prints `AFFINITY_MASK:unsupported` so the test skips
//! cleanly.
//!
//! After printing, the probe consumes tick events until the dora stream
//! closes. Exiting on the first tick used to race `dora run`'s log-subscribe
//! path (short-lived dataflows could be gone from the coordinator by the
//! time subscribe_logs arrives → "no running dataflow with id" at
//! binaries/cli/src/ws_client.rs:454). Staying alive for the duration of
//! `--stop-after` closes that race on the node side.

use dora_node_api::{DoraNode, Event};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    #[cfg(target_os = "linux")]
    {
        let mask = read_affinity_mask()?;
        let mut sorted = mask;
        sorted.sort_unstable();
        let csv = sorted
            .iter()
            .map(|c| c.to_string())
            .collect::<Vec<_>>()
            .join(",");
        println!("AFFINITY_MASK:{csv}");
    }

    #[cfg(not(target_os = "linux"))]
    {
        println!("AFFINITY_MASK:unsupported");
    }

    while let Some(event) = events.recv() {
        if matches!(event, Event::Stop(_)) {
            break;
        }
    }

    Ok(())
}

#[cfg(target_os = "linux")]
fn read_affinity_mask() -> eyre::Result<Vec<usize>> {
    // SAFETY: sched_getaffinity is thread-safe; we pass a correctly-sized
    // zero-initialized cpu_set_t and our own PID (0 means current thread).
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        let ret = libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut set);
        if ret != 0 {
            return Err(eyre::eyre!(
                "sched_getaffinity failed: {}",
                std::io::Error::last_os_error()
            ));
        }
        let max = std::mem::size_of::<libc::cpu_set_t>() * 8;
        let mut cores = Vec::new();
        for core in 0..max {
            if libc::CPU_ISSET(core, &set) {
                cores.push(core);
            }
        }
        Ok(cores)
    }
}
