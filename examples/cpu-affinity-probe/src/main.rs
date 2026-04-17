//! Diagnostic node for regression-testing `cpu_affinity`.
//!
//! On Linux, queries its own scheduler affinity mask via `sched_getaffinity`
//! and prints it as a comma-separated list of core IDs prefixed with
//! `AFFINITY_MASK:`. The e2e test greps for that marker line and compares
//! the mask against the value configured in the fixture YAML.
//!
//! On non-Linux, prints `AFFINITY_MASK:unsupported` so the test skips
//! cleanly.

use dora_node_api::DoraNode;
use eyre::Context;

fn main() -> eyre::Result<()> {
    // Initialize as a dora node so the daemon can manage the process
    // lifecycle. We don't consume events; the node prints its mask and
    // exits on the first tick (or immediately).
    let (_node, _events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;

    #[cfg(target_os = "linux")]
    {
        let mask = read_affinity_mask()?;
        // Sort for deterministic comparison.
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
