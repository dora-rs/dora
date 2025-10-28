use std::time::Instant;

use eyre::Result;
use sysinfo::{CpuExt, NetworkExt, NetworksExt, System, SystemExt};

use crate::tui::app::SystemMetrics;

pub(crate) fn gather_system_metrics() -> Result<SystemMetrics> {
    let mut system = System::new_all();
    system.refresh_cpu();
    system.refresh_memory();
    system.refresh_networks();

    let cpu_usage = system.global_cpu_info().cpu_usage();

    let total_memory = system.total_memory() as f32;
    let used_memory = system.used_memory() as f32;
    let memory_percent = if total_memory > 0.0 {
        (used_memory / total_memory) * 100.0
    } else {
        0.0
    };

    let networks = system.networks();
    let total_received: u64 = networks.iter().map(|(_, data)| data.total_received()).sum();
    let total_transmitted: u64 = networks
        .iter()
        .map(|(_, data)| data.total_transmitted())
        .sum();

    Ok(SystemMetrics {
        cpu_usage,
        memory_usage: memory_percent,
        network_io: (total_received, total_transmitted),
        last_update: Some(Instant::now()),
    })
}
