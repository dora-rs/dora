use std::time::Instant;

use eyre::Result;
use sysinfo::{CpuExt, NetworkExt, NetworksExt, System, SystemExt};

use crate::tui::app::SystemMetrics;

#[derive(Debug)]
pub struct MetricsCollector {
    system: System,
    last_io: Option<(u64, u64)>,
}

impl MetricsCollector {
    pub fn new() -> Self {
        let mut system = System::new_all();
        system.refresh_all();
        Self {
            system,
            last_io: None,
        }
    }

    pub fn collect(&mut self) -> Result<SystemMetrics> {
        self.system.refresh_cpu();
        self.system.refresh_memory();
        self.system.refresh_networks();

        let cpu_usage = self.system.global_cpu_info().cpu_usage();

        let total_memory = self.system.total_memory() as f32;
        let used_memory = self.system.used_memory() as f32;
        let memory_percent = if total_memory > 0.0 {
            (used_memory / total_memory) * 100.0
        } else {
            0.0
        };

        let networks = self.system.networks();
        let total_received: u64 = networks.iter().map(|(_, data)| data.received()).sum();
        let total_transmitted: u64 = networks.iter().map(|(_, data)| data.transmitted()).sum();

        let io_delta = match self.last_io {
            Some((prev_rx, prev_tx)) => (
                total_received.saturating_sub(prev_rx),
                total_transmitted.saturating_sub(prev_tx),
            ),
            None => (total_received, total_transmitted),
        };
        self.last_io = Some((total_received, total_transmitted));

        Ok(SystemMetrics {
            cpu_usage,
            memory_usage: memory_percent,
            network_io: io_delta,
            last_update: Some(Instant::now()),
        })
    }
}
