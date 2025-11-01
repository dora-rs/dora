use std::time::{Duration, Instant};

use eyre::Result;
use sysinfo::{CpuExt, DiskExt, NetworkExt, NetworksExt, System, SystemExt};

use crate::tui::app::{DiskMetrics, LoadAverages, MemoryMetrics, NetworkMetrics, SystemMetrics};

#[derive(Debug)]
pub struct MetricsCollector {
    system: System,
    last_sample: Option<NetworkSample>,
}

#[derive(Debug, Clone, Copy)]
struct NetworkSample {
    timestamp: Instant,
    received: u64,
    transmitted: u64,
}

impl Default for MetricsCollector {
    fn default() -> Self {
        Self::new()
    }
}

impl MetricsCollector {
    pub fn new() -> Self {
        let mut system = System::new_all();
        system.refresh_all();
        Self {
            system,
            last_sample: None,
        }
    }

    pub fn collect(&mut self) -> Result<SystemMetrics> {
        self.system.refresh_cpu();
        self.system.refresh_memory();
        self.system.refresh_networks();
        self.system.refresh_disks_list();
        self.system.refresh_disks();

        let now = Instant::now();
        let cpu_usage = self.system.global_cpu_info().cpu_usage();

        let total_memory_kb = self.system.total_memory();
        let used_memory_kb = self.system.used_memory();
        let free_memory_kb = total_memory_kb.saturating_sub(used_memory_kb);
        let memory_percent = if total_memory_kb > 0 {
            (used_memory_kb as f32 / total_memory_kb as f32) * 100.0
        } else {
            0.0
        };

        let swap_total_kb = self.system.total_swap();
        let swap_used_kb = self.system.used_swap();
        let swap_usage_percent = if swap_total_kb > 0 {
            (swap_used_kb as f32 / swap_total_kb as f32) * 100.0
        } else {
            0.0
        };

        let memory = MemoryMetrics {
            total_bytes: kb_to_bytes(total_memory_kb),
            used_bytes: kb_to_bytes(used_memory_kb),
            free_bytes: kb_to_bytes(free_memory_kb),
            usage_percent: memory_percent,
            swap_total_bytes: kb_to_bytes(swap_total_kb),
            swap_used_bytes: kb_to_bytes(swap_used_kb),
            swap_usage_percent,
        };

        let (disk_total_bytes, disk_used_bytes) =
            self.system
                .disks()
                .iter()
                .fold((0_u64, 0_u64), |(total_acc, used_acc), disk| {
                    let total = disk.total_space();
                    let available = disk.available_space();
                    let used = total.saturating_sub(available);
                    (
                        total_acc.saturating_add(total),
                        used_acc.saturating_add(used),
                    )
                });

        let disk_usage_percent = if disk_total_bytes > 0 {
            (disk_used_bytes as f64 / disk_total_bytes as f64 * 100.0) as f32
        } else {
            0.0
        };

        let disk = DiskMetrics {
            total_bytes: disk_total_bytes,
            used_bytes: disk_used_bytes,
            usage_percent: disk_usage_percent,
        };

        let networks = self.system.networks();
        let total_received: u64 = networks.iter().map(|(_, data)| data.received()).sum();
        let total_transmitted: u64 = networks.iter().map(|(_, data)| data.transmitted()).sum();

        let mut rx_delta = 0_u64;
        let mut tx_delta = 0_u64;
        let (received_per_second, transmitted_per_second) = if let Some(sample) = self.last_sample {
            let elapsed = now.saturating_duration_since(sample.timestamp);
            if elapsed > Duration::from_secs(0) {
                rx_delta = total_received.saturating_sub(sample.received);
                tx_delta = total_transmitted.saturating_sub(sample.transmitted);
                let seconds = elapsed.as_secs_f64();
                (
                    rx_delta as f64 / seconds.max(f64::MIN_POSITIVE),
                    tx_delta as f64 / seconds.max(f64::MIN_POSITIVE),
                )
            } else {
                (0.0, 0.0)
            }
        } else {
            (0.0, 0.0)
        };

        self.last_sample = Some(NetworkSample {
            timestamp: now,
            received: total_received,
            transmitted: total_transmitted,
        });

        let network = NetworkMetrics {
            total_received,
            total_transmitted,
            received_per_second,
            transmitted_per_second,
        };

        let load_avg = self.system.load_average();
        let load_average = Some(LoadAverages {
            one: load_avg.one,
            five: load_avg.five,
            fifteen: load_avg.fifteen,
        });

        Ok(SystemMetrics {
            cpu_usage,
            memory_usage: memory_percent,
            network_io: (rx_delta, tx_delta),
            memory,
            disk,
            network,
            load_average,
            uptime: Duration::from_secs(self.system.uptime()),
            process_count: self.system.processes().len(),
            last_update: Some(now),
        })
    }
}

fn kb_to_bytes(value: u64) -> u64 {
    value.saturating_mul(1024)
}
