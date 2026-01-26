use crate::LOCALHOST;
use crate::command::Executable;
use crate::common::{connect_to_coordinator, query_running_dataflows};
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use eyre::{Context, Result, bail};
use std::process::Command;

/// Launch Rerun visualization viewer for Dora dataflows
#[derive(Debug, clap::Args)]
pub struct Rerun {
    /// Port for the Rerun viewer web server
    #[clap(long, default_value = "9876")]
    port: u16,

    /// Connect to an existing Rerun viewer instead of spawning a new one
    #[clap(long)]
    connect: Option<String>,

    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: std::net::IpAddr,

    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    coordinator_port: u16,
}

impl Executable for Rerun {
    fn execute(self) -> Result<()> {
        println!("🔭 Starting Rerun visualization viewer...\n");

        let rerun_check = Command::new("rerun").arg("--version").output();

        match rerun_check {
            Ok(output) if output.status.success() => {
                let version = String::from_utf8_lossy(&output.stdout);
                println!("✓ Found Rerun: {}", version.trim());
            }
            _ => {
                bail!(
                    "Rerun CLI not found. Please install it with:\n\n  \
                    pip install rerun-sdk\n\n\
                    For more information, see: https://rerun.io/docs/getting-started"
                );
            }
        }

        // Try to connect to dora coordinator and show active dataflows
        let coordinator_addr = (self.coordinator_addr, self.coordinator_port).into();
        match connect_to_coordinator(coordinator_addr) {
            Ok(mut session) => {
                println!("✓ Connected to dora coordinator at {}", coordinator_addr);

                match query_running_dataflows(&mut *session) {
                    Ok(list) => {
                        let active = list.get_active();
                        if active.is_empty() {
                            println!("  No dataflows running\n");
                        } else {
                            println!("  Active dataflows:");
                            for dataflow in &active {
                                let name = dataflow.name.as_deref().unwrap_or("<unnamed>");
                                println!("    • {} ({})", name, dataflow.uuid);
                            }
                            println!(
                                "\n💡 Add 'dora-rerun' node to visualize: https://github.com/dora-rs/dora-hub/tree/main/node-hub/dora-rerun\n"
                            );
                        }
                    }
                    Err(_) => {
                        println!("  Could not query dataflows\n");
                    }
                }
            }
            Err(_) => {
                println!("  Coordinator not running. Start with: dora up\n");
            }
        }

        let mut cmd = Command::new("rerun");

        cmd.arg("--web-viewer-port").arg(self.port.to_string());

        if let Some(ref connect_addr) = self.connect {
            cmd.arg("--connect").arg(connect_addr);
            println!("✓ Connecting to Rerun viewer at {}", connect_addr);
        } else {
            println!("✓ Launching Rerun viewer on http://localhost:{}", self.port);
        }

        println!("\n📊 Starting Rerun...\n");

        let status = cmd.status().wrap_err("Failed to start Rerun viewer")?;

        if !status.success() {
            bail!("Rerun viewer exited with status: {}", status);
        }

        Ok(())
    }
}
