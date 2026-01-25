use crate::command::Executable;
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
}

impl Executable for Rerun {
    fn execute(self) -> Result<()> {
        println!("🔭 Starting Rerun visualization viewer...");

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
