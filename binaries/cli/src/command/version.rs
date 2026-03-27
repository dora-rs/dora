use super::{Executable, default_tracing};
use crate::{LOCALHOST, common::connect_to_coordinator_rpc, get_python_dora_version};
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::tarpc;

#[derive(Debug, clap::Args)]
/// Show detailed version information for CLI, message format, and coordinator.
pub struct Version {
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: std::net::IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    coordinator_port: u16,
}

impl Executable for Version {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        println!("CLI version:                {}", env!("CARGO_PKG_VERSION"));
        println!("CLI message format version: {}", dora_message::VERSION);

        match get_python_dora_version() {
            Some(v) => println!("Python dora-rs version:     {v}"),
            None => println!("Python dora-rs version:     not found"),
        }

        match connect_to_coordinator_rpc(self.coordinator_addr, self.coordinator_port).await {
            Ok(client) => match client.get_version(tarpc::context::current()).await {
                Ok(info) => {
                    println!("Coordinator version:        {}", info.coordinator_version);
                    println!(
                        "Coordinator message format: {}",
                        info.message_format_version
                    );
                }
                Err(_) => {
                    println!("Coordinator version:        unknown (RPC not supported)");
                }
            },
            Err(_) => {
                println!("Coordinator version:        not reachable");
            }
        }

        Ok(())
    }
}
