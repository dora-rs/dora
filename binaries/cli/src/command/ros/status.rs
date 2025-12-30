use clap::Args;

use crate::command::{Executable, default_tracing};

/// View ROS bridge status and active connections.
///
/// Examples:
///
/// View bridge status:
///   dora ros status
///
/// View status for specific ROS version:
///   dora ros status --ros-version 1
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Status {
    /// ROS version (1 or 2)
    #[clap(long, value_name = "VERSION", default_value = "2")]
    pub ros_version: String,
}

impl Executable for Status {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        show_status(self)
    }
}

fn show_status(args: Status) -> eyre::Result<()> {
    use dora_ros_compat::bridge_config::BridgeConfig;
    use std::path::PathBuf;

    println!("ROS Bridge Status");
    println!("==================");
    println!("ROS Version: ROS {}", args.ros_version);

    // Try to find and load bridge configuration
    let config_paths = [
        PathBuf::from("ros_bridge.yml"),
        PathBuf::from(".dora/ros_bridge.yml"),
        PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(".dora/ros_bridge.yml"),
    ];

    let mut found_config = false;
    for path in &config_paths {
        if path.exists() {
            match BridgeConfig::from_file(path) {
                Ok(config) => {
                    found_config = true;
                    println!("\nConfiguration: {}", path.display());
                    println!(
                        "ROS Master URI: {}",
                        config.ros_master_uri.as_deref().unwrap_or("Not set")
                    );
                    println!("Active Bridges: {}", config.bridges.len());

                    for (i, bridge) in config.bridges.iter().enumerate() {
                        let dir_str = match bridge.direction {
                            dora_ros_compat::bridge_config::BridgeDirection::RosToDora => {
                                "ROS→Dora"
                            }
                            dora_ros_compat::bridge_config::BridgeDirection::DoraToRos => {
                                "Dora→ROS"
                            }
                            dora_ros_compat::bridge_config::BridgeDirection::Bidirectional => {
                                "Bidirectional"
                            }
                        };
                        println!(
                            "  {}. {} ({}) ↔ {} ({})",
                            i + 1,
                            bridge.ros_topic,
                            dir_str,
                            bridge.dora_topic,
                            bridge.msg_type
                        );
                    }
                    break;
                }
                Err(e) => {
                    eprintln!(
                        "Warning: Failed to load config from {}: {}",
                        path.display(),
                        e
                    );
                }
            }
        }
    }

    if !found_config {
        println!("\nNo active bridge configuration found.");
        println!("Create a ros_bridge.yml file to configure bridges.");
    }

    // Check ROS environment
    if args.ros_version == "1" {
        if let Ok(master_uri) = std::env::var("ROS_MASTER_URI") {
            println!("\nROS Master URI: {}", master_uri);
        } else {
            println!("\nROS Master URI: Not set (using default: http://localhost:11311)");
        }
    }

    Ok(())
}
