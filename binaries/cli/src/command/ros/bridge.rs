use clap::Args;
use std::path::PathBuf;

use crate::command::{Executable, default_tracing};

/// Start ROS bridge node for bidirectional communication between ROS and Dora.
///
/// Examples:
///
/// Start bridge with configuration file:
///   dora ros bridge --config ros_bridge.yml
///
/// Start bridge for ROS 1:
///   dora ros bridge --config bridge.yml --ros-version 1
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Bridge {
    /// Path to bridge configuration file
    #[clap(long, short, value_name = "PATH")]
    pub config: Option<PathBuf>,

    /// ROS version (1 or 2)
    #[clap(long, value_name = "VERSION", default_value = "2")]
    pub ros_version: String,

    /// ROS Master URI (for ROS 1)
    #[clap(long, value_name = "URI")]
    pub ros_master_uri: Option<String>,
}

impl Executable for Bridge {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        start_bridge(self)
    }
}

fn start_bridge(args: Bridge) -> eyre::Result<()> {
    use dora_ros_compat::bridge_config::BridgeConfig;

    println!("Starting ROS bridge...");

    // Load configuration
    let config = if let Some(config_path) = &args.config {
        let mut config = BridgeConfig::from_file(config_path)?;
        config.validate()?;

        // Override ROS version if specified
        if let Ok(version) = args.ros_version.parse::<u8>() {
            config.ros_version = version;
        }

        // Override ROS Master URI if specified
        if let Some(master_uri) = &args.ros_master_uri {
            config.ros_master_uri = Some(master_uri.clone());
        }

        config
    } else {
        eyre::bail!("Bridge configuration file is required. Use --config <path>");
    };

    println!("ROS Version: {}", config.ros_version);
    if let Some(master_uri) = &config.ros_master_uri {
        println!("ROS Master URI: {}", master_uri);
    }
    println!("Bridging {} topics:", config.bridges.len());

    for bridge in &config.bridges {
        let dir_str = match bridge.direction {
            dora_ros_compat::bridge_config::BridgeDirection::RosToDora => "ROS→Dora",
            dora_ros_compat::bridge_config::BridgeDirection::DoraToRos => "Dora→ROS",
            dora_ros_compat::bridge_config::BridgeDirection::Bidirectional => "Bidirectional",
        };
        println!(
            "  {} ({}) ↔ {} ({})",
            bridge.ros_topic, dir_str, bridge.dora_topic, bridge.msg_type
        );
    }

    // Start bridge based on ROS version
    match config.ros_version {
        1 => start_ros1_bridge(config)?,
        2 => start_ros2_bridge(config)?,
        _ => eyre::bail!("Unsupported ROS version: {}", config.ros_version),
    }

    Ok(())
}

fn start_ros1_bridge(_config: dora_ros_compat::bridge_config::BridgeConfig) -> eyre::Result<()> {
    println!("Starting ROS 1 bridge...");

    #[cfg(feature = "ros1")]
    {
        use dora_ros_compat::bridge::RosBridge;

        let mut bridge = RosBridge::new(_config)?;

        tokio::runtime::Runtime::new()?.block_on(async {
            bridge.start().await?;
            Ok::<(), eyre::Error>(())
        })?;
        Ok(())
    }

    #[cfg(not(feature = "ros1"))]
    {
        eyre::bail!(
            "ROS 1 support requires 'ros1' feature. Build dora-ros-compat with: cargo build --features ros1"
        )
    }
}

fn start_ros2_bridge(_config: dora_ros_compat::bridge_config::BridgeConfig) -> eyre::Result<()> {
    println!("Starting ROS 2 bridge...");
    println!("Note: ROS 2 bridge integration with dora-ros2-bridge coming soon");

    // For now, use the bridge implementation if available
    // In the future, integrate with existing dora-ros2-bridge
    #[cfg(any(feature = "ros1", feature = "ros2"))]
    {
        use dora_ros_compat::bridge::RosBridge;

        let mut bridge = RosBridge::new(_config)?;

        tokio::runtime::Runtime::new()?.block_on(async {
            bridge.start().await?;
            Ok::<(), eyre::Error>(())
        })?;
        Ok(())
    }

    #[cfg(not(any(feature = "ros1", feature = "ros2")))]
    {
        eyre::bail!("ROS bridge support requires 'ros1' or 'ros2' feature")
    }
}
