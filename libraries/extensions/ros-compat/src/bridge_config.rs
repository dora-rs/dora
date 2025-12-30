//! Bridge configuration for ROS ↔ Dora message bridging

use eyre::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Bridge configuration loaded from YAML
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BridgeConfig {
    /// ROS version (1 or 2)
    pub ros_version: u8,
    
    /// ROS Master URI (for ROS 1)
    #[serde(default)]
    pub ros_master_uri: Option<String>,
    
    /// List of bridge mappings
    pub bridges: Vec<BridgeMapping>,
}

/// A single bridge mapping between ROS and Dora
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BridgeMapping {
    /// ROS topic name
    pub ros_topic: String,
    
    /// Dora topic name
    pub dora_topic: String,
    
    /// ROS message type (e.g., "sensor_msgs/Image")
    pub msg_type: String,
    
    /// Bridge direction
    pub direction: BridgeDirection,
    
    /// QoS settings (for ROS 2)
    #[serde(default)]
    pub qos: Option<QosSettings>,
}

/// Bridge direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BridgeDirection {
    /// ROS → Dora
    RosToDora,
    /// Dora → ROS
    DoraToRos,
    /// Bidirectional
    Bidirectional,
}

/// QoS settings for ROS 2
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QosSettings {
    /// Reliability setting
    #[serde(default)]
    pub reliability: Option<String>, // "reliable" or "best_effort"
    
    /// Durability setting
    #[serde(default)]
    pub durability: Option<String>, // "volatile", "transient_local", etc.
    
    /// History depth
    #[serde(default)]
    pub depth: Option<i32>,
}

impl BridgeConfig {
    /// Load bridge configuration from a YAML file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = std::fs::read_to_string(path.as_ref())
            .with_context(|| format!("Failed to read bridge config: {}", path.as_ref().display()))?;
        
        let config: BridgeConfig = serde_yaml::from_str(&content)
            .with_context(|| "Failed to parse bridge configuration")?;
        
        Ok(config)
    }
    
    /// Validate the configuration
    pub fn validate(&self) -> Result<()> {
        if self.ros_version != 1 && self.ros_version != 2 {
            eyre::bail!("ros_version must be 1 or 2, got {}", self.ros_version);
        }
        
        if self.ros_version == 1 && self.ros_master_uri.is_none() {
            // Try to get from environment
            if std::env::var("ROS_MASTER_URI").is_err() {
                eyre::bail!("ROS_MASTER_URI must be set for ROS 1");
            }
        }
        
        for bridge in &self.bridges {
            if bridge.ros_topic.is_empty() {
                eyre::bail!("ros_topic cannot be empty");
            }
            if bridge.dora_topic.is_empty() {
                eyre::bail!("dora_topic cannot be empty");
            }
            if bridge.msg_type.is_empty() {
                eyre::bail!("msg_type cannot be empty");
            }
        }
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_bridge_config() {
        let yaml = r#"
ros_version: 1
ros_master_uri: "http://localhost:11311"
bridges:
  - ros_topic: /camera/image_raw
    dora_topic: /sensors/camera
    msg_type: sensor_msgs/Image
    direction: ros_to_dora
  - dora_topic: /control/cmd_vel
    ros_topic: /mobile_base/commands/velocity
    msg_type: geometry_msgs/Twist
    direction: dora_to_ros
"#;
        
        let config: BridgeConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.ros_version, 1);
        assert_eq!(config.bridges.len(), 2);
    }
}

