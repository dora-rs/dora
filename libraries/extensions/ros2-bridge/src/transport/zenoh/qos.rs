use std::fmt;

use crate::transport::{Durability, History, Liveliness, Reliability, Ros2Qos};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ZenohQosMapping(String);

impl ZenohQosMapping {
    pub fn from_ros_qos(qos: &Ros2Qos) -> Self {
        let reliability = match qos.reliability {
            Reliability::Reliable { .. } => "",
            Reliability::BestEffort => "2",
        };
        let durability = match qos.durability {
            Durability::Volatile => "",
            Durability::TransientLocal => "1",
        };
        let (history, depth) = match qos.history {
            History::KeepLast { depth } => ("", depth.to_string()),
            History::KeepAll => ("2", String::new()),
        };
        let liveliness = match qos.liveliness {
            Liveliness::Automatic { .. } => "",
            Liveliness::ManualByParticipant { .. } | Liveliness::ManualByTopic { .. } => "3",
        };
        Self(format!(
            "{reliability}:{durability}:{history},{depth}:,:,:,{liveliness},"
        ))
    }
}
impl fmt::Display for ZenohQosMapping {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}
