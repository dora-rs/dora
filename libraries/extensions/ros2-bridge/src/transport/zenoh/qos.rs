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
        // Upstream `rmw_zenoh_cpp` encodes the trailing group as
        // `<liveliness_kind>,<lease_sec>,<lease_nsec>` — the liveliness *kind*
        // is the first subfield. Keep it first (with empty lease sec/nsec) so a
        // real `rmw_zenoh` peer reads the kind, not the lease-seconds slot.
        Self(format!(
            "{reliability}:{durability}:{history},{depth}:,:,:{liveliness},,"
        ))
    }
}
impl fmt::Display for ZenohQosMapping {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}
