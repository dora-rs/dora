use rustdds::{policy::*, *};

pub mod ros_discovery {
  use super::*;

  lazy_static! {
    pub static ref QOS_PUB: QosPolicies = QosPolicyBuilder::new()
      .durability(Durability::TransientLocal)
      .deadline(Deadline(Duration::DURATION_INFINITE))
      .ownership(Ownership::Shared)
      .reliability(Reliability::Reliable {
        max_blocking_time: Duration::DURATION_ZERO
      })
      .history(History::KeepLast { depth: 1 })
      .lifespan(Lifespan {
        duration: Duration::DURATION_INFINITE
      })
      .build();
    pub static ref QOS_SUB: QosPolicies = QosPolicyBuilder::new()
      .durability(Durability::Volatile)
      .ownership(Ownership::Shared)
      .reliability(Reliability::Reliable {
        max_blocking_time: Duration::DURATION_ZERO
      })
      .history(History::KeepLast { depth: 1 })
      .lifespan(Lifespan {
        duration: Duration::DURATION_INFINITE
      })
      .build();
  }

  pub const TOPIC_NAME: &str = "ros_discovery_info";

  pub const TYPE_NAME: &str = "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_";
}

pub mod parameter_events {
  use super::*;

  lazy_static! {
    pub static ref QOS: QosPolicies = QosPolicyBuilder::new()
      .durability(Durability::TransientLocal)
      .reliability(Reliability::Reliable {
        max_blocking_time: Duration::DURATION_ZERO
      })
      .history(History::KeepLast { depth: 1 })
      .build();
  }

  pub const TOPIC_NAME: &str = "rt/parameter_events";

  pub const TYPE_NAME: &str = "rcl_interfaces::msg::dds_::ParameterEvent_";
}

pub mod rosout {
  use super::*;

  lazy_static! {
    pub static ref QOS: QosPolicies = QosPolicyBuilder::new()
      .durability(Durability::TransientLocal)
      .deadline(Deadline(Duration::DURATION_INFINITE))
      .ownership(Ownership::Shared)
      .reliability(Reliability::Reliable {
        max_blocking_time: Duration::DURATION_ZERO
      })
      .history(History::KeepLast { depth: 1 })
      .lifespan(Lifespan {
        duration: Duration::from_secs(10)
      })
      .build();
  }

  pub const TOPIC_NAME: &str = "rt/rosout";

  pub const TYPE_NAME: &str = "rcl_interfaces::msg::dds_::Log_";
}
