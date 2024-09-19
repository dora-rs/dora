use std::net::{IpAddr, Ipv4Addr};

pub const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const DORA_COORDINATOR_PORT_DEFAULT: u16 = 53290;
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT: u16 = 53291;
pub const DORA_COORDINATOR_PORT_CONTROL_DEFAULT: u16 = 6012;

pub const MANUAL_STOP: &str = "dora/stop";
