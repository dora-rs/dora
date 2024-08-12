use std::net::{IpAddr, Ipv4Addr};

pub const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const DORA_COORDINATOR_PORT_DEFAULT: u16 = 0xD02A;
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT: u16 = 0xD02B;
pub const DORA_COORDINATOR_PORT_CONTROL_DEFAULT: u16 = 0x177C;

pub const MANUAL_STOP: &str = "dora/stop";
