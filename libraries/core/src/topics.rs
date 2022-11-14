use std::{
    net::{Ipv4Addr, SocketAddr},
    path::PathBuf,
};
use uuid::Uuid;

pub const MANUAL_STOP: &str = "dora/stop";

pub fn control_socket_addr() -> SocketAddr {
    SocketAddr::new(Ipv4Addr::new(127, 0, 0, 1).into(), 6012)
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum ControlRequest {
    Start { dataflow_path: PathBuf },
    Stop { dataflow_uuid: Uuid },
    Destroy,
    List,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum StartDataflowResult {
    Ok { uuid: Uuid },
    Error(String),
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum StopDataflowResult {
    Ok,
    Error(String),
}
