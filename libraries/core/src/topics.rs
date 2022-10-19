pub const MANUAL_STOP: &str = "dora/stop";

pub const ZENOH_CONTROL_QUERYABLE: &str = "dora_control/*";
pub const ZENOH_CONTROL_START: &str = "dora_control/start";
pub const ZENOH_CONTROL_STOP: &str = "dora_control/stop";
pub const ZENOH_CONTROL_DESTROY: &str = "dora_control/destroy";

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum StartDataflowResult {
    Ok { uuid: String },
    Error(String),
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum StopDataflowResult {
    Ok,
    Error(String),
}
