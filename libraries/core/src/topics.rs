pub const ZENOH_CONTROL_QUERYABLE: &str = "dora_control/*";
pub const ZENOH_CONTROL_START: &str = "dora_control/start";
pub const ZENOH_CONTROL_DESTROY: &str = "dora_control/destroy";

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum StartDataflowResult {
    Ok { uuid: String },
    Error(String),
}
