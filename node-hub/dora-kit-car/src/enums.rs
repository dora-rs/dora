use serde::{Deserialize, Serialize};

// command type
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "command_type")]
pub enum CommandType {
    DifferSpeed { x: f64, y: f64, w: f64 }, // Differential Speed
}
