use serde::{Deserialize, Serialize};

use crate::enums::CommandType;

#[derive(Debug, Serialize, Deserialize)]
pub struct JsonData {
    pub sleep_second: u64,
    pub command: CommandType,
}
