use serde::{Deserialize, Serialize};

use crate::{Message, Service};

pub struct ListParametersService {}

impl Service for ListParametersService {
  type Request = ListParametersRequest;
  type Response = ListParametersResponse;
  fn request_type_name(&self) -> &str {
    "rcl_interfaces::srv::dds_::ListParameters_Request_"
  }
  fn response_type_name(&self) -> &str {
    "rcl_interfaces::srv::dds_::ListParameters_Response_"
  }
}

#[allow(non_snake_case)]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ListParametersRequest {
  pub DEPTH_RECURSIVE: u64,
  pub prefixes: Vec<String>,
  pub depth: u64,
}
impl Message for ListParametersRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ListParametersResponse {
  pub result: ListParametersResult,
}
impl Message for ListParametersResponse {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ListParametersResult {
  pub names: Vec<String>,
  pub prefixes: Vec<String>,
}
