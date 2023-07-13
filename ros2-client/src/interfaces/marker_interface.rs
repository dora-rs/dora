use serde::{Deserialize, Serialize};

use crate::{Message, Service};

pub struct MarkerService {}

impl Service for MarkerService {
  type Request = MarkerRequest;
  type Response = MarkerResponse;
  fn request_type_name() -> String {
    "rustdds_gateway_idl::srv::dds_::Marker_Request_".to_owned()
  }
  fn response_type_name() -> String {
    "rustdds_gateway_idl::srv::dds_::Marker_Response_".to_owned()
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarkerRequest {
  pub marker: String,
}
impl Message for MarkerRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarkerResponse {
  pub marker: String,
}
impl Message for MarkerResponse {}
