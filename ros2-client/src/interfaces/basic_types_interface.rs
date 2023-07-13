use serde::{Deserialize, Serialize};

use crate::{Message, Service};

pub struct BasicTypesService {}

impl Service for BasicTypesService {
  type Request = BasicTypesRequest;
  type Response = BasicTypesResponse;
  fn request_type_name(&self) -> &str {
    "test_msgs::srv::dds_::BasicTypes_Request_"
  }
  fn response_type_name(&self) -> &str {
    "test_msgs::srv::dds_::BasicTypes_Response_"
  }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct BasicTypesRequest {
  pub bool_value: bool,
  pub byte_value: Vec<u8>,
  pub char_value: u8,
  pub float32_value: f32,
  pub float64_value: f64,
  pub int8_value: i8,
  pub uint8_value: u8,
  pub int16_value: i16,
  pub uint16_value: u16,
  pub int32_value: i32,
  pub uint32_value: u32,
  pub int64_value: i64,
  pub uint64_value: u64,
  pub string_value: String,
}
impl BasicTypesRequest {
  pub fn new() -> BasicTypesRequest {
    BasicTypesRequest {
      bool_value: true,
      byte_value: vec![],
      char_value: 0,
      float32_value: 0.0,
      float64_value: 0.0,
      int8_value: 0,
      uint8_value: 0,
      int16_value: 0,
      uint16_value: 0,
      int32_value: 0,
      uint32_value: 0,
      int64_value: 0,
      uint64_value: 0,
      string_value: String::from("From RustDDS service, this a Request"),
    }
  }
}
impl Message for BasicTypesRequest {}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct BasicTypesResponse {
  pub bool_value: bool,
  pub byte_value: Vec<u8>,
  pub char_value: u8,
  pub float32_value: f32,
  pub float64_value: f64,
  pub int8_value: i8,
  pub uint8_value: u8,
  pub int16_value: i16,
  pub uint16_value: u16,
  pub int32_value: i32,
  pub uint32_value: u32,
  pub int64_value: i64,
  pub uint64_value: u64,
  pub string_value: String,
}
impl BasicTypesResponse {
  pub fn new() -> BasicTypesResponse {
    BasicTypesResponse {
      bool_value: true,
      byte_value: vec![],
      char_value: 0,
      float32_value: 0.0,
      float64_value: 0.0,
      int8_value: 0,
      uint8_value: 0,
      int16_value: 0,
      uint16_value: 0,
      int32_value: 0,
      uint32_value: 0,
      int64_value: 0,
      uint64_value: 0,
      string_value: String::from("From RustDDS service, this a Response"),
    }
  }
}
impl Message for BasicTypesResponse {}
