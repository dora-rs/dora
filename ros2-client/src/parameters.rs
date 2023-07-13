/// Rust-like representation of ROS2 Parameter
pub struct Parameter {
  pub name: String,
  pub value: ParameterValue,
}

/// Rust-like representation of ROS2
/// [ParameterValue](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterValue.msg)
pub enum ParameterValue {
  NotSet,
  Boolean(bool),
  Integer(i64),
  Double(f64),
  String(String),
  ByteArray(Vec<u8>),
  BooleanArray(Vec<bool>),
  IntegerArray(Vec<i64>),
  DoubleArray(Vec<f64>),
  StringArray(Vec<String>),
}

impl From<raw::Parameter> for Parameter {
  fn from(rp: raw::Parameter) -> Self {
    let pv = match rp.value.ptype {
      raw::ParameterType::NOT_SET => ParameterValue::NotSet,
      raw::ParameterType::BOOL => ParameterValue::Boolean(rp.value.boolean_value),
      raw::ParameterType::INTEGER => ParameterValue::Integer(rp.value.int_value),
      raw::ParameterType::DOUBLE => ParameterValue::Double(rp.value.double_value),
      raw::ParameterType::STRING => ParameterValue::String(rp.value.string_value),

      raw::ParameterType::BYTE_ARRAY => ParameterValue::ByteArray(rp.value.byte_array),
      raw::ParameterType::BOOL_ARRAY => ParameterValue::BooleanArray(rp.value.bool_array),
      raw::ParameterType::INTEGER_ARRAY => ParameterValue::IntegerArray(rp.value.int_array),
      raw::ParameterType::DOUBLE_ARRAY => ParameterValue::DoubleArray(rp.value.double_array),
      raw::ParameterType::STRING_ARRAY => ParameterValue::StringArray(rp.value.string_array),

      _ =>
      // This may be an uspecified case.
      // TODO: Do something better, at least log a warning.
      {
        ParameterValue::NotSet
      }
    };

    Parameter {
      name: rp.name,
      value: pv,
    }
  }
}

impl From<Parameter> for raw::Parameter {
  fn from(p: Parameter) -> raw::Parameter {
    let mut value = raw::ParameterValue {
      ptype: raw::ParameterType::NOT_SET,
      boolean_value: false,
      int_value: 0,
      double_value: 0.0,
      string_value: String::new(),
      byte_array: Vec::new(),
      int_array: Vec::new(),
      bool_array: Vec::new(),
      double_array: Vec::new(),
      string_array: Vec::new(),
    };
    match p.value {
      ParameterValue::NotSet => (), // already there
      ParameterValue::Boolean(b) => {
        value.ptype = raw::ParameterType::BOOL;
        value.boolean_value = b;
      }
      ParameterValue::Integer(i) => {
        value.ptype = raw::ParameterType::INTEGER;
        value.int_value = i;
      }
      ParameterValue::Double(d) => {
        value.ptype = raw::ParameterType::DOUBLE;
        value.double_value = d;
      }
      ParameterValue::String(s) => {
        value.ptype = raw::ParameterType::STRING;
        value.string_value = s;
      }
      ParameterValue::ByteArray(a) => {
        value.ptype = raw::ParameterType::BYTE_ARRAY;
        value.byte_array = a;
      }
      ParameterValue::BooleanArray(a) => {
        value.ptype = raw::ParameterType::BOOL_ARRAY;
        value.bool_array = a;
      }
      ParameterValue::IntegerArray(a) => {
        value.ptype = raw::ParameterType::INTEGER_ARRAY;
        value.int_array = a;
      }
      ParameterValue::DoubleArray(a) => {
        value.ptype = raw::ParameterType::DOUBLE_ARRAY;
        value.double_array = a;
      }
      ParameterValue::StringArray(a) => {
        value.ptype = raw::ParameterType::STRING_ARRAY;
        value.string_array = a;
      }
    }

    raw::Parameter {
      name: p.name,
      value,
    }
  }
}

// This submodule contains raw, ROS2 -compatible Parameters.
//
pub(crate) mod raw {
  use rustdds::*;
  use serde::{Deserialize, Serialize};

  /// ROS2 [ParameterEvent](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterEvent.msg)
  #[derive(Debug, Clone, Serialize, Deserialize)]
  pub struct ParameterEvent {
    pub timestamp: Timestamp,
    // fully qualified path
    pub node: String,
    pub new_parameters: Vec<Parameter>,
    pub changed_parameters: Vec<Parameter>,
    pub deleted_parameters: Vec<Parameter>,
  }

  /// [Parameter](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/Parameter.msg)
  #[derive(Debug, Clone, Serialize, Deserialize)]
  pub struct Parameter {
    pub name: String,
    pub value: ParameterValue,
  }

  /// [ParameterValue](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterValue.msg)
  #[derive(Debug, Clone, Serialize, Deserialize)]
  pub struct ParameterValue {
    pub ptype: u8,
    pub boolean_value: bool,
    pub int_value: i64,
    pub double_value: f64,
    pub string_value: String,
    pub byte_array: Vec<u8>,
    pub bool_array: Vec<bool>,
    pub int_array: Vec<i64>,
    pub double_array: Vec<f64>,
    pub string_array: Vec<String>,
  }

  /// ROS2 defines this as an empty .msg
  /// [ParameterType](https://github.com/ros2/rcl_interfaces/blob/master/rcl_interfaces/msg/ParameterType.msg)
  pub struct ParameterType {}

  impl ParameterType {
    pub const NOT_SET: u8 = 0;

    pub const BOOL: u8 = 1;
    pub const INTEGER: u8 = 2;
    pub const DOUBLE: u8 = 3;
    pub const STRING: u8 = 4;
    pub const BYTE_ARRAY: u8 = 5;
    pub const BOOL_ARRAY: u8 = 6;
    pub const INTEGER_ARRAY: u8 = 7;
    pub const DOUBLE_ARRAY: u8 = 8;
    pub const STRING_ARRAY: u8 = 9;
  }
}
