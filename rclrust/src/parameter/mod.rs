mod parameters;
mod rcl_params;
mod type_;
mod value;

use std::fmt;

pub(crate) use parameters::Parameters;
pub(crate) use rcl_params::RclParams;
pub(crate) use rclrust_msg::rcl_interfaces::msg::{
    Parameter as RclParameter, ParameterDescriptor, ParameterType as RclParameterType,
    ParameterValue as RclParameterValue, SetParametersResult,
};
pub use type_::ParameterType;
pub use value::ParameterValue;

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Parameter {
    pub name: String,
    pub value: ParameterValue,
}

impl fmt::Display for Parameter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            r#"Parameter(name="{}", value={})"#,
            self.name, self.value
        )
    }
}

impl Parameter {
    pub fn get_type(&self) -> ParameterType {
        self.value.get_type()
    }

    pub fn not_set(name: &str) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::not_set(),
        }
    }

    pub fn bool(name: &str, v: bool) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::bool(v),
        }
    }

    pub fn integer<T: Into<i64>>(name: &str, v: T) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::integer(v),
        }
    }

    pub fn double<T: Into<f64>>(name: &str, v: T) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::double(v),
        }
    }

    pub fn string<T: Into<String>>(name: &str, v: T) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::string(v),
        }
    }

    pub fn byte_array<T: Into<Vec<u8>>>(name: &str, v: T) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::byte_array(v),
        }
    }

    pub fn bool_array<T: Into<Vec<bool>>>(name: &str, v: T) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::bool_array(v),
        }
    }

    pub fn integer_array<T: Into<Vec<i64>>>(name: &str, v: T) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::integer_array(v),
        }
    }

    pub fn double_array<T: Into<Vec<f64>>>(name: &str, v: T) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::double_array(v),
        }
    }

    pub fn string_array(name: &str, v: Vec<String>) -> Self {
        Self {
            name: name.into(),
            value: ParameterValue::string_array(v),
        }
    }
}

impl From<RclParameter> for Parameter {
    fn from(v: RclParameter) -> Self {
        let RclParameter { name, value } = v;
        Self {
            name,
            value: value.into(),
        }
    }
}
