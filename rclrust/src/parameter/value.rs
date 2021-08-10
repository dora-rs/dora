use std::fmt;

use super::{ParameterDescriptor, ParameterType, RclParameterType, RclParameterValue};
use crate::internal::ffi::SizedFromCChar;

#[derive(Debug, Clone, PartialEq)]
pub enum Variant {
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    BoolArray(Vec<bool>),
    ByteArray(Vec<u8>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl fmt::Display for Variant {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Bool(v) => write!(f, "Bool({})", v),
            Self::Integer(v) => write!(f, "Integer({})", v),
            Self::Double(v) => write!(f, "Double({})", v),
            Self::String(v) => write!(f, "String({})", v),
            Self::ByteArray(v) => write!(f, "ByteArray({:?})", v),
            Self::BoolArray(v) => write!(f, "BoolArray({:?})", v),
            Self::IntegerArray(v) => write!(f, "IntegerArray({:?})", v),
            Self::DoubleArray(v) => write!(f, "DoubleArray({:?})", v),
            Self::StringArray(v) => write!(f, "StringArray({:?})", v),
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ParameterValue(RclParameterValue);

impl ParameterValue {
    pub const fn new(value: RclParameterValue) -> Self {
        Self(value)
    }

    pub fn not_set() -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_NOT_SET,
            ..Default::default()
        })
    }

    pub fn bool(v: bool) -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_BOOL,
            bool_value: v,
            ..Default::default()
        })
    }

    pub fn integer<T: Into<i64>>(v: T) -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_INTEGER,
            integer_value: v.into(),
            ..Default::default()
        })
    }

    pub fn double<T: Into<f64>>(v: T) -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_DOUBLE,
            double_value: v.into(),
            ..Default::default()
        })
    }

    pub fn string<T: Into<String>>(v: T) -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_STRING,
            string_value: v.into(),
            ..Default::default()
        })
    }

    pub fn byte_array<T: Into<Vec<u8>>>(v: T) -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_BYTE_ARRAY,
            byte_array_value: v.into(),
            ..Default::default()
        })
    }

    pub fn bool_array<T: Into<Vec<bool>>>(v: T) -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_BOOL_ARRAY,
            bool_array_value: v.into(),
            ..Default::default()
        })
    }

    pub fn integer_array<T: Into<Vec<i64>>>(v: T) -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_INTEGER_ARRAY,
            integer_array_value: v.into(),
            ..Default::default()
        })
    }

    pub fn double_array<T: Into<Vec<f64>>>(v: T) -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_DOUBLE_ARRAY,
            double_array_value: v.into(),
            ..Default::default()
        })
    }

    pub fn string_array(v: Vec<String>) -> Self {
        Self(RclParameterValue {
            type_: RclParameterType::PARAMETER_STRING_ARRAY,
            string_array_value: v,
            ..Default::default()
        })
    }

    pub(crate) const fn get_u8_type(&self) -> u8 {
        self.0.type_
    }

    pub fn get_type(&self) -> ParameterType {
        self.0.type_.into()
    }

    pub fn get_value(&self) -> Option<Variant> {
        Some(match self.get_type() {
            ParameterType::NotSet => return None,
            ParameterType::Bool => Variant::Bool(self.0.bool_value),
            ParameterType::Integer => Variant::Integer(self.0.integer_value),
            ParameterType::Double => Variant::Double(self.0.double_value),
            ParameterType::String => Variant::String(self.0.string_value.clone()),
            ParameterType::ByteArray => Variant::ByteArray(self.0.byte_array_value.clone()),
            ParameterType::BoolArray => Variant::BoolArray(self.0.bool_array_value.clone()),
            ParameterType::IntegerArray => {
                Variant::IntegerArray(self.0.integer_array_value.clone())
            }
            ParameterType::DoubleArray => Variant::DoubleArray(self.0.double_array_value.clone()),
            ParameterType::StringArray => Variant::StringArray(self.0.string_array_value.clone()),
        })
    }

    pub(crate) fn check_range(
        &self,
        desc: &ParameterDescriptor,
    ) -> std::result::Result<(), String> {
        match self.get_type() {
            ParameterType::Integer => {
                let ok = if desc.integer_range.is_empty() {
                    true
                } else {
                    let v = self.0.integer_value;
                    let range = &desc.integer_range[0];
                    if v == range.from_value || v == range.to_value {
                        true
                    } else if v < range.from_value || range.to_value < v {
                        false
                    } else {
                        range.step == 0 || (v - range.from_value).rem_euclid(range.step as i64) == 0
                    }
                };

                if ok {
                    Ok(())
                } else {
                    Err(format!(
                        "Parameter {{{}}} doesn't comply with integer range.",
                        desc.name
                    ))
                }
            }
            ParameterType::Double => {
                let ok = if desc.floating_point_range.is_empty() {
                    true
                } else {
                    let equal =
                        |x: f64, y: f64| (x - y).abs() <= f64::EPSILON * (x + y).abs() * 100.;

                    let v = self.0.double_value;
                    let range = &desc.floating_point_range[0];
                    if equal(v, range.from_value) || equal(v, range.to_value) {
                        true
                    } else if v < range.from_value || range.to_value < v {
                        false
                    } else {
                        range.step == 0.
                            || equal(
                                v,
                                ((v - range.from_value) / range.step)
                                    .round()
                                    .mul_add(range.step, range.from_value),
                            )
                    }
                };

                if ok {
                    Ok(())
                } else {
                    Err(format!(
                        "Parameter {{{}}} doesn't comply with floating point range.",
                        desc.name
                    ))
                }
            }
            _ => Ok(()),
        }
    }
}

impl From<RclParameterValue> for ParameterValue {
    fn from(v: RclParameterValue) -> Self {
        Self::new(v)
    }
}

impl fmt::Display for ParameterValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.get_value() {
            Some(v) => write!(f, "{}", v),
            None => write!(f, "NotSet"),
        }
    }
}

impl From<&rcl_sys::rcl_variant_t> for ParameterValue {
    fn from(v: &rcl_sys::rcl_variant_t) -> Self {
        unsafe {
            if !v.bool_value.is_null() {
                Self::bool(*v.bool_value)
            } else if !v.integer_value.is_null() {
                Self::integer(*v.integer_value)
            } else if !v.double_value.is_null() {
                Self::double(*v.double_value)
            } else if !v.string_value.is_null() {
                Self::string(String::from_c_char(v.string_value).unwrap_or_default())
            } else if !v.byte_array_value.is_null() {
                Self::byte_array(ptr_to_vec(
                    (*v.byte_array_value).values,
                    (*v.byte_array_value).size,
                ))
            } else if !v.bool_array_value.is_null() {
                Self::bool_array(ptr_to_vec(
                    (*v.bool_array_value).values,
                    (*v.bool_array_value).size,
                ))
            } else if !v.integer_array_value.is_null() {
                Self::integer_array(ptr_to_vec(
                    (*v.integer_array_value).values,
                    (*v.integer_array_value).size,
                ))
            } else if !v.double_array_value.is_null() {
                Self::double_array(ptr_to_vec(
                    (*v.double_array_value).values,
                    (*v.double_array_value).size,
                ))
            } else if !v.string_array_value.is_null() {
                Self::string_array(
                    std::slice::from_raw_parts(
                        (*v.string_array_value).data,
                        (*v.string_array_value).size,
                    )
                    .iter()
                    .map(|ptr| String::from_c_char(*ptr).unwrap_or_default())
                    .collect(),
                )
            } else {
                Self::not_set()
            }
        }
    }
}

fn ptr_to_vec<T: Clone>(data: *const T, len: usize) -> Vec<T> {
    unsafe { std::slice::from_raw_parts(data, len) }.to_vec()
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn not_set_value() {
        assert!(ParameterValue::not_set().get_value().is_none());
    }

    #[test]
    fn bool_parameter() {
        let v = true;
        assert_eq!(
            ParameterValue::bool(v).get_value().unwrap(),
            Variant::Bool(v)
        );
    }

    #[test]
    fn integer_parameter() {
        let v = 42;
        assert_eq!(
            ParameterValue::integer(v).get_value().unwrap(),
            Variant::Integer(v)
        );
    }

    #[test]
    fn double_parameter() {
        let v = 23.4378;
        assert_eq!(
            ParameterValue::double(v).get_value().unwrap(),
            Variant::Double(v)
        );
    }

    #[test]
    fn string_parameter() {
        let v = "hello world".to_string();
        assert_eq!(
            ParameterValue::string(v.clone()).get_value().unwrap(),
            Variant::String(v)
        );
    }

    #[test]
    fn byte_array_parameter() {
        let v = vec![2, 3, 42];
        assert_eq!(
            ParameterValue::byte_array(v.clone()).get_value().unwrap(),
            Variant::ByteArray(v)
        );
    }

    #[test]
    fn bool_array_parameter() {
        let v = vec![true, false, true, true];
        assert_eq!(
            ParameterValue::bool_array(v.clone()).get_value().unwrap(),
            Variant::BoolArray(v)
        );
    }

    #[test]
    fn integer_array_parameter() {
        let v = vec![38, -3848, 3984];
        assert_eq!(
            ParameterValue::integer_array(v.clone())
                .get_value()
                .unwrap(),
            Variant::IntegerArray(v)
        );
    }

    #[test]
    fn double_array_parameter() {
        let v = vec![3.38, -3.848, 398.4];
        assert_eq!(
            ParameterValue::double_array(v.clone()).get_value().unwrap(),
            Variant::DoubleArray(v)
        );
    }

    #[test]
    fn string_array_parameter() {
        let v = vec!["hoge".into(), "huga".into()];
        assert_eq!(
            ParameterValue::string_array(v.clone()).get_value().unwrap(),
            Variant::StringArray(v)
        );
    }
}
