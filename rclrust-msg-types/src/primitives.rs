use std::fmt;

use crate::define_enum_from;

/// A basic type according to the IDL specification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BasicType {
    // signed integer type
    /// Rust: [i8], C++: `int8_t`
    I8,
    /// Rust: [i16], C++: `int16_t`
    I16,
    /// Rust: [i32], C++: `int32_t`
    I32,
    /// Rust: [i64], C++: `int64_t`
    I64,

    // unsigned integer type
    /// Rust: [u8], C++: `uint8_t`
    U8,
    /// Rust: [u16], C++: `uint16_t`
    U16,
    /// Rust: [u32], C++: `uint32_t`
    U32,
    /// Rust: [u64], C++: `uint64_t`
    U64,

    // floating point type
    /// Rust: [f32], C++: `float`
    F32,
    /// Rust: [f64], C++: `double`
    F64,
    // long double is not supported

    // boolean type
    /// Rust: [bool], C++: `bool`
    Bool,

    // duplicated type
    /// Rust: [u8], C++: `unsigned char`
    Char,
    /// Rust: [u8], C++: `unsigned char`
    Byte,
}

impl BasicType {
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "int8" => Self::I8,
            "int16" => Self::I16,
            "int32" => Self::I32,
            "int64" => Self::I64,
            "uint8" => Self::U8,
            "uint16" => Self::U16,
            "uint32" => Self::U32,
            "uint64" => Self::U64,
            "float32" => Self::F32,
            "float64" => Self::F64,
            "bool" => Self::Bool,
            "char" => Self::Char,
            "byte" => Self::Byte,
            _ => {
                return None;
            }
        })
    }

    pub const fn to_rust_str(self) -> &'static str {
        match self {
            Self::I8 => "i8",
            Self::I16 => "i16",
            Self::I32 => "i32",
            Self::I64 => "i64",
            Self::U8 | Self::Char | Self::Byte => "u8",
            Self::U16 => "u16",
            Self::U32 => "u32",
            Self::U64 => "u64",
            Self::F32 => "f32",
            Self::F64 => "f64",
            Self::Bool => "bool",
        }
    }
}

/// A type identified by the name
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NamedType(pub String);

impl fmt::Display for NamedType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// A type identified by a name in a namespaced scope
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NamespacedType {
    /// A package name which this type belongs to
    /// e.g. `std_msgs`
    pub package: String,
    /// msg or action
    pub namespace: String,
    /// A name of message
    /// e.g. `Bool`
    pub name: String,
}

impl NamespacedType {
    pub fn to_rust_str(&self) -> String {
        format!("{}::{}::{}", self.package, self.namespace, self.name)
    }
}

impl fmt::Display for NamespacedType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}/{}", self.package, self.namespace, self.name)
    }
}

/// A string type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GenericString {
    String,
    WString,
    BoundedString(usize),
    BoundedWString(usize),
}

impl GenericString {
    pub const fn is_wide(self) -> bool {
        matches!(self, Self::WString | Self::BoundedWString(_))
    }
}

impl From<GenericUnboundedString> for GenericString {
    fn from(t: GenericUnboundedString) -> Self {
        match t {
            GenericUnboundedString::String => Self::String,
            GenericUnboundedString::WString => Self::WString,
        }
    }
}

/// A generic unbounded string type
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GenericUnboundedString {
    String,
    WString,
}

/// A type which can be used inside nested types
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NestableType {
    BasicType(BasicType),
    NamedType(NamedType),
    NamespacedType(NamespacedType),
    GenericString(GenericString),
}

define_enum_from!(NestableType, BasicType, Self::BasicType);
define_enum_from!(NestableType, NamedType, Self::NamedType);
define_enum_from!(NestableType, NamespacedType, Self::NamespacedType);
define_enum_from!(NestableType, GenericString, Self::GenericString);

/// A primitive type which can be used for constant
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PrimitiveType {
    BasicType(BasicType),
    GenericUnboundedString(GenericUnboundedString),
}

define_enum_from!(PrimitiveType, BasicType, Self::BasicType);
define_enum_from!(
    PrimitiveType,
    GenericUnboundedString,
    Self::GenericUnboundedString
);
