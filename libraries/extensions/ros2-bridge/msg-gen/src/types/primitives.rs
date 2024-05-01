use std::fmt;

use proc_macro2::{Ident, Literal, Span};
use quote::{format_ident, quote, ToTokens};

macro_rules! define_enum_from {
    ($into_t:ty, $from_t:ty, $path:path) => {
        impl From<$from_t> for $into_t {
            fn from(t: $from_t) -> Self {
                $path(t)
            }
        }
    };
}

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

    pub fn type_tokens(self) -> impl ToTokens {
        match self {
            Self::I8 => quote! { i8 },
            Self::I16 => quote! { i16 },
            Self::I32 => quote! { i32 },
            Self::I64 => quote! { i64 },
            Self::U8 | Self::Char | Self::Byte => quote! { u8 },
            Self::U16 => quote! { u16 },
            Self::U32 => quote! { u32 },
            Self::U64 => quote! { u64 },
            Self::F32 => quote! { f32 },
            Self::F64 => quote! { f64 },
            Self::Bool => quote! { bool },
        }
    }

    fn value_literal(self, value: &str) -> Option<Literal> {
        Some(match self {
            Self::I8 => Literal::i8_suffixed(value.parse().unwrap()),
            Self::I16 => Literal::i16_suffixed(value.parse().unwrap()),
            Self::I32 => Literal::i32_suffixed(value.parse().unwrap()),
            Self::I64 => Literal::i64_suffixed(value.parse().unwrap()),
            Self::U8 | Self::Char | Self::Byte => Literal::u8_suffixed(value.parse().unwrap()),
            Self::U16 => Literal::u16_suffixed(value.parse().unwrap()),
            Self::U32 => Literal::u32_suffixed(value.parse().unwrap()),
            Self::U64 => Literal::u64_suffixed(value.parse().unwrap()),
            Self::F32 => Literal::f32_suffixed(value.parse().unwrap()),
            Self::F64 => Literal::f64_suffixed(value.parse().unwrap()),
            // bool is Ident not Literal!
            Self::Bool => return None,
        })
    }

    fn value_tokens(self, value: &str) -> impl ToTokens {
        match self {
            Self::Bool => match value {
                "true" => quote! { true },
                "false" => quote! { false },
                _ => unreachable!(),
            },
            _ => {
                let value = self.value_literal(value).unwrap();
                quote! { #value }
            }
        }
    }
}

/// A type identified by the name
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NamedType(pub String);

impl NamedType {
    fn type_tokens(&self, package: &str) -> impl ToTokens {
        let package = Ident::new(package, Span::call_site());
        let name = Ident::new(&self.0, Span::call_site());
        let ident = format_ident!("{package}__{name}");
        quote! { #ident }
    }

    fn raw_type_tokens(&self, package: &str) -> impl ToTokens {
        let package = Ident::new(package, Span::call_site());
        let namespace = Ident::new("msg", Span::call_site());
        let name = format_ident!("{}_Raw", self.0);
        quote! { crate::#package::#namespace::#name }
    }

    fn raw_ref_type_tokens(&self, package: &str) -> impl ToTokens {
        let package = Ident::new(package, Span::call_site());
        let namespace = Ident::new("msg", Span::call_site());
        let name = format_ident!("{}_RawRef", self.0);
        quote! { crate::#package::#namespace::#name }
    }
}

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
    fn type_tokens(&self) -> impl ToTokens {
        let package = Ident::new(&self.package, Span::call_site());
        let name = Ident::new(&self.name, Span::call_site());
        let ident = format_ident!("{package}__{name}");
        quote! { #ident }
    }

    fn raw_type_tokens(&self) -> impl ToTokens {
        let package = Ident::new(&self.package, Span::call_site());
        let namespace = Ident::new(&self.namespace, Span::call_site());
        let name = format_ident!("{}_Raw", self.name);
        quote! { crate::#package::#namespace::#name }
    }

    fn raw_ref_type_tokens(&self) -> impl ToTokens {
        let package = Ident::new(&self.package, Span::call_site());
        let namespace = Ident::new(&self.namespace, Span::call_site());
        let name = format_ident!("{}_RawRef", self.name);
        quote! { crate::#package::#namespace::#name }
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
    const fn is_wide(self) -> bool {
        matches!(self, Self::WString | Self::BoundedWString(_))
    }

    fn type_tokens(self) -> impl ToTokens {
        if self.is_wide() {
            quote! { U16String }
        } else {
            quote! { String }
        }
    }

    fn raw_type_tokens(self) -> impl ToTokens {
        if self.is_wide() {
            quote! { crate::_core::FFIWString }
        } else {
            quote! { crate::_core::FFIString }
        }
    }

    fn raw_ref_type_tokens(self) -> impl ToTokens {
        if self.is_wide() {
            quote! { crate::_core::OwnedFFIWString }
        } else {
            quote! { crate::_core::OwnedFFIString }
        }
    }

    fn value_tokens(self, value: &str) -> impl ToTokens {
        // TODO: Assertion
        let value = Literal::string(value);
        if self.is_wide() {
            quote! { ffi::U16String::from_str(#value) }
        } else {
            quote! { ::std::string::String::from(#value) }
        }
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GenericUnboundedString {
    String,
    WString,
}

impl GenericUnboundedString {
    fn type_tokens(self) -> impl ToTokens {
        quote! { &'static str }
    }

    fn value_tokens(self, value: &str) -> impl ToTokens {
        let value = Literal::string(value);
        quote! { #value }
    }
}

/// A type which can be used inside nested types
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NestableType {
    BasicType(BasicType),
    NamedType(NamedType),
    NamespacedType(NamespacedType),
    GenericString(GenericString),
}

impl NestableType {
    pub fn type_tokens(&self, package: &str) -> impl ToTokens {
        match self {
            Self::BasicType(t) => {
                let token = t.type_tokens();
                quote! { #token }
            }
            Self::NamedType(t) => {
                let token = t.type_tokens(package);
                quote! { #token }
            }
            Self::NamespacedType(t) => {
                let token = t.type_tokens();
                quote! { #token }
            }
            Self::GenericString(t) => {
                let token = t.type_tokens();
                quote! { #token }
            }
        }
    }

    pub fn raw_type_tokens(&self, package: &str) -> impl ToTokens {
        match self {
            Self::BasicType(t) => {
                let token = t.type_tokens();
                quote! { #token }
            }
            Self::NamedType(t) => {
                let token = t.raw_type_tokens(package);
                quote! { #token }
            }
            Self::NamespacedType(t) => {
                let token = t.raw_type_tokens();
                quote! { #token }
            }
            Self::GenericString(t) => {
                let token = t.raw_type_tokens();
                quote! { #token }
            }
        }
    }

    pub fn raw_ref_type_tokens(&self, package: &str) -> impl ToTokens {
        match self {
            Self::BasicType(t) => {
                let token = t.type_tokens();
                quote! { #token }
            }
            Self::NamedType(t) => {
                let token = t.raw_ref_type_tokens(package);
                quote! { #token }
            }
            Self::NamespacedType(t) => {
                let token = t.raw_ref_type_tokens();
                quote! { #token }
            }
            Self::GenericString(t) => {
                let token = t.raw_ref_type_tokens();
                quote! { #token }
            }
        }
    }

    pub fn value_tokens(&self, default: &str) -> impl ToTokens {
        match self {
            Self::BasicType(t) => {
                let token = t.value_tokens(default);
                quote! { #token }
            }
            Self::GenericString(t) => {
                let token = t.value_tokens(default);
                quote! { #token }
            }
            _ => unreachable!(),
        }
    }
}

define_enum_from!(NestableType, BasicType, Self::BasicType);
define_enum_from!(NestableType, NamedType, Self::NamedType);
define_enum_from!(NestableType, NamespacedType, Self::NamespacedType);
define_enum_from!(NestableType, GenericString, Self::GenericString);

/// A primitive type which can be used for constant
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrimitiveType {
    BasicType(BasicType),
    GenericUnboundedString(GenericUnboundedString),
}

impl PrimitiveType {
    pub fn type_tokens(self) -> impl ToTokens {
        match self {
            Self::BasicType(t) => {
                let token = t.type_tokens();
                quote! { #token }
            }
            Self::GenericUnboundedString(t) => {
                let token = t.type_tokens();
                quote! { #token }
            }
        }
    }

    pub fn value_tokens(self, value: &str) -> impl ToTokens {
        match self {
            Self::BasicType(t) => {
                let token = t.value_tokens(value);
                quote! { #token }
            }
            Self::GenericUnboundedString(t) => {
                let token = t.value_tokens(value);
                quote! { #token }
            }
        }
    }
}

define_enum_from!(PrimitiveType, BasicType, Self::BasicType);
define_enum_from!(
    PrimitiveType,
    GenericUnboundedString,
    Self::GenericUnboundedString
);
