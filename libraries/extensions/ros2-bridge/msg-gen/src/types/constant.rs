use quote::{quote, ToTokens};

use super::{
    primitives::{BasicType, GenericUnboundedString, PrimitiveType},
    sequences::PrimitiveArray,
};

macro_rules! define_enum_from {
    ($into_t:ty, $from_t:ty, $path:path) => {
        impl From<$from_t> for $into_t {
            fn from(t: $from_t) -> Self {
                $path(t)
            }
        }
    };
}

/// A type which is available for constant
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConstantType {
    PrimitiveType(PrimitiveType),
    PrimitiveArray(PrimitiveArray),
}

impl ConstantType {
    pub fn type_tokens(&self) -> impl ToTokens {
        match self {
            Self::PrimitiveType(t) => {
                let token = t.type_tokens();
                quote! { #token }
            }
            Self::PrimitiveArray(t) => {
                let token = t.type_tokens();
                quote! { #token }
            }
        }
    }

    pub fn value_tokens(&self, values: &[String]) -> impl ToTokens {
        match self {
            Self::PrimitiveType(t) => {
                assert_eq!(values.len(), 1);
                let token = t.value_tokens(&values[0]);
                quote! { #token }
            }
            Self::PrimitiveArray(t) => {
                assert_eq!(values.len(), t.size);
                let tokens = values.iter().map(|v| t.value_type.value_tokens(v));
                quote! { [#(#tokens,)*] }
            }
        }
    }
}

define_enum_from!(ConstantType, PrimitiveType, Self::PrimitiveType);
define_enum_from!(ConstantType, PrimitiveArray, Self::PrimitiveArray);

impl From<BasicType> for ConstantType {
    fn from(t: BasicType) -> Self {
        Self::PrimitiveType(PrimitiveType::BasicType(t))
    }
}

impl From<GenericUnboundedString> for ConstantType {
    fn from(t: GenericUnboundedString) -> Self {
        Self::PrimitiveType(PrimitiveType::GenericUnboundedString(t))
    }
}
