use quote::{quote, ToTokens};

use super::{primitives::*, sequences::*};

macro_rules! define_enum_from {
    ($into_t:ty, $from_t:ty, $path:path) => {
        impl From<$from_t> for $into_t {
            fn from(t: $from_t) -> Self {
                $path(t)
            }
        }
    };
}

/// A type which is available for member
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MemberType {
    NestableType(NestableType),
    Array(Array),
    Sequence(Sequence),
    BoundedSequence(BoundedSequence),
}

impl MemberType {
    pub fn type_tokens(&self, package: &str) -> (impl ToTokens, impl ToTokens) {
        match self {
            Self::NestableType(t) => {
                let token = t.type_tokens(package);
                (quote! {}, quote! { #token })
            }
            Self::Array(t) => {
                let token = t.type_tokens(package);
                (
                    quote! { #[serde(with = "serde_big_array::BigArray")] },
                    quote! { #token },
                )
            }
            Self::Sequence(t) => {
                let token = t.type_tokens(package);
                (quote! {}, quote! { #token })
            }
            Self::BoundedSequence(t) => {
                let token = t.type_tokens(package);
                (quote! {}, quote! { #token })
            }
        }
    }

    pub fn raw_type_tokens(&self, package: &str) -> impl ToTokens {
        match self {
            Self::NestableType(t) => {
                let token = t.raw_type_tokens(package);
                quote! { #token }
            }
            Self::Array(t) => {
                let token = t.raw_type_tokens(package);
                quote! { #token }
            }
            Self::Sequence(t) => {
                let token = t.raw_type_tokens(package);
                quote! { #token }
            }
            Self::BoundedSequence(t) => {
                let token = t.raw_type_tokens(package);
                quote! { #token }
            }
        }
    }

    pub fn raw_ref_type_tokens(&self, package: &str) -> impl ToTokens {
        match self {
            Self::NestableType(t) => {
                let token = t.raw_ref_type_tokens(package);
                quote! { #token }
            }
            Self::Array(t) => {
                let token = t.raw_ref_type_tokens(package);
                quote! { #token }
            }
            Self::Sequence(t) => {
                let token = t.raw_ref_type_tokens(package);
                quote! { #token }
            }
            Self::BoundedSequence(t) => {
                let token = t.raw_ref_type_tokens(package);
                quote! { #token }
            }
        }
    }

    pub fn value_tokens(&self, default: &[String]) -> impl ToTokens {
        match self {
            Self::NestableType(t) => {
                let token = t.value_tokens(&default[0]);
                quote! { #token }
            }
            Self::Array(t) => {
                assert_eq!(default.len(), t.size);
                let tokens = default.iter().map(|v| t.value_type.value_tokens(v));
                quote! { [#(#tokens,)*] }
            }
            Self::Sequence(t) => {
                let tokens = default.iter().map(|v| t.value_type.value_tokens(v));
                quote! { vec![#(#tokens,)*] }
            }
            Self::BoundedSequence(t) => {
                assert!(default.len() <= t.max_size);
                let tokens = default.iter().map(|v| t.value_type.value_tokens(v));
                quote! { vec![#(#tokens,)*] }
            }
        }
    }
}

define_enum_from!(MemberType, NestableType, Self::NestableType);
define_enum_from!(MemberType, Array, Self::Array);
define_enum_from!(MemberType, Sequence, Self::Sequence);
define_enum_from!(MemberType, BoundedSequence, Self::BoundedSequence);

impl From<BasicType> for MemberType {
    fn from(t: BasicType) -> Self {
        Self::NestableType(NestableType::BasicType(t))
    }
}

impl From<NamedType> for MemberType {
    fn from(t: NamedType) -> Self {
        Self::NestableType(NestableType::NamedType(t))
    }
}

impl From<NamespacedType> for MemberType {
    fn from(t: NamespacedType) -> Self {
        Self::NestableType(NestableType::NamespacedType(t))
    }
}

impl From<GenericString> for MemberType {
    fn from(t: GenericString) -> Self {
        Self::NestableType(NestableType::GenericString(t))
    }
}
