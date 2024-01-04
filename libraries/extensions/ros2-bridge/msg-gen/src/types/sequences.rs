use quote::{quote, ToTokens};

use super::primitives::*;

/// An array type with a static size
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Array {
    /// The type of the elements
    pub value_type: NestableType,
    /// The number of elements in the array
    pub size: usize,
}

impl Array {
    pub fn type_tokens(&self, package: &str) -> impl ToTokens {
        let inner_type = self.value_type.type_tokens(package);
        let size = self.size;
        quote! { [#inner_type; #size] }
    }

    pub fn raw_type_tokens(&self, package: &str) -> impl ToTokens {
        let inner_type = self.value_type.raw_type_tokens(package);
        let size = self.size;
        quote! { [#inner_type; #size] }
    }

    pub fn raw_ref_type_tokens(&self, package: &str) -> impl ToTokens {
        let inner_type = self.value_type.raw_ref_type_tokens(package);
        let size = self.size;
        quote! { [#inner_type; #size] }
    }
}

/// A sequence type with an unlimited number of elements
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sequence {
    /// The type of the elements
    pub value_type: NestableType,
}

impl Sequence {
    pub fn type_tokens(&self, package: &str) -> impl ToTokens {
        let inner_type = self.value_type.type_tokens(package);
        quote! { Vec<#inner_type> }
    }

    pub fn raw_type_tokens(&self, package: &str) -> impl ToTokens {
        let inner_type = self.value_type.raw_type_tokens(package);
        quote! { crate::_core::FFISeq<#inner_type> }
    }

    pub fn raw_ref_type_tokens(&self, package: &str) -> impl ToTokens {
        let inner_type = self.value_type.raw_ref_type_tokens(package);
        match self.value_type {
            NestableType::BasicType(_) => {
                quote! { crate::_core::RefFFISeq<#inner_type> }
            }
            _ => quote! { crate::_core::OwnedFFISeq<#inner_type> },
        }
    }
}

/// A sequence type with a maximum number of elements
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoundedSequence {
    /// The type of the elements
    pub value_type: NestableType,
    /// The maximum number of elements in the sequence
    pub max_size: usize,
}

impl BoundedSequence {
    pub fn type_tokens(&self, package: &str) -> impl ToTokens {
        let inner_type = self.value_type.type_tokens(package);
        quote! { Vec<#inner_type> }
    }

    pub fn raw_type_tokens(&self, package: &str) -> impl ToTokens {
        let inner_type = self.value_type.raw_type_tokens(package);
        quote! { crate::_core::FFISeq<#inner_type> }
    }

    pub fn raw_ref_type_tokens(&self, package: &str) -> impl ToTokens {
        let inner_type = self.value_type.raw_ref_type_tokens(package);
        match self.value_type {
            NestableType::BasicType(_) => {
                quote! { crate::_core::RefFFISeq<#inner_type> }
            }
            _ => quote! { crate::_core::OwnedFFISeq<#inner_type> },
        }
    }
}

/// An array type of a primitive type
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PrimitiveArray {
    /// The type of the elements
    pub value_type: PrimitiveType,
    /// The number of elements in the array
    pub size: usize,
}

impl PrimitiveArray {
    pub fn type_tokens(&self) -> impl ToTokens {
        let inner_type = self.value_type.type_tokens();
        let size = self.size;
        quote! { [#inner_type; #size] }
    }
}
