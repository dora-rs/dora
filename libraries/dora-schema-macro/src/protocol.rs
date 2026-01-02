use convert_case::{Case, Casing};
use quote::{format_ident, quote};

use crate::{syntax::MethodDef, SchemaInput};

pub fn request_enum_ident(schema: &SchemaInput) -> proc_macro2::Ident {
    format_ident!("{}Request", schema.protocol_name())
}

pub fn response_enum_ident(schema: &SchemaInput) -> proc_macro2::Ident {
    format_ident!("{}Response", schema.protocol_name())
}

pub fn error_struct_ident(schema: &SchemaInput) -> proc_macro2::Ident {
    format_ident!("{}ErrorResponse", schema.protocol_name())
}

pub fn enum_variant_ident(method: &MethodDef) -> proc_macro2::Ident {
    let variant_name =
        Casing::from_case(&method.name.to_string(), Case::Snake).to_case(Case::Pascal);
    format_ident!("{}", variant_name)
}

/// Generates 2 enums representing the request and response messages
pub fn generate_protocol(schema: &SchemaInput) -> proc_macro2::TokenStream {
    let request_variants: Vec<_> = schema
        .methods
        .iter()
        .map(|m| {
            let variant_name = enum_variant_ident(m);
            let req_type = &m.request;
            quote! {
                #variant_name(#req_type)
            }
        })
        .collect();

    let response_variants: Vec<_> = schema
        .methods
        .iter()
        .map(|m| {
            let variant_name = enum_variant_ident(m);
            let resp_type = &m.response;
            quote! {
                #variant_name(#resp_type)
            }
        })
        .collect();

    let request_enum = request_enum_ident(schema);
    let response_enum = response_enum_ident(schema);
    let error_struct = error_struct_ident(schema);

    quote! {
        #[derive(Debug, ::serde::Serialize, ::serde::Deserialize)]
        pub enum #request_enum {
            #(#request_variants,)*
        }

        #[derive(Debug, ::serde::Serialize, ::serde::Deserialize)]
        pub enum #response_enum {
            #(#response_variants,)*
            Error(#error_struct),
        }

        #[derive(Debug, ::serde::Serialize, ::serde::Deserialize)]
        pub struct #error_struct {
            pub msg: String,
        }

        impl #error_struct {
            pub fn new(msg: String) -> Self {
                Self { msg }
            }
        }
    }
}
