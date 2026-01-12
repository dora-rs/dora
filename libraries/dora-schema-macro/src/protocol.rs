use convert_case::{Case, Casing};
use quote::{format_ident, quote};
use syn::TraitItem;

use crate::SchemaInput;

pub fn request_enum_ident(schema: &SchemaInput) -> proc_macro2::Ident {
    format_ident!("{}Request", schema.item.ident)
}

pub fn response_enum_ident(schema: &SchemaInput) -> proc_macro2::Ident {
    format_ident!("{}Response", schema.item.ident)
}

pub fn error_struct_ident(schema: &SchemaInput) -> proc_macro2::Ident {
    format_ident!("{}ErrorResponse", schema.item.ident)
}

/// Generates 2 enums representing the request and response messages
pub fn generate_protocol(schema: &SchemaInput) -> proc_macro2::TokenStream {
    let (request_variants, response_variants): (Vec<_>, Vec<_>) = schema
        .methods
        .iter()
        .map(|m| {
            let variant_name = m.variant_ident();
            let arguments = &m.arguments;
            let response = &m.response;
            (
                quote! {
                    #variant_name { #(#arguments),* }
                },
                quote! {
                    #variant_name(#response)
                },
            )
        })
        .unzip();

    let request_enum = request_enum_ident(schema);
    let response_enum = response_enum_ident(schema);
    let error_struct = error_struct_ident(schema);

    // TODO: better eyre conversion
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

        impl ::std::fmt::Display for #error_struct {
            fn fmt(&self, f: &mut ::std::fmt::Formatter<'_>) -> ::std::fmt::Result {
                write!(f, "{}", self.msg)
            }
        }

        impl ::std::error::Error for #error_struct {}

        impl ::std::convert::From<::eyre::Report> for #error_struct {
            fn from(report: ::eyre::Report) -> Self {
                Self { msg: format!("{}", report) }
            }
        }

        impl #error_struct {
            pub fn new(msg: String) -> Self {
                Self { msg }
            }
        }
    }
}
