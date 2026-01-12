use quote::{format_ident, quote};

use crate::SchemaInput;

pub fn request_enum_ident(schema: &SchemaInput) -> proc_macro2::Ident {
    format_ident!("{}Request", schema.item.ident)
}

pub fn response_enum_ident(schema: &SchemaInput) -> proc_macro2::Ident {
    format_ident!("{}Response", schema.item.ident)
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

    // TODO: better eyre conversion
    quote! {
        #[derive(::std::fmt::Debug, ::std::clone::Clone, ::serde::Serialize, ::serde::Deserialize)]
        pub enum #request_enum {
            #(#request_variants,)*
        }

        #[derive(::std::fmt::Debug, ::std::clone::Clone, ::serde::Serialize, ::serde::Deserialize)]
        pub enum #response_enum {
            #(#response_variants,)*
            Error(::std::string::String),
        }
    }
}
