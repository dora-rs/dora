use convert_case::{Case, Casing};
use proc_macro2::Ident;
use quote::{format_ident, quote};

use crate::{
    SchemaInput, protocol::{enum_variant_ident, error_struct_ident, request_enum_ident, response_enum_ident}
};

pub fn server_trait_ident(schema: &SchemaInput) -> Ident {
    format_ident!("{}Handler", schema.protocol_name())
}

pub fn handle_func_ident(schema: &SchemaInput) -> Ident {
    let snake_case_protocol_name =
        Casing::from_case(&schema.protocol_name(), Case::Pascal).to_case(Case::Snake);
    format_ident!("handle_{}", snake_case_protocol_name)
}

pub fn method_handler_ident(method: &crate::syntax::MethodDef) -> Ident {
    format_ident!("{}_handler", method.name)
}

pub fn generate_server_trait(schema: &SchemaInput) -> proc_macro2::TokenStream {
    let trait_name = server_trait_ident(schema);
    let request_enum = request_enum_ident(schema);
    let response_enum = response_enum_ident(schema);
    let error_type = error_struct_ident(schema);

    let trait_methods: Vec<_> = schema
        .methods
        .iter()
        .map(|m| {
            let handler_name = method_handler_ident(m);
            let request_type = &m.request;
            let response_type = &m.response;

            // TODO: better
            quote! {
                fn #handler_name(
                    &self,
                    request: #request_type
                ) -> ::std::pin::Pin<
                    Box<dyn ::std::future::Future<Output = ::std::result::Result<#response_type, #error_type>> + Send>
                >;
            }
        })
        .collect();

    let dispatch_arms = schema
        .methods
        .iter()
        .map(|m| {
            let handler_name = method_handler_ident(m);
            let method_variant = enum_variant_ident(m);
            quote! {
                #request_enum::#method_variant(request) => {
                    let response = handler.#handler_name(request).await;
                    match response {
                        Ok(resp) => #response_enum::#method_variant(resp),
                        Err(err) => #response_enum::Error(err),
                    }
                }
            }
        })
        .collect::<Vec<_>>();

    let handle_func_name = handle_func_ident(schema);

    // TODO: remove unwraps and handle errors properly
    let handle_func = quote! {
        pub async fn #handle_func_name(
            handler: impl #trait_name,
            request: ::std::vec::Vec<u8>
        ) -> ::std::vec::Vec<u8> {
            let request_enum: #request_enum = match ::serde_json::from_slice(&request) {
                Ok(req) => req,
                Err(err) => {
                    let error_response = #response_enum::Error(#error_type {
                        msg: format!("Failed to deserialize request: {}", err),
                    });
                    return ::serde_json::to_vec(&error_response).unwrap();
                }
            };

            let response_enum = match request_enum {
                #(#dispatch_arms)*
            };

            ::serde_json::to_vec(&response_enum).unwrap()
        }
    };

    quote! {
        pub trait #trait_name: std::marker::Send + std::marker::Sync {
            #(#trait_methods)*
        }

        #handle_func
    }
}
