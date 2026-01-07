use proc_macro2::Ident;
use quote::{format_ident, quote};

use crate::{
    protocol::{enum_variant_ident, error_struct_ident, request_enum_ident, response_enum_ident},
    SchemaInput,
};

pub fn server_trait_ident(schema: &SchemaInput) -> Ident {
    format_ident!("{}Handler", schema.protocol_name())
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

            quote! {
                async fn #handler_name(self, request: #request_type) -> ::std::result::Result<#response_type, #error_type>;
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
                    let response = self.#handler_name(request).await;
                    match response {
                        Ok(resp) => #response_enum::#method_variant(resp),
                        Err(err) => #response_enum::Error(err),
                    }
                }
            }
        })
        .collect::<Vec<_>>();

    let handle_function = quote! {
        async fn handle<T>(self, mut transport: T) -> ::eyre::Result<()>
        where
            T: ::communication_layer_request_reply::AsyncTransport<#response_enum, #request_enum> + ::std::marker::Send + ::std::marker::Sync,
        {
            use ::communication_layer_request_reply::AsyncTransport;
            // TODO: handle transprt errors (e.g. serde errors) and return error responses instead of exiting
            if let Some(req) = transport.receive().await? {
                let resp = match req {
                    #(#dispatch_arms)*
                };
                transport.send(&resp).await?;
            } else {
                ::eyre::bail!("Transport closed while waiting for request");
            }
            Ok(())
        }
    };

    quote! {
        #[::async_trait::async_trait]
        /// Note: the `serve` function will clone the handler for each request, use `Arc<Mutex<State>>` to share state between requests.
        pub trait #trait_name: ::std::marker::Sized {
            #(#trait_methods)*

            #handle_function
        }
    }
}
