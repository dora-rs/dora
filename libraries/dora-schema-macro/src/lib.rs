mod client;
mod protocol;
mod server;
mod syntax;

// dora-schema-macro/src/lib.rs
use proc_macro::TokenStream;
use quote::{format_ident, quote};

use crate::syntax::SchemaInput;

#[proc_macro]
pub fn dora_schema(input: TokenStream) -> TokenStream {
    let schema = syn::parse_macro_input!(input as SchemaInput);

    let protocol_code = protocol::generate_protocol(&schema);
    let client_code = client::generate_client(&schema);
    // let server_trait_code = server::generate_server_trait(&schema);
    let server_trait_code = quote! {};

    let expanded = quote! {
        #protocol_code
        #client_code
        #server_trait_code
    };

    TokenStream::from(expanded)
}

fn generate_server(schema: &SchemaInput) -> proc_macro2::TokenStream {
    let client_name = &schema.client_name;
    let server_name = &schema.server_name;
    let handler_trait_name = format_ident!("{}Handler", server_name);
    let request_enum = format_ident!("{}To{}Request", client_name, server_name);
    let response_enum = format_ident!("{}To{}Response", client_name, server_name);

    let trait_methods: Vec<_> = schema
        .methods
        .iter()
        .map(|m| {
            let handler_name = format_ident!("{}_handler", m.name);
            let request_type = &m.request;
            let response_type = &m.response;

            // TODO: better
            quote! {
                fn #handler_name(
                    &self,
                    request: #request_type
                ) -> std::pin::Pin<
                    Box<dyn std::future::Future<Output = ::eyre::Result<#response_type>> + Send>
                >;
            }
        })
        .collect();

    let dispatch_arms: Vec<_> = schema
        .methods
        .iter()
        .map(|m| {
            let handler_name = format_ident!("{}_handler", m.name);
            let request_variant = format_ident!("{}", capitalize(&m.name.to_string()));
            let response_variant = request_variant.clone();

            quote! {
                #request_enum::#request_variant(req) => {
                    match self.#handler_name(req).await {
                        Ok(resp) => {
                            let resp_enum = #response_enum::#response_variant(resp);
                            ::bincode::serialize(&resp_enum)
                                .wrap_err("Failed to serialize response")
                        }
                        Err(e) => {
                            ::eyre::bail!("Handler error: {:?}", e)
                        }
                    }
                }
            }
        })
        .collect();

    quote! {
        pub trait #handler_trait_name: Send + Sync {
            #(#trait_methods)*
        }

        pub struct #server_name<H, L> {
            handler: H,
            listener: L,
        }

        impl<H, L> #server_name<H, L>
        where
            H: #handler_trait_name + 'static,
            L: ListenConnection,
        {
            pub fn new(handler: H, listener: L) -> Self {
                Self { handler, listener }
            }

            pub async fn serve(self) -> ::eyre::Result<()> {
                loop {
                    let req_bytes = self.listener.receive()
                        .await
                        .wrap_err("Failed to receive request")?;

                    let request: #request_enum = match ::bincode::deserialize(&req_bytes) {
                        Ok(req) => req,
                        Err(e) => {
                            eprintln!("Failed to deserialize request: {:?}", e);
                            continue;
                        }
                    };

                    let response_bytes = match request {
                        #(#dispatch_arms,)*
                    };

                    match response_bytes {
                        Ok(bytes) => {
                            if let Err(e) = self.listener.send(bytes).await {
                                eprintln!("Failed to send response: {:?}", e);
                            }
                        }
                        Err(e) => {
                            eprintln!("Handler error: {:?}", e);
                        }
                    }
                }
            }
        }
    }
}

fn capitalize(s: &str) -> String {
    let mut chars = s.chars();
    match chars.next() {
        Some(first) => first.to_uppercase().chain(chars).collect(),
        None => String::new(),
    }
}
