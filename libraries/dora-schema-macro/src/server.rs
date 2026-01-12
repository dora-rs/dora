use quote::quote;

use crate::{
    SchemaInput,
    protocol::{request_enum_ident, response_enum_ident},
};

pub fn generate_server_trait(schema: &SchemaInput) -> proc_macro2::TokenStream {
    let trait_name = &schema.item.ident;
    let request_enum = request_enum_ident(schema);
    let response_enum = response_enum_ident(schema);

    let trait_methods: Vec<_> = schema
        .methods
        .iter()
        .map(|m| {
            let asyncness = &m.sig.asyncness;
            let ident = &m.sig.ident;
            let arguments = &m.arguments;
            let response_type = &m.response;
            quote! {
                #asyncness fn #ident(self, #(#arguments),*) -> ::eyre::Result<#response_type>;
            }
        })
        .collect();

    let dispatch_arms = schema
        .methods
        .iter()
        .map(|m| {
            let asyncness = &m.sig.asyncness.as_ref().map(|_| quote! { .await });
            let ident = &m.sig.ident;
            let argument_pats = m.arguments.iter().map(|arg| &arg.pat).collect::<Vec<_>>();
            let method_variant = m.variant_ident();
            quote! {
                #request_enum::#method_variant { #(#argument_pats),* } => {
                    let response = self.#ident(#(#argument_pats),*) #asyncness;
                    match response {
                        Ok(resp) => #response_enum::#method_variant(resp),
                        Err(err) => #response_enum::Error(err.to_string()),
                    }
                }
            }
        })
        .collect::<Vec<_>>();

    let handle_function = quote! {
        /// Handle a single request, returning a response.
        ///
        /// Do the dispatching by default, override this method to implement custom dispatching logic.
        ///
        async fn handle(self, request: #request_enum) -> ::eyre::Result<#response_enum>
        {
            Ok(match request {
                #(#dispatch_arms)*
            })
        }
    };

    quote! {
        #[::async_trait::async_trait]
        /// Note: the `serve` function will clone the handler for each request, use `Arc<Mutex<State>>` to share state between requests.
        pub trait #trait_name: ::std::marker::Sized + ::std::marker::Send {
            #(#trait_methods)*

            #handle_function
        }
    }
}
