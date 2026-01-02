mod client;
mod protocol;
mod server;
mod syntax;

// dora-schema-macro/src/lib.rs
use proc_macro::TokenStream;
use quote::quote;

use crate::syntax::SchemaInput;

#[proc_macro]
pub fn dora_schema(input: TokenStream) -> TokenStream {
    let schema = syn::parse_macro_input!(input as SchemaInput);

    let protocol_code = protocol::generate_protocol(&schema);
    let client_code = client::generate_client(&schema);
    let server_trait_code = server::generate_server_trait(&schema);

    let expanded = quote! {
        #protocol_code
        #client_code
        #server_trait_code
    };

    TokenStream::from(expanded)
}
