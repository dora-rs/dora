mod client;
mod protocol;
mod server;
mod syntax;

// dora-schema-macro/src/lib.rs
use proc_macro::TokenStream;
use quote::quote;

use crate::syntax::SchemaInput;

/// Defines a protocol schema, generating the protocol enums, client, and server trait.
///
/// ```ignore
/// use dora_schema_macro::dora_schema;
/// dora_schema! {
///     MyClient => MyServer:
///     foo: FooRequest => FooResponse;
///     bar: BarRequest => BarResponse;
/// }
/// ```
///
/// This will generate:
/// - Protocol enums `MyProtocolRequest` and `MyProtocolResponse`
/// - A client struct `MyClient` with methods `foo` and `bar`
/// - A server trait `MyHandler` with handler methods for `foo` and `bar
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
