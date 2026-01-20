mod client;
mod protocol;
mod server;
mod syntax;

// dora-schema-macro/src/lib.rs
use proc_macro::TokenStream;
use quote::quote;

use crate::syntax::{AttributeArgs, SchemaInput};

/// Defines a protocol schema, generating the protocol enums, client, and server trait.
///
/// ```ignore
/// use dora_schema_macro::dora_schema;
///
/// #[dora_schema]
/// pub trait MyProtocol {
///     fn foo(foo: FooRequest) -> FooResponse;
///     fn bar(bar: BarRequest) -> BarResponse;
/// }
/// ```
///
/// This will generate:
/// - Protocol enums `MyProtocolRequest` and `MyProtocolResponse`
/// - A client struct `MyProtocolClient` with methods `foo` and `bar`
/// - A server trait `MyProtocol` with handler methods for `foo` and `bar`
///
/// For example, the above trait will generate the following protocol enums:
///
/// ```ignore
/// pub enum MyProtocolRequest {
///     Foo{ foo: FooRequest },
///     Bar{ bar: BarRequest },
/// }
///
/// pub enum MyProtocolResponse {
///     Foo(FooResponse),
///     Bar(BarResponse),
/// }
/// ```
///
#[proc_macro_attribute]
pub fn dora_schema(attr: TokenStream, input: TokenStream) -> TokenStream {
    let mut schema = syn::parse_macro_input!(input as SchemaInput);
    let args = syn::parse_macro_input!(attr as AttributeArgs);

    schema.encoding = args.encoding;

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
