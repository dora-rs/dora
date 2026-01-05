use quote::{format_ident, quote};

use crate::{
    protocol::{enum_variant_ident, request_enum_ident, response_enum_ident},
    SchemaInput,
};

fn sync_client(
    schema: &SchemaInput,
    request_enum_ident: &proc_macro2::Ident,
    response_enum_ident: &proc_macro2::Ident,
) -> proc_macro2::TokenStream {
    let client_struct = format_ident!("{}Client", schema.protocol_name());
    let client_methods = schema
        .methods
        .iter()
        .map(|m| {
            let method_name = &m.name;
            let attributes = &m.attrs;
            let request_type = &m.request;
            let response_type = &m.response;
            let variant = enum_variant_ident(m);
            quote! {
                #(#attributes)*
                pub fn #method_name(
                    &mut self,
                    request: #request_type
                ) -> ::eyre::Result<#response_type> {
                    use ::communication_layer_request_reply::Transport;

                    let req_enum = #request_enum_ident::#variant(request);

                    self.transport.send(&req_enum).map_err(|e| ::eyre::eyre!("Transport IO error while sending: {}", e))?;
                    let Some(resp_enum) = self.transport.receive().map_err(|e| ::eyre::eyre!("Transport IO error while receiving: {}", e))? else {
                        ::eyre::bail!("Response expected");
                    };

                    match resp_enum {
                        #response_enum_ident::#variant(resp) => Ok(resp),
                        #response_enum_ident::Error(err) => Err(::eyre::eyre!("Server returned error: {}", err.msg)),
                        _ => ::eyre::bail!("Unexpected response type"),
                    }
                }
            }
        })
        .collect::<Vec<_>>();

    quote! {
        pub struct #client_struct {
            transport: ::communication_layer_request_reply::encoding::EncodedTransport::<
                ::communication_layer_request_reply::transport::FramedTransport<std::net::TcpStream>,
                ::communication_layer_request_reply::encoding::JsonEncoding,
                #request_enum_ident,
                #response_enum_ident,
            >,
        }

        impl #client_struct {
            #(#client_methods)*
        }

        impl #client_struct {
            /// `JSON | u64-length-delimited(framed) | tcp`
            pub fn new_tcp(addr: std::net::SocketAddr) -> std::io::Result<Self> {
                let transport = ::communication_layer_request_reply::transport::FramedTransport::new(std::net::TcpStream::connect(addr)?);
                let transport = ::communication_layer_request_reply::Transport::with_encoding::<
                    _,
                    #request_enum_ident,
                    #response_enum_ident,
                >(
                    transport,
                    ::communication_layer_request_reply::encoding::JsonEncoding,
                );
                Ok(Self { transport })
            }
        }
    }
}

fn async_client(
    schema: &SchemaInput,
    request_enum_ident: &proc_macro2::Ident,
    response_enum_ident: &proc_macro2::Ident,
) -> proc_macro2::TokenStream {
    let client_struct = format_ident!("Async{}Client", schema.protocol_name());
    quote! {
        pub struct #client_struct<T: ::communication_layer_request_reply::Transport<#request_enum_ident, #response_enum_ident>> {
            transport: T,
        }
    }
}

pub fn generate_client(schema: &SchemaInput) -> proc_macro2::TokenStream {
    let request_enum = request_enum_ident(schema);
    let response_enum = response_enum_ident(schema);

    let sync_client_code = sync_client(schema, &request_enum, &response_enum);
    let async_client_code = async_client(schema, &request_enum, &response_enum);

    quote! {
        #sync_client_code
        #async_client_code
    }
}
