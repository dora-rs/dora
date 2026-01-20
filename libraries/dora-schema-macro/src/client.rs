use proc_macro2::Ident;
use quote::{format_ident, quote};

use crate::{
    SchemaInput,
    protocol::{protocol_ident, request_enum_ident, response_enum_ident},
};

fn sync_client(
    schema: &SchemaInput,
    request_enum_ident: &Ident,
    response_enum_ident: &Ident,
    protocol_ident: &Ident,
) -> proc_macro2::TokenStream {
    let client_struct = format_ident!("{}Client", schema.item.ident);
    let client_methods = schema
    .methods
        .iter()
        .map(|m| {
            let method_name = &m.sig.ident;
            let attributes = &m.attrs;
            let arguments = &m.arguments;
            let argument_pats = m.arguments.iter().map(|arg| &arg.pat);
            let response = &m.response;
            let variant = m.variant_ident();
            quote! {
                #(#attributes)*
                pub fn #method_name(
                    &mut self,
                    #(#arguments),*
                ) -> ::eyre::Result<#response> {
                    use ::communication_layer_request_reply::Transport;

                    let req_enum = #request_enum_ident::#variant { #(#argument_pats),* };

                    self.transport.send(&req_enum).map_err(|e| ::eyre::eyre!("Transport IO error while sending: {}", e))?;
                    let Some(resp_enum) = self.transport.receive().map_err(|e| ::eyre::eyre!("Transport IO error while receiving: {}", e))? else {
                        ::eyre::bail!("Response expected");
                    };

                    match resp_enum {
                        #response_enum_ident::#variant(resp) => Ok(resp),
                        #response_enum_ident::Error(err) => Err(::eyre::eyre!("Server returned error: {err}")),
                        _ => ::eyre::bail!("Unexpected response type"),
                    }
                }
            }
        })
        .collect::<Vec<_>>();

    let transport = quote! {
        ::communication_layer_request_reply::Transport<
            #request_enum_ident,
            #response_enum_ident
        >
    };

    quote! {
        pub struct #client_struct {
            transport: ::std::boxed::Box<dyn #transport>,
        }

        impl #client_struct {
            #(#client_methods)*

            pub fn new(transport: impl #transport + 'static) -> Self {
                Self {
                    transport: ::std::boxed::Box::new(transport),
                }
            }

            pub fn from_io<IO>(io: IO) -> Self
            where
                IO: ::std::io::Read + ::std::io::Write + 'static,
            {
                use ::communication_layer_request_reply::Protocol;
                let transport = #protocol_ident::connect(io);
                Self::new(transport)
            }

            pub fn transport_mut(&mut self) -> &mut dyn #transport {
                self.transport.as_mut()
            }
            pub fn into_inner(self) -> ::std::boxed::Box<dyn #transport> {
                self.transport
            }
        }

        impl<T> From<T> for #client_struct
        where
            T: #transport + 'static,
        {
            fn from(transport: T) -> Self {
                Self::new(transport)
            }
        }
    }
}

fn async_client(
    schema: &SchemaInput,
    request_enum_ident: &Ident,
    response_enum_ident: &Ident,
    protocol_ident: &Ident,
) -> proc_macro2::TokenStream {
    let client_struct = format_ident!("{}AsyncClient", schema.item.ident);
    let client_methods = schema
        .methods
        .iter()
        .map(|m| {
            let method_name = &m.sig.ident;
            let attributes = &m.attrs;
            let arguments = &m.arguments;
            let argument_pats = m.arguments.iter().map(|arg| &arg.pat);
            let response = &m.response;
            let variant = m.variant_ident();
            quote! {
                #(#attributes)*
                pub async fn #method_name(
                    &mut self,
                    #(#arguments),*
                ) -> ::eyre::Result<#response> {
                    use ::communication_layer_request_reply::AsyncTransport;

                    let req_enum = #request_enum_ident::#variant { #(#argument_pats),* };

                    self.transport.send(&req_enum).await.map_err(|e| ::eyre::eyre!("Transport IO error while sending: {}", e))?;
                    let Some(resp_enum) = self.transport.receive().await.map_err(|e| ::eyre::eyre!("Transport IO error while receiving: {}", e))? else {
                        ::eyre::bail!("Response expected");
                    };

                    match resp_enum {
                        #response_enum_ident::#variant(resp) => Ok(resp),
                        #response_enum_ident::Error(err) => Err(::eyre::eyre!("Server returned error: {err}")),
                        _ => ::eyre::bail!("Unexpected response type"),
                    }
                }
            }
        })
        .collect::<Vec<_>>();

    let transport = quote! {
        ::communication_layer_request_reply::AsyncTransport<
            #request_enum_ident,
            #response_enum_ident
        >
    };

    quote! {
        pub struct #client_struct {
            transport: ::std::boxed::Box<dyn #transport>,
        }

        impl #client_struct {
            #(#client_methods)*

            pub fn new(transport: impl #transport + 'static) -> Self {
                Self {
                    transport: ::std::boxed::Box::new(transport),
                }
            }

            pub fn from_io<IO>(io: IO) -> Self
            where
                IO: ::tokio::io::AsyncRead
                    + ::tokio::io::AsyncWrite
                    + ::std::marker::Unpin
                    + ::std::marker::Send
                    + 'static,
            {
                use ::communication_layer_request_reply::Protocol;
                let transport = #protocol_ident::connect_async(io);
                Self::new(transport)
            }

            pub fn transport_mut(&mut self) -> &mut dyn #transport {
                self.transport.as_mut()
            }
            pub fn into_inner(self) -> ::std::boxed::Box<dyn #transport> {
                self.transport
            }
        }

        impl<T> From<T> for #client_struct
        where
            T: #transport + 'static,
        {
            fn from(transport: T) -> Self {
                Self::new(transport)
            }
        }
    }
}

pub fn generate_client(schema: &SchemaInput) -> proc_macro2::TokenStream {
    let request_enum = request_enum_ident(schema);
    let response_enum = response_enum_ident(schema);
    let protocol_ident = protocol_ident(schema);

    let sync_client_code = sync_client(schema, &request_enum, &response_enum, &protocol_ident);
    let async_client_code = async_client(schema, &request_enum, &response_enum, &protocol_ident);

    quote! {
        #sync_client_code
        #async_client_code
    }
}
