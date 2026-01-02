use quote::{format_ident, quote};

use crate::{
    protocol::{enum_variant_ident, request_enum_ident, response_enum_ident},
    SchemaInput,
};

pub fn generate_client(schema: &SchemaInput) -> proc_macro2::TokenStream {
    let client_struct = format_ident!("{}Client", schema.protocol_name());
    let request_enum = request_enum_ident(schema);
    let response_enum = response_enum_ident(schema);

    let client_methods = schema
        .methods
        .iter()
        .map(|m| {
            let method_name = &m.name;
            let request_type = &m.request;
            let response_type = &m.response;
            let variant = enum_variant_ident(m);

            // TODO: consider adding an async version, may require communication layer support
            quote! {
                pub fn #method_name(
                    &mut self,
                    request: #request_type
                ) -> ::eyre::Result<#response_type> {
                    let req_enum = #request_enum::#variant(request);
                    let req_bytes = ::serde_json::to_vec(&req_enum);
                    let req_bytes = ::eyre::WrapErr::wrap_err(req_bytes, "Failed to serialize request")?;

                    let resp = self.connection.request(&req_bytes);
                    let resp_bytes = resp.map_err(|e| ::eyre::eyre!("Request failed: {}", e))?;

                    let resp_enum = ::serde_json::from_slice(&resp_bytes);
                    let resp_enum: #response_enum = ::eyre::WrapErr::wrap_err(resp_enum, "Failed to parse response")?;

                    match resp_enum {
                        #response_enum::#variant(resp) => Ok(resp),
                        _ => ::eyre::bail!("Unexpected response type"),
                    }
                }
            }
        })
        .collect::<Vec<_>>();

    quote! {
        pub struct #client_struct<C, ReqDType, RepDType, E> {
            connection: C,
            _phantom_req: ::std::marker::PhantomData<ReqDType>,
            _phantom_rep: ::std::marker::PhantomData<RepDType>,
            _phantom_err: ::std::marker::PhantomData<E>,
        }

        impl<C, ReqDType, RepDType, E> #client_struct<C, ReqDType, RepDType, E>
        {
            pub fn new(connection: C) -> Self {
                Self {
                    connection,
                    _phantom_req: ::std::marker::PhantomData,
                    _phantom_rep: ::std::marker::PhantomData,
                    _phantom_err: ::std::marker::PhantomData,
                }
            }
        }

        impl<C, E> #client_struct<C, std::vec::Vec<u8>, std::vec::Vec<u8>, E>
        where
            C: ::communication_layer_request_reply::RequestReplyConnection<
                RequestData = std::vec::Vec<u8>,
                ReplyData = std::vec::Vec<u8>,
                Error = E,
            >,
            E: std::marker::Send + std::marker::Sync + std::error::Error + 'static,
        {
            #(#client_methods)*
        }
    }
}
