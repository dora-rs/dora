use heck::SnakeCase;
use quote::{format_ident, quote, ToTokens};

use super::Message;

/// A service definition
#[derive(Debug, Clone)]
pub struct Service {
    /// The name of The package
    pub package: String,
    /// The name of the service
    pub name: String,
    /// The type of the request
    pub request: Message,
    /// The type of the response
    pub response: Message,
}

impl Service {
    pub fn token_stream_with_mod(&self) -> impl ToTokens {
        let mod_name = format_ident!("_{}", self.name.to_snake_case());
        let inner = self.token_stream();
        quote! {
            pub use #mod_name::*;
            mod #mod_name {
                #inner
            }
        }
    }

    pub fn token_stream(&self) -> impl ToTokens {
        let srv_type = format_ident!("{}", self.name);
        let req_type = format_ident!("{}_Request", self.name);
        let res_type = format_ident!("{}_Response", self.name);

        let request_body = self.request.token_stream();
        let response_body = self.response.token_stream();

        quote! {
            use std::os::raw::c_void;

            pub use self::request::*;
            pub use self::response::*;

            #[allow(non_camel_case_types)]
            #[derive(std::fmt::Debug)]
            pub struct #srv_type;


            impl crate::_core::ServiceT for #srv_type {
                type Request = #req_type;
                type Response = #res_type;
            }

            mod request {
                #request_body
            }  // mod request

            mod response {
                #response_body
            }  // mod response

            #[cfg(test)]
            mod test {
                use super::*;
                use crate::_core::ServiceT;

                #[test]
                fn test_type_support() {
                    let ptr = #srv_type::type_support();
                    assert!(!ptr.is_null());
                }
            }
        }
    }
}
