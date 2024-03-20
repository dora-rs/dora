use heck::SnakeCase;
use quote::{format_ident, quote, ToTokens};
use syn::Ident;

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
    pub fn struct_token_stream(
        &self,
        package_name: &str,
        gen_cxx_bridge: bool,
    ) -> (impl ToTokens, impl ToTokens) {
        let (request_def, request_impl) = self
            .request
            .struct_token_stream(package_name, gen_cxx_bridge);
        let (response_def, response_impl) = self
            .response
            .struct_token_stream(package_name, gen_cxx_bridge);

        let def = quote! {
            #request_def
            #response_def
        };

        let impls = quote! {
            #request_impl
            #response_impl
        };

        (def, impls)
    }

    pub fn alias_token_stream(&self, package_name: &Ident) -> impl ToTokens {
        let srv_type = format_ident!("{}", self.name);
        let req_type_raw = format_ident!("{package_name}__{}_Request", self.name);
        let res_type_raw = format_ident!("{package_name}__{}_Response", self.name);

        let req_type = format_ident!("{}Request", self.name);
        let res_type = format_ident!("{}Response", self.name);

        let request_type_name = req_type.to_string();
        let response_type_name = res_type.to_string();

        quote! {
            #[allow(non_camel_case_types)]
            #[derive(std::fmt::Debug)]
            pub struct #srv_type;

            impl crate::ros2_client::Service for #srv_type {
                type Request = #req_type;
                type Response = #res_type;

                fn request_type_name(&self) -> &str {
                    #request_type_name
                }
                fn response_type_name(&self) -> &str {
                    #response_type_name
                }
            }

            pub use super::super::ffi::#req_type_raw as #req_type;
            pub use super::super::ffi::#res_type_raw as #res_type;
        }
    }

    pub fn cxx_service_creation_functions(
        &self,
        package_name: &str,
    ) -> (impl ToTokens, impl ToTokens) {
        let client_name = format_ident!("Client__{package_name}__{}", self.name);
        let cxx_client_name = format_ident!("Client_{}", self.name);
        let create_client = format_ident!("new_Client__{package_name}__{}", self.name);
        let cxx_create_client = format!("create_client_{package_name}_{}", self.name);

        let package = format_ident!("{package_name}");
        let self_name = format_ident!("{}", self.name);
        let self_name_str = &self.name;

        let def = quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_client_name]
            type #client_name;
            #[cxx_name = #cxx_create_client]
            fn #create_client(self: &mut Ros2Node, name_space: &str, base_name: &str, qos: Ros2QosPolicies) -> Result<Box<#client_name>>;
        };
        let imp = quote! {
            #[allow(non_camel_case_types)]
            pub struct #client_name(ros2_client::service::Client< #package :: service :: #self_name >);

            impl Ros2Node {
                #[allow(non_snake_case)]
                pub fn #create_client(&mut self, name_space: &str, base_name: &str, qos: ffi::Ros2QosPolicies) -> eyre::Result<Box<#client_name>> {
                    let client = self.0.create_client::< #package :: service :: #self_name >(
                        ros2_client::ServiceMapping::Enhanced,
                        &ros2_client::Name::new(name_space, base_name).unwrap(),
                        &ros2_client::ServiceTypeName::new(#package_name, #self_name_str),
                        qos.clone().into(),
                        qos.into(),
                    )?;
                    Ok(Box::new(#client_name(client)))
                }
            }
        };
        (def, imp)
    }

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
