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

        let wait_for_service = format_ident!("wait_for_service__{package_name}__{}", self.name);
        let cxx_wait_for_service = format_ident!("wait_for_service");
        let send_request = format_ident!("send_request__{package_name}__{}", self.name);
        let cxx_send_request = format_ident!("send_request");
        let req_type_raw = format_ident!("{package_name}__{}_Request", self.name);
        let res_type_raw = format_ident!("{package_name}__{}_Response", self.name);
        let res_type_raw_str = res_type_raw.to_string();

        let matches = format_ident!("matches__{package_name}__{}", self.name);
        let cxx_matches = format_ident!("matches");
        let downcast = format_ident!("downcast__{package_name}__{}", self.name);
        let cxx_downcast = format_ident!("downcast");

        let def = quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_client_name]
            type #client_name;
            // TODO: add `merged_streams` argument (for sending replies)
            #[cxx_name = #cxx_create_client]
            fn #create_client(self: &mut Ros2Node, name_space: &str, base_name: &str, qos: Ros2QosPolicies, events: &mut CombinedEvents) -> Result<Box<#client_name>>;

            #[namespace = #package_name]
            #[cxx_name = #cxx_wait_for_service]
            fn #wait_for_service(self: &mut #client_name, node: &Box<Ros2Node>) -> Result<()>;
            #[namespace = #package_name]
            #[cxx_name = #cxx_send_request]
            fn #send_request(self: &mut #client_name, request: #req_type_raw) -> Result<()>;
            #[namespace = #package_name]
            #[cxx_name = #cxx_matches]
            fn #matches(self: &#client_name, event: &CombinedEvent) -> bool;
            #[namespace = #package_name]
            #[cxx_name = #cxx_downcast]
            fn #downcast(self: &#client_name, event: CombinedEvent) -> Result<#res_type_raw>;
        };
        let imp = quote! {
            impl Ros2Node {
                #[allow(non_snake_case)]
                pub fn #create_client(&mut self, name_space: &str, base_name: &str, qos: ffi::Ros2QosPolicies, events: &mut crate::ffi::CombinedEvents) -> eyre::Result<Box<#client_name>> {
                    use futures::StreamExt as _;

                    let client = self.node.create_client::< #package :: service :: #self_name >(
                        ros2_client::ServiceMapping::Enhanced,
                        &ros2_client::Name::new(name_space, base_name).unwrap(),
                        &ros2_client::ServiceTypeName::new(#package_name, #self_name_str),
                        qos.clone().into(),
                        qos.into(),
                    ).map_err(|e| eyre::eyre!("{e:?}"))?;
                    let (response_tx, response_rx) = flume::bounded(1);
                    let stream = response_rx.into_stream().map(|v: eyre::Result<_>| Box::new(v) as Box<dyn std::any::Any + 'static>);
                    let id = events.events.merge(Box::pin(stream));

                    Ok(Box::new(#client_name {
                        client: std::sync::Arc::new(client),
                        response_tx: std::sync::Arc::new(response_tx),
                        executor: self.executor.clone(),
                        stream_id: id,
                    }))
                }
            }

            #[allow(non_camel_case_types)]
            pub struct #client_name {
                client: std::sync::Arc<ros2_client::service::Client< #package :: service :: #self_name >>,
                response_tx: std::sync::Arc<flume::Sender<eyre::Result<ffi::#res_type_raw>>>,
                executor: std::sync::Arc<futures::executor::ThreadPool>,
                stream_id: u32,
            }

            impl #client_name {
                #[allow(non_snake_case)]
                fn #wait_for_service(self: &mut #client_name, node: &Box<Ros2Node>) -> eyre::Result<()> {
                    let service_ready = async {
                        for _ in 0..10 {
                            let ready = self.client.wait_for_service(&node.node);
                            futures::pin_mut!(ready);
                            let timeout = futures_timer::Delay::new(std::time::Duration::from_secs(2));
                            match futures::future::select(ready, timeout).await {
                                futures::future::Either::Left(((), _)) => {
                                    return Ok(());
                                }
                                futures::future::Either::Right(_) => {
                                    eprintln!("timeout while waiting for service, retrying");
                                }
                            }
                        }
                        eyre::bail!("service not available");
                    };
                    futures::executor::block_on(service_ready)?;
                    Ok(())
                }

                #[allow(non_snake_case)]
                fn #send_request(&mut self, request: ffi::#req_type_raw) -> eyre::Result<()> {
                    use eyre::WrapErr;
                    use futures::task::SpawnExt as _;

                    let request_id = futures::executor::block_on(self.client.async_send_request(request.clone()))
                        .context("failed to send request")
                        .map_err(|e| eyre::eyre!("{e:?}"))?;
                    let client = self.client.clone();
                    let response_tx = self.response_tx.clone();
                    let send_result = async move {
                        let response = client.async_receive_response(request_id).await.with_context(|| format!("failed to receive response for request {request_id:?}"));
                        if response_tx.send_async(response).await.is_err() {
                            tracing::warn!("failed to send service response");
                        }
                    };
                    self.executor.spawn(send_result).context("failed to spawn response task").map_err(|e| eyre::eyre!("{e:?}"))?;
                    Ok(())
                }

                #[allow(non_snake_case)]
                fn #matches(&self, event: &crate::ffi::CombinedEvent) -> bool {
                    match &event.event.as_ref().0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.stream_id  => true,
                        _ => false
                    }
                }
                #[allow(non_snake_case)]
                fn #downcast(&self, event: crate::ffi::CombinedEvent) -> eyre::Result<ffi::#res_type_raw> {
                    use eyre::WrapErr;

                    match (*event.event).0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.stream_id  => {
                            let result = event.event.downcast::<eyre::Result<ffi::#res_type_raw>>()
                                .map_err(|_| eyre::eyre!("downcast to {} failed", #res_type_raw_str))?;

                            let data = result.with_context(|| format!("failed to receive {} response", #self_name_str))
                                .map_err(|e| eyre::eyre!("{e:?}"))?;
                            Ok(data)
                        },
                        _ => eyre::bail!("not a {} response event", #self_name_str),
                    }
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
