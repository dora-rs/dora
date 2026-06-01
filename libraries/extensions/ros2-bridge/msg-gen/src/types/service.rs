use heck::SnakeCase;
use quote::{ToTokens, format_ident, quote};
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

            pub use super::ffi::#req_type_raw as #req_type;
            pub use super::ffi::#res_type_raw as #res_type;
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

        // Pick the ROS2 service mapping based on RMW_IMPLEMENTATION /
        // ROS_DISTRO at codegen time. See dora-rs/dora#449.
        let ros_service_mapping = crate::detect_ros_service_mapping_ident();

        let def = quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_client_name]
            type #client_name;
            // TODO: add `merged_streams` argument (for sending replies)
            #[cxx_name = #cxx_create_client]
            fn #create_client(node: &mut Ros2Node, name_space: &str, base_name: &str, qos: Ros2QosPolicies, events: &mut CombinedEvents) -> Result<Box<#client_name>>;

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

            #[allow(non_snake_case)]
            pub fn #create_client(node: &mut Ros2Node, name_space: &str, base_name: &str, qos: ffi::Ros2QosPolicies, events: &mut crate::ffi::CombinedEvents) -> eyre::Result<Box<#client_name>> {
                use futures::StreamExt as _;

                let client = node.node.create_client::< service :: #self_name >(
                    crate::ros2_client::ServiceMapping:: #ros_service_mapping,
                    &crate::ros2_client::Name::new(name_space, base_name).unwrap(),
                    &crate::ros2_client::ServiceTypeName::new(#package_name, #self_name_str),
                    qos.clone().into(),
                    qos.into(),
                ).map_err(|e| eyre::eyre!("{e:?}"))?;
                let client = std::sync::Arc::new(client);
                // Request ids this client has sent and not yet matched to a
                // response. `receive_response` can hand back responses for *other*
                // clients sharing the service topic (see its rustdoc), so the pump
                // forwards only responses whose id is in this set.
                let pending: std::sync::Arc<std::sync::Mutex<Vec<crate::ros2_client::service::RmwRequestId>>> =
                    std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
                let (response_tx, response_rx) = flume::bounded(1);
                let stream = response_rx.into_stream().map(|v: eyre::Result<_>| Box::new(v) as Box<dyn std::any::Any + 'static>);
                let id = events.events.merge(Box::pin(stream));

                // Single response pump per client. ros2-client's
                // `async_receive_response(id)` discards samples whose id does not
                // match, so spawning one receiver per request makes concurrent
                // receivers steal and drop each other's responses (the C++ client
                // then never sees any) — see dora-rs/dora#1970. A single consumer
                // that forwards each response to the request it belongs to avoids
                // that while still honoring the request/response id contract.
                {
                    let client = client.clone();
                    let pending = pending.clone();
                    std::thread::spawn(move || {
                        loop {
                            if response_tx.is_disconnected() {
                                break;
                            }
                            match client.receive_response() {
                                Ok(Some((req_id, response))) => {
                                    // Drop responses we have no matching request for:
                                    // they belong to another client and are delivered
                                    // to that client's own pump too.
                                    let is_ours = match pending.lock() {
                                        Ok(mut pending) => match pending.iter().position(|pid| *pid == req_id) {
                                            Some(pos) => {
                                                pending.remove(pos);
                                                true
                                            }
                                            None => false,
                                        },
                                        Err(_) => false,
                                    };
                                    if is_ours && response_tx.send(Ok(response)).is_err() {
                                        break;
                                    }
                                }
                                Ok(None) => {
                                    std::thread::sleep(std::time::Duration::from_millis(2));
                                }
                                Err(e) => {
                                    let _ = response_tx
                                        .send(Err(eyre::eyre!("failed to receive service response: {e:?}")));
                                    std::thread::sleep(std::time::Duration::from_millis(2));
                                }
                            }
                        }
                    });
                }

                Ok(Box::new(#client_name {
                    client,
                    pending,
                    stream_id: id,
                }))
            }

            #[allow(non_camel_case_types)]
            pub struct #client_name {
                client: std::sync::Arc<crate::ros2_client::service::Client< service :: #self_name >>,
                pending: std::sync::Arc<std::sync::Mutex<Vec<crate::ros2_client::service::RmwRequestId>>>,
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

                    // Register the request id *before* the response pump can act on
                    // any reply. The id is only known once `async_send_request`
                    // returns, so hold the `pending` lock across the send: the pump
                    // must take the same lock to match (or drop) a response, so a
                    // service that replies before the send call returns cannot have
                    // its response dropped as "not ours" during the registration
                    // window. See dora-rs/dora#1970.
                    let mut pending = self
                        .pending
                        .lock()
                        .map_err(|_| eyre::eyre!("service client response state poisoned"))?;
                    let req_id = futures::executor::block_on(self.client.async_send_request(request.clone()))
                        .context("failed to send request")
                        .map_err(|e| eyre::eyre!("{e:?}"))?;
                    pending.push(req_id);
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

    /// C++ FFI for hosting a ROS2 service **server**. Mirrors
    /// `cxx_service_creation_functions` (the client) in reverse: a background
    /// pump receives requests and forwards them (tagged with a `u64` token)
    /// into the merged event stream; the C++ side inspects each request and
    /// calls `send_response(id, response)`, which correlates the token back to
    /// the ros2-client `RmwRequestId`. The token avoids exposing `RmwRequestId`
    /// across the cxx boundary.
    pub fn cxx_service_server_creation_functions(
        &self,
        package_name: &str,
    ) -> (impl ToTokens, impl ToTokens) {
        let server_name = format_ident!("Server__{package_name}__{}", self.name);
        let cxx_server_name = format_ident!("Server_{}", self.name);
        let create_server = format_ident!("new_Server__{package_name}__{}", self.name);
        let cxx_create_server = format!("create_service_server_{package_name}_{}", self.name);

        let request_event_name = format_ident!("ServiceRequest__{package_name}__{}", self.name);
        let cxx_request_event_name = format_ident!("ServiceRequest_{}", self.name);
        let get_request = format_ident!("get_request__{package_name}__{}", self.name);
        let get_id = format_ident!("get_id__{package_name}__{}", self.name);

        let self_name = format_ident!("{}", self.name);
        let self_name_str = &self.name;

        let matches = format_ident!("server_matches__{package_name}__{}", self.name);
        let downcast = format_ident!("server_downcast__{package_name}__{}", self.name);
        let send_response = format_ident!("send_response__{package_name}__{}", self.name);

        let req_type_raw = format_ident!("{package_name}__{}_Request", self.name);
        let res_type_raw = format_ident!("{package_name}__{}_Response", self.name);
        let req_type_raw_str = req_type_raw.to_string();

        let ros_service_mapping = crate::detect_ros_service_mapping_ident();

        let def = quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_request_event_name]
            type #request_event_name;
            #[namespace = #package_name]
            #[cxx_name = get_request]
            fn #get_request(self: &#request_event_name) -> &#req_type_raw;
            #[namespace = #package_name]
            #[cxx_name = get_id]
            fn #get_id(self: &#request_event_name) -> u64;

            #[namespace = #package_name]
            #[cxx_name = #cxx_server_name]
            type #server_name;
            #[cxx_name = #cxx_create_server]
            fn #create_server(node: &mut Ros2Node, name_space: &str, base_name: &str, qos: Ros2QosPolicies, events: &mut CombinedEvents) -> Result<Box<#server_name>>;
            #[namespace = #package_name]
            #[cxx_name = matches]
            fn #matches(self: &#server_name, event: &CombinedEvent) -> bool;
            #[namespace = #package_name]
            #[cxx_name = downcast]
            fn #downcast(self: &#server_name, event: CombinedEvent) -> Result<Box<#request_event_name>>;
            #[namespace = #package_name]
            #[cxx_name = send_response]
            fn #send_response(self: &mut #server_name, id: u64, response: #res_type_raw) -> Result<()>;
        };
        let imp = quote! {
            #[allow(non_snake_case)]
            pub fn #create_server(node: &mut Ros2Node, name_space: &str, base_name: &str, qos: ffi::Ros2QosPolicies, events: &mut crate::ffi::CombinedEvents) -> eyre::Result<Box<#server_name>> {
                use futures::StreamExt as _;

                let server = node.node.create_server::< service :: #self_name >(
                    crate::ros2_client::ServiceMapping:: #ros_service_mapping,
                    &crate::ros2_client::Name::new(name_space, base_name).unwrap(),
                    &crate::ros2_client::ServiceTypeName::new(#package_name, #self_name_str),
                    qos.clone().into(),
                    qos.into(),
                ).map_err(|e| eyre::eyre!("{e:?}"))?;
                let server = std::sync::Arc::new(server);
                // Token -> (ros2-client request id, receive time). The C++ side
                // gets the opaque `u64` token with each request and passes it back
                // to `send_response`, so `RmwRequestId` never crosses the FFI. The
                // receive time bounds the map: an entry whose C++ handler never
                // calls `send_response` (crash, early return, dropped request) is
                // evicted, so a misbehaving server cannot leak unboundedly. Mirrors
                // the daemon's SERVICE_RESPONSE_TIMEOUT / MAX_PENDING_REQUESTS.
                let requests: std::sync::Arc<std::sync::Mutex<std::collections::HashMap<u64, (crate::ros2_client::service::RmwRequestId, std::time::Instant)>>> =
                    std::sync::Arc::new(std::sync::Mutex::new(std::collections::HashMap::new()));
                let next_id = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
                let (request_tx, request_rx) = flume::bounded(64);
                let stream = request_rx.into_stream().map(|v: eyre::Result<(u64, ffi::#req_type_raw)>| Box::new(v) as Box<dyn std::any::Any + 'static>);
                let id = events.events.merge(Box::pin(stream));

                // Single request pump per server: poll for requests, tag each with
                // a token, record its `RmwRequestId`, and forward it to the merged
                // stream. Mirrors the per-client response pump.
                {
                    // Cap unanswered requests so a server that never replies cannot
                    // grow the map forever (matches the daemon's eviction policy).
                    const REQUEST_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(30);
                    const MAX_PENDING_REQUESTS: usize = 64;
                    let server = server.clone();
                    let requests = requests.clone();
                    let next_id = next_id.clone();
                    std::thread::spawn(move || {
                        loop {
                            if request_tx.is_disconnected() {
                                break;
                            }
                            match server.receive_request() {
                                Ok(Some((rmw_id, request))) => {
                                    let token = next_id.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                    if let Ok(mut map) = requests.lock() {
                                        let now = std::time::Instant::now();
                                        // Evict entries whose response never came.
                                        map.retain(|_, (_, t)| now.duration_since(*t) < REQUEST_TIMEOUT);
                                        // If still at capacity, drop the oldest
                                        // (smallest token) to make room.
                                        if map.len() >= MAX_PENDING_REQUESTS {
                                            if let Some(&oldest) = map.keys().min() {
                                                map.remove(&oldest);
                                            }
                                        }
                                        map.insert(token, (rmw_id, now));
                                    }
                                    if request_tx.send(Ok((token, request))).is_err() {
                                        break;
                                    }
                                }
                                Ok(None) => {
                                    std::thread::sleep(std::time::Duration::from_millis(2));
                                }
                                Err(e) => {
                                    let _ = request_tx
                                        .send(Err(eyre::eyre!("failed to receive service request: {e:?}")));
                                    std::thread::sleep(std::time::Duration::from_millis(2));
                                }
                            }
                        }
                    });
                }

                Ok(Box::new(#server_name {
                    server,
                    requests,
                    stream_id: id,
                }))
            }

            #[allow(non_camel_case_types)]
            pub struct #server_name {
                server: std::sync::Arc<crate::ros2_client::service::Server< service :: #self_name >>,
                requests: std::sync::Arc<std::sync::Mutex<std::collections::HashMap<u64, (crate::ros2_client::service::RmwRequestId, std::time::Instant)>>>,
                stream_id: u32,
            }

            #[allow(non_camel_case_types)]
            pub struct #request_event_name {
                id: u64,
                request: ffi::#req_type_raw,
            }

            impl #request_event_name {
                #[allow(non_snake_case)]
                fn #get_request(&self) -> &ffi::#req_type_raw {
                    &self.request
                }
                #[allow(non_snake_case)]
                fn #get_id(&self) -> u64 {
                    self.id
                }
            }

            impl #server_name {
                #[allow(non_snake_case)]
                fn #matches(&self, event: &crate::ffi::CombinedEvent) -> bool {
                    match &event.event.as_ref().0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.stream_id => true,
                        _ => false,
                    }
                }

                #[allow(non_snake_case)]
                fn #downcast(&self, event: crate::ffi::CombinedEvent) -> eyre::Result<Box<#request_event_name>> {
                    match (*event.event).0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.stream_id => {
                            let result = event.event.downcast::<eyre::Result<(u64, ffi::#req_type_raw)>>()
                                .map_err(|_| eyre::eyre!("downcast to {} failed", #req_type_raw_str))?;
                            let (id, request) = (*result).map_err(|e| eyre::eyre!("{e:?}"))?;
                            Ok(Box::new(#request_event_name { id, request }))
                        }
                        _ => eyre::bail!("not a {} request event", #self_name_str),
                    }
                }

                #[allow(non_snake_case)]
                fn #send_response(&mut self, id: u64, response: ffi::#res_type_raw) -> eyre::Result<()> {
                    let (rmw_id, _received_at) = self
                        .requests
                        .lock()
                        .map_err(|_| eyre::eyre!("service server request state poisoned"))?
                        .remove(&id)
                        .ok_or_else(|| eyre::eyre!("no pending request with id {id} (it may have timed out)"))?;
                    futures::executor::block_on(self.server.async_send_response(rmw_id, response))
                        .map_err(|e| eyre::eyre!("failed to send service response: {e:?}"))?;
                    Ok(())
                }
            }
        };
        (def, imp)
    }

    pub fn token_stream_with_mod(&self) -> impl ToTokens + use<> {
        let mod_name = format_ident!("_{}", self.name.to_snake_case());
        let inner = self.token_stream();
        quote! {
            pub use #mod_name::*;
            mod #mod_name {
                #inner
            }
        }
    }

    pub fn token_stream(&self) -> impl ToTokens + use<> {
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
