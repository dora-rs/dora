use quote::{ToTokens, format_ident, quote};
use syn::Ident;

use super::Message;

pub fn dds_name(package: &str, name: &str) -> String {
    format!("{package}::srv::dds_::{name}_")
}

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
        let client_backend = format_ident!("ClientBackend__{package_name}__{}", self.name);

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

                let name = crate::ros2_client::Name::new(name_space, base_name)
                    .map_err(|e| eyre::eyre!("invalid ROS2 name/namespace: {e:?}"))?;
                let (response_tx, response_rx) = flume::bounded::<eyre::Result<ffi::#res_type_raw>>(1);

                let backend = match &mut node.node {
                    GeneratedNode::Dds(dds_node) => {
                        let client = dds_node.create_client::< service :: #self_name >(
                            crate::ros2_client::ServiceMapping:: #ros_service_mapping,
                            &name,
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
                        #client_backend::Dds { client, pending }
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { node: zenoh_node, compatibility } => {
                        let (key, token) = zenoh_service_endpoint(
                            zenoh_node, *compatibility, #package_name, #self_name_str, name.to_string(), &qos,
                        )?;
                        let client = futures::executor::block_on(
                            dora_ros2_bridge::transport::zenoh::service::NodeServiceClient::declare(zenoh_node, key.as_str(), token.clone(), MAX_PENDING_REQUESTS)
                        )?;
                        #client_backend::Zenoh {
                            client: std::sync::Arc::new(client),
                            token,
                            response_tx,
                            executor: node.executor.clone(),
                        }
                    }
                };

                let stream = response_rx.into_stream().map(|v| Box::new(v) as Box<dyn std::any::Any + 'static>);
                let id = events.events.merge(Box::pin(stream));
                Ok(Box::new(#client_name {
                    backend,
                    stream_id: id,
                }))
            }

            #[allow(non_camel_case_types)]
            pub struct #client_name {
                backend: #client_backend,
                stream_id: u32,
            }

            #[allow(non_camel_case_types)]
            enum #client_backend {
                Dds {
                    client: std::sync::Arc<crate::ros2_client::service::Client< service :: #self_name >>,
                    pending: std::sync::Arc<std::sync::Mutex<Vec<crate::ros2_client::service::RmwRequestId>>>,
                },
                #[cfg(feature = "rmw-zenoh")]
                Zenoh {
                    client: std::sync::Arc<dora_ros2_bridge::transport::zenoh::service::NodeServiceClient>,
                    token: dora_ros2_bridge::transport::zenoh::keyexpr::TopicToken,
                    // Each request runs as its own task on the node's executor and
                    // sends its response (or error) here, into the merged stream.
                    response_tx: flume::Sender<eyre::Result<ffi::#res_type_raw>>,
                    executor: std::sync::Arc<futures::executor::ThreadPool>,
                },
            }

            impl #client_name {
                #[allow(non_snake_case)]
                fn #wait_for_service(self: &mut #client_name, node: &Box<Ros2Node>) -> eyre::Result<()> {
                    match &self.backend {
                        #client_backend::Dds { client, .. } => {
                            let service_ready = async {
                                for _ in 0..30 {
                                    let ready = client.wait_for_service(node.dds_node()?);
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
                        #[cfg(feature = "rmw-zenoh")]
                        #client_backend::Zenoh { token, .. } => {
                            let GeneratedNode::Zenoh { node: zenoh_node, .. } = &node.node else {
                                eyre::bail!("service client belongs to a different ROS2 transport");
                            };
                            // Same 60 s budget as the DDS loop above (30 x 2 s).
                            futures::executor::block_on(dora_ros2_bridge::transport::zenoh::service::wait_for_service(
                                zenoh_node.graph(),
                                &token.name,
                                &token.type_name,
                                &token.type_hash,
                                &token.qos,
                                std::time::Instant::now() + std::time::Duration::from_secs(60),
                            ))
                            .map_err(|e| eyre::eyre!("service not available: {e}"))
                        }
                    }
                }

                #[allow(non_snake_case)]
                fn #send_request(&mut self, request: ffi::#req_type_raw) -> eyre::Result<()> {
                    use eyre::WrapErr;

                    match &self.backend {
                        #client_backend::Dds { client, pending } => {
                            // Register the request id *before* the response pump can act on
                            // any reply. The id is only known once `async_send_request`
                            // returns, so hold the `pending` lock across the send: the pump
                            // must take the same lock to match (or drop) a response, so a
                            // service that replies before the send call returns cannot have
                            // its response dropped as "not ours" during the registration
                            // window. See dora-rs/dora#1970.
                            let mut pending = pending
                                .lock()
                                .map_err(|_| eyre::eyre!("service client response state poisoned"))?;
                            let req_id = futures::executor::block_on(client.async_send_request(request.clone()))
                                .context("failed to send request")
                                .map_err(|e| eyre::eyre!("{e:?}"))?;
                            pending.push(req_id);
                            Ok(())
                        }
                        #[cfg(feature = "rmw-zenoh")]
                        #client_backend::Zenoh { client, response_tx, executor, .. } => {
                            use futures::task::SpawnExt as _;

                            // Same bound as the server-side request expiry, so an
                            // unanswered request surfaces as an error event instead of hanging.
                            let payload = dora_ros2_bridge::transport::zenoh::serialize_cdr(&request)?;
                            let client = client.clone();
                            let response_tx = response_tx.clone();
                            executor.spawn(async move {
                                let response = match client.call(payload, SERVICE_RESPONSE_TIMEOUT).await {
                                    Ok(bytes) => dora_ros2_bridge::transport::zenoh::deserialize_cdr::<ffi::#res_type_raw>(&bytes)
                                        .map_err(|e| eyre::eyre!("failed to decode {} response: {e}", #self_name_str)),
                                    Err(e) => Err(eyre::eyre!("{} service call failed: {e}", #self_name_str)),
                                };
                                let _ = response_tx.send_async(response).await;
                            })
                            .context("failed to spawn service request")?;
                            Ok(())
                        }
                    }
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
        let server_backend = format_ident!("ServerBackend__{package_name}__{}", self.name);

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

                let name = crate::ros2_client::Name::new(name_space, base_name)
                    .map_err(|e| eyre::eyre!("invalid ROS2 name/namespace: {e:?}"))?;
                // Token -> (transport request id, receive time). The C++ side
                // gets the opaque `u64` token with each request and passes it back
                // to `send_response`, so the transport's request id never crosses
                // the FFI. `record_request` bounds the map.
                let requests: std::sync::Arc<std::sync::Mutex<std::collections::HashMap<u64, (PendingRequestId, std::time::Instant)>>> =
                    std::sync::Arc::new(std::sync::Mutex::new(std::collections::HashMap::new()));
                let next_id = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
                let (request_tx, request_rx) = flume::bounded::<eyre::Result<(u64, ffi::#req_type_raw)>>(MAX_PENDING_REQUESTS);

                let backend = match &mut node.node {
                    GeneratedNode::Dds(dds_node) => {
                        let server = dds_node.create_server::< service :: #self_name >(
                            crate::ros2_client::ServiceMapping:: #ros_service_mapping,
                            &name,
                            &crate::ros2_client::ServiceTypeName::new(#package_name, #self_name_str),
                            qos.clone().into(),
                            qos.into(),
                        ).map_err(|e| eyre::eyre!("{e:?}"))?;
                        let server = std::sync::Arc::new(server);

                        // Single request pump per server: poll for requests, tag each with
                        // a token, record its `RmwRequestId`, and forward it to the merged
                        // stream. Mirrors the per-client response pump.
                        {
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
                                            record_request(&requests, token, PendingRequestId::Dds(rmw_id));
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
                        #server_backend::Dds(server)
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { node: zenoh_node, compatibility } => {
                        use futures::task::SpawnExt as _;
                        use eyre::WrapErr as _;

                        let (key, token) = zenoh_service_endpoint(
                            zenoh_node, *compatibility, #package_name, #self_name_str, name.to_string(), &qos,
                        )?;
                        let server = std::sync::Arc::new(futures::executor::block_on(
                            dora_ros2_bridge::transport::zenoh::service::NodeServiceServer::declare(
                                zenoh_node,
                                key.as_str(),
                                token,
                                MAX_PENDING_REQUESTS,
                                SERVICE_RESPONSE_TIMEOUT,
                            )
                        )?);

                        // Request pump: same token scheme as the DDS pump above. It
                        // stops when the server object is dropped (its `shutdown`
                        // sender closes), so the service leaves the ROS graph.
                        let (shutdown, shutdown_rx) = flume::bounded::<()>(1);
                        let pump = {
                            let server = server.clone();
                            let requests = requests.clone();
                            let next_id = next_id.clone();
                            async move {
                                let stopped = shutdown_rx.recv_async();
                                futures::pin_mut!(stopped);
                                loop {
                                    let received = server.recv();
                                    futures::pin_mut!(received);
                                    // `recv` only fails once the transport is closed.
                                    let request = match futures::future::select(received, stopped.as_mut()).await {
                                        futures::future::Either::Left((Ok(request), _)) => request,
                                        _ => break,
                                    };
                                    let decoded = match dora_ros2_bridge::transport::zenoh::deserialize_cdr::<ffi::#req_type_raw>(&request.payload) {
                                        Ok(decoded) => decoded,
                                        Err(e) => {
                                            // Tell the caller rather than leave it waiting
                                            // for a reply that will never come.
                                            let _ = server.reject(request.id, &format!("invalid {} request: {e}", #self_name_str)).await;
                                            continue;
                                        }
                                    };
                                    let token = next_id.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                    record_request(&requests, token, PendingRequestId::Zenoh(request.id));
                                    if request_tx.send_async(Ok((token, decoded))).await.is_err() {
                                        break;
                                    }
                                }
                            }
                        };
                        node.executor.spawn(pump).context("failed to spawn service request pump")?;
                        #server_backend::Zenoh { server, _shutdown: shutdown }
                    }
                };

                let stream = request_rx.into_stream().map(|v| Box::new(v) as Box<dyn std::any::Any + 'static>);
                let id = events.events.merge(Box::pin(stream));
                Ok(Box::new(#server_name {
                    backend,
                    requests,
                    stream_id: id,
                }))
            }

            #[allow(non_camel_case_types)]
            pub struct #server_name {
                backend: #server_backend,
                requests: std::sync::Arc<std::sync::Mutex<std::collections::HashMap<u64, (PendingRequestId, std::time::Instant)>>>,
                stream_id: u32,
            }

            #[allow(non_camel_case_types)]
            enum #server_backend {
                Dds(std::sync::Arc<crate::ros2_client::service::Server< service :: #self_name >>),
                #[cfg(feature = "rmw-zenoh")]
                Zenoh {
                    server: std::sync::Arc<dora_ros2_bridge::transport::zenoh::service::NodeServiceServer>,
                    // Dropping this stops the request pump.
                    _shutdown: flume::Sender<()>,
                },
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
                    let (pending_id, _received_at) = self
                        .requests
                        .lock()
                        .map_err(|_| eyre::eyre!("service server request state poisoned"))?
                        .remove(&id)
                        .ok_or_else(|| eyre::eyre!("no pending request with id {id} (it may have timed out)"))?;
                    match (&self.backend, pending_id) {
                        (#server_backend::Dds(server), PendingRequestId::Dds(rmw_id)) => {
                            futures::executor::block_on(server.async_send_response(rmw_id, response))
                                .map_err(|e| eyre::eyre!("failed to send service response: {e:?}"))?;
                        }
                        #[cfg(feature = "rmw-zenoh")]
                        (#server_backend::Zenoh { server, .. }, PendingRequestId::Zenoh(request_id)) => {
                            let payload = dora_ros2_bridge::transport::zenoh::serialize_cdr(&response)?;
                            futures::executor::block_on(server.reply(request_id, &payload))
                                .map_err(|e| eyre::eyre!("failed to send service response: {e}"))?;
                        }
                        #[allow(unreachable_patterns)]
                        _ => eyre::bail!("request id belongs to a different ROS2 transport"),
                    }
                    Ok(())
                }
            }
        };
        (def, imp)
    }
}

#[cfg(test)]
mod codegen_tests {
    use super::*;

    fn empty_msg(name: &str) -> Message {
        Message {
            package: "test_pkg".to_string(),
            name: name.to_string(),
            members: vec![],
            constants: vec![],
        }
    }

    fn add_two_ints() -> Service {
        Service {
            package: "test_pkg".to_string(),
            name: "AddTwoInts".to_string(),
            request: empty_msg("AddTwoInts_Request"),
            response: empty_msg("AddTwoInts_Response"),
        }
    }

    fn format_valid_rust(tokens: impl ToTokens) -> String {
        crate::format_token_stream(tokens.into_token_stream())
    }

    // #2027: the generated client/server `create` fns used to
    // `Name::new(..).unwrap()` (a node-abort panic on a bad name/namespace);
    // they now propagate via `?`. Guard that the generated code (a) still parses
    // and (b) routes the `Name::new` error through `map_err` rather than panic.
    #[test]
    fn service_creation_functions_propagate_name_errors() {
        let svc = add_two_ints();
        let needle = "invalid ROS2 name/namespace"; // must propagate via `?`, not panic
        let (_decl, imp) = svc.cxx_service_creation_functions("test_pkg");
        assert!(format_valid_rust(imp).contains(needle), "client create fn");
        let (_decl, imp) = svc.cxx_service_server_creation_functions("test_pkg");
        assert!(format_valid_rust(imp).contains(needle), "server create fn");
    }

    // #3764: on rmw_zenoh the generated C++ service client/server used to
    // bail with "entity requires the DDS transport".
    #[test]
    fn service_creation_functions_support_rmw_zenoh() {
        let svc = add_two_ints();
        let (_decl, imp) = svc.cxx_service_creation_functions("test_pkg");
        let client = format_valid_rust(imp);
        assert!(client.contains("GeneratedNode::Zenoh"), "client create fn");
        assert!(
            client.contains("NodeServiceClient::declare"),
            "client create fn"
        );
        let (_decl, imp) = svc.cxx_service_server_creation_functions("test_pkg");
        let server = format_valid_rust(imp);
        assert!(server.contains("GeneratedNode::Zenoh"), "server create fn");
        assert!(
            server.contains("NodeServiceServer::declare"),
            "server create fn"
        );
    }
}
