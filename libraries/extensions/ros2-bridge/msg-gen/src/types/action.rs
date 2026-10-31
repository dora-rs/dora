use quote::{ToTokens, format_ident, quote};
use syn::Ident;

use super::Message;

pub fn component_names(name: &str) -> [String; 6] {
    [
        "_Goal",
        "_Result",
        "_Feedback",
        "_SendGoal",
        "_GetResult",
        "_FeedbackMessage",
    ]
    .map(|suffix| format!("{name}{suffix}"))
}

/// DDS type names of the three services and two topics composing a ROS action.
pub fn dds_type_names(package: &str, name: &str) -> [String; 5] {
    [
        format!("{package}::action::dds_::{name}_SendGoal_"),
        format!("{package}::action::dds_::{name}_GetResult_"),
        "action_msgs::srv::dds_::CancelGoal_".into(),
        format!("{package}::action::dds_::{name}_FeedbackMessage_"),
        "action_msgs::msg::dds_::GoalStatusArray_".into(),
    ]
}

/// An action definition
#[derive(Debug, Clone)]
pub struct Action {
    /// The name of The package
    pub package: String,
    /// The name of The action
    pub name: String,
    /// The type of the goal
    pub goal: Message,
    /// The type of the result
    pub result: Message,
    /// The type of the feedback
    pub feedback: Message,
}

impl Action {
    pub fn struct_token_stream(
        &self,
        package_name: &str,
        gen_cxx_bridge: bool,
    ) -> (impl ToTokens, impl ToTokens) {
        let (goal_def, goal_impl) = self.goal.struct_token_stream(package_name, gen_cxx_bridge);
        let (result_def, result_impl) = self
            .result
            .struct_token_stream(package_name, gen_cxx_bridge);
        let (feedback_def, feedback_impl) = self
            .feedback
            .struct_token_stream(package_name, gen_cxx_bridge);

        let def = quote! {
            #goal_def
            #result_def
            #feedback_def
        };

        let impls = quote! {
            #goal_impl
            #result_impl
            #feedback_impl
        };

        (def, impls)
    }

    pub fn alias_token_stream(&self, package_name: &Ident) -> impl ToTokens {
        let action_type = format_ident!("{}", self.name);
        let goal_type_raw = format_ident!("{package_name}__{}_Goal", self.name);
        let result_type_raw = format_ident!("{package_name}__{}_Result", self.name);
        let feedback_type_raw = format_ident!("{package_name}__{}_Feedback", self.name);

        let goal_type = format_ident!("{}Goal", self.name);
        let result_type = format_ident!("{}Result", self.name);
        let feedback_type = format_ident!("{}Feedback", self.name);

        let goal_type_name = goal_type.to_string();
        let result_type_name = result_type.to_string();
        let feedback_type_name = feedback_type.to_string();

        quote! {
            #[allow(non_camel_case_types)]
            #[derive(std::fmt::Debug)]
            pub struct #action_type;

            impl crate::ros2_client::ActionTypes for #action_type {
                type GoalType = #goal_type;
                type ResultType = #result_type;
                type FeedbackType = #feedback_type;

                fn goal_type_name(&self) -> &str {
                    #goal_type_name
                }

                fn result_type_name(&self) -> &str {
                    #result_type_name
                }

                fn feedback_type_name(&self) -> &str {
                    #feedback_type_name
                }
            }

            pub use super::ffi::#goal_type_raw as #goal_type;
            pub use super::ffi::#result_type_raw as #result_type;
            pub use super::ffi::#feedback_type_raw as #feedback_type;

        }
    }

    pub fn cxx_action_creation_functions(
        &self,
        package_name: &str,
    ) -> (impl ToTokens, impl ToTokens) {
        let client_name = format_ident!("Actionclient__{package_name}__{}", self.name);
        let cxx_client_name = format!("Actionclient_{}", self.name);
        let create_client = format_ident!("new_ActionClient__{package_name}__{}", self.name);
        let cxx_create_client = format!("create_{}_action_client", self.name);

        let self_name = format_ident!("{}", self.name);
        let self_name_str = &self.name;

        let wait_for_action = format_ident!("wait_for__{package_name}__{}", self.name);
        let send_goal = format_ident!("send__{package_name}__{}_goal", self.name);
        let send_goal_with_timeout =
            format_ident!("send__{package_name}__{}_goal_with_timeout", self.name);
        let cancel_goal = format_ident!("cancel__{package_name}__{}_goal", self.name);

        let request_result = format_ident!("request__{package_name}__{}_result", self.name);
        let result_matches = format_ident!("matches__{package_name}__{}_response", self.name);
        let feedback_matches = format_ident!("matches__{package_name}__{}_feedback", self.name);
        let status_matches = format_ident!("matches__{package_name}__{}_status", self.name);

        let result_event_name =
            format_ident!("ActionClientResultEvent__{package_name}__{}", self.name);
        let cxx_result_event = format!("ResultEvent_{}", self.name);
        let result_event_matches = format_ident!(
            "ActionClientResultEvent_matches__{package_name}__{}",
            self.name
        );
        let result_event_get_status = format_ident!(
            "ActionClientResultEvent_get_status__{package_name}__{}",
            self.name
        );
        let result_event_get_result = format_ident!(
            "ActionClientResultEvent_get_result__{package_name}__{}",
            self.name
        );
        let feedback_event_name =
            format_ident!("ActionClientFeedbackEvent__{package_name}__{}", self.name);
        let cxx_feedback_event = format!("FeedbackEvent_{}", self.name);
        let feedback_event_matches = format_ident!(
            "ActionClientFeedbackEvent_matches__{package_name}__{}",
            self.name
        );
        let feedback_event_get_feedback = format_ident!(
            "ActionClientFeedbackEvent_get_feedback__{package_name}__{}",
            self.name
        );
        let status_event_name =
            format_ident!("ActionClientStatusEvent__{package_name}__{}", self.name);
        let cxx_status_event = format!("StatusEvent_{}", self.name);
        let status_event_matches = format_ident!(
            "ActionClientStatusEvent_matches__{package_name}__{}",
            self.name
        );
        let status_event_get_status = format_ident!(
            "ActionClientStatusEvent_get_status__{package_name}__{}",
            self.name
        );

        let downcast_result =
            format_ident!("action_downcast_result__{package_name}__{}", self.name);
        let downcast_feedback =
            format_ident!("action_downcast_feedback__{package_name}__{}", self.name);
        let downcast_status =
            format_ident!("action_downcast_status__{package_name}__{}", self.name);

        let goal_type_raw = format_ident!("{package_name}__{}_Goal", self.name);
        let feedback_type_raw = format_ident!("{package_name}__{}_Feedback", self.name);
        let result_type_raw = format_ident!("{package_name}__{}_Result", self.name);

        let result_type_raw_str = result_type_raw.to_string();
        let feedback_type_raw_str = feedback_type_raw.to_string();

        let client_backend = format_ident!("ActionClientBackend__{package_name}__{}", self.name);
        let zenoh_client = quote! {
            dora_ros2_bridge::transport::action::zenoh::TypedActionClient<
                ffi::#goal_type_raw, ffi::#result_type_raw, ffi::#feedback_type_raw
            >
        };

        // Pick the ROS2 service mapping based on RMW_IMPLEMENTATION /
        // ROS_DISTRO at codegen time. See dora-rs/dora#449.
        let ros_service_mapping = crate::detect_ros_service_mapping_ident();

        let def = quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_result_event]
            type #result_event_name;
            #[namespace = #package_name]
            #[cxx_name = #cxx_feedback_event]
            type #feedback_event_name;
            #[namespace = #package_name]
            #[cxx_name = #cxx_status_event]
            type #status_event_name;

            #[namespace = #package_name]
            #[cxx_name = matches_goal]
            fn #result_event_matches(self: &#result_event_name, goal_id: &Box<ActionGoalId>) -> bool;

            #[namespace = #package_name]
            #[cxx_name = get_status]
            fn #result_event_get_status(self: &#result_event_name) -> ActionStatusEnum;

            #[namespace = #package_name]
            #[cxx_name = get_result]
            fn #result_event_get_result(self: &#result_event_name) -> &#result_type_raw;

            #[namespace = #package_name]
            #[cxx_name = matches_goal]
            fn #feedback_event_matches(self: &#feedback_event_name, goal_id: &Box<ActionGoalId>) -> bool;

            #[namespace = #package_name]
            #[cxx_name = get_feedback]
            fn #feedback_event_get_feedback(self: &#feedback_event_name) -> &#feedback_type_raw;

            #[namespace = #package_name]
            #[cxx_name = matches_goal]
            fn #status_event_matches(self: &#status_event_name, goal_id: &Box<ActionGoalId>) -> bool;

            #[namespace = #package_name]
            #[cxx_name = get_status]
            fn #status_event_get_status(self: &#status_event_name) -> ActionStatusEnum;

            #[namespace = #package_name]
            #[cxx_name = #cxx_client_name]
            type #client_name;

            #[namespace = #package_name]
            #[cxx_name = #cxx_create_client]
            fn #create_client(
                node: &mut Ros2Node,
                name_space: &str,
                base_name: &str,
                qos:Ros2ActionClientQosPolicies,
                events: &mut CombinedEvents) -> Result<Box<#client_name>>;

            #[namespace = #package_name]
            #[cxx_name = wait_for_action]
            fn #wait_for_action(self: &mut #client_name, node: &Box<Ros2Node>) -> Result<()>;

            #[namespace = #package_name]
            #[cxx_name = send_goal]
            fn #send_goal(self: &mut #client_name, request: #goal_type_raw) -> Result<Box<ActionGoalId>>;

            #[namespace = #package_name]
            #[cxx_name = send_goal_with_timeout]
            fn #send_goal_with_timeout(
                self: &mut #client_name,
                request: #goal_type_raw,
                timeout: u64) -> Result<Box<ActionGoalId>>;

            #[namespace = #package_name]
            #[cxx_name = cancel_goal]
            fn #cancel_goal(self: &mut #client_name, goal_id: &Box<ActionGoalId>) -> Result<()>;

            #[namespace = #package_name]
            #[cxx_name = request_result]
            fn #request_result(self: &mut #client_name, goal_id: &Box<ActionGoalId>) -> Result<()>;

            #[namespace = #package_name]
            #[cxx_name = matches_result]
            fn #result_matches(self: &mut #client_name, event: &CombinedEvent) -> bool;

            #[namespace = #package_name]
            #[cxx_name = matches_feedback]
            fn #feedback_matches(self: &mut #client_name, event: &CombinedEvent) -> bool;

            #[namespace = #package_name]
            #[cxx_name = matches_status]
            fn #status_matches(self: &mut #client_name, event: &CombinedEvent) -> bool;

            #[namespace = #package_name]
            #[cxx_name = downcast_result]
            fn #downcast_result(self: &mut #client_name, event: CombinedEvent) -> Result<Box<#result_event_name>>;

            #[namespace = #package_name]
            #[cxx_name = downcast_feedback]
            fn #downcast_feedback(self: &mut #client_name, event: CombinedEvent) -> Result<Box<#feedback_event_name>>;

            #[namespace = #package_name]
            #[cxx_name = downcast_status]
            fn #downcast_status(self: &mut #client_name, event: CombinedEvent) -> Result<Box<#status_event_name>>;
        };

        let imp = quote! {

            #[allow(non_snake_case)]
            pub fn #create_client(
                node: &mut Ros2Node,
                name_space: &str,
                base_name: &str,
                qos: ffi::Ros2ActionClientQosPolicies,
                events: &mut crate::ffi::CombinedEvents
            ) -> eyre::Result<Box<#client_name>> {
                use futures::StreamExt as _;
                use std::sync::Arc;

                let name = crate::ros2_client::Name::new(name_space, base_name)
                    .map_err(|e| eyre::eyre!("invalid ROS2 name/namespace: {e:?}"))?;
                let (result_tx, result_rx) = flume::bounded::<eyre::Result<#result_event_name>>(1);

                let (backend, feedback_id, status_id) = match &mut node.node {
                    GeneratedNode::Dds(dds_node) => {
                        let client = dds_node.create_action_client::< action :: #self_name >(
                            crate::ros2_client::ServiceMapping:: #ros_service_mapping,
                            &name,
                            &crate::ros2_client::ActionTypeName::new(#package_name, #self_name_str),
                            qos.into(),
                        ).map_err(|e| eyre::eyre!("{e:?}"))?;
                        let client = Arc::new(client);
                        // GetResult request ids this client has sent and not yet matched to a
                        // response, each paired with the goal it belongs to.
                        let pending: Arc<std::sync::Mutex<Vec<(crate::ros2_client::service::RmwRequestId, ffi::ActionGoalId)>>> =
                            Arc::new(std::sync::Mutex::new(Vec::new()));

                        // dora #1972 (same class as #1970): one GetResult response pump that
                        // demuxes by request id. ros2-client's `async_request_result` discards
                        // responses whose id does not match, so a per-goal receiver lets
                        // concurrent goals steal each other's results; one pump avoids that.
                        {
                            let client = Arc::clone(&client);
                            let pending = pending.clone();
                            let result_tx = result_tx.clone();
                            std::thread::spawn(move || {
                                loop {
                                    if result_tx.is_disconnected() {
                                        break;
                                    }
                                    // SAFETY: the `&mut`-via-ptr cast mirrors the feedback stream
                                    // above — `result_client(&mut self)` only needs `&mut` to hand
                                    // back the sub-client, and `receive_response` itself takes &self.
                                    // Only this pump thread receives. The concurrent &self
                                    // `send_request` from `request_result` is data-race-free on the
                                    // ros2-client `Client` (disjoint request/response paths + atomic
                                    // sequence ids) — the same concurrency the #1971 service pump relies on.
                                    let recv = unsafe {
                                        let ptr = Arc::as_ptr(&client)
                                            as *mut crate::ros2_client::action::ActionClient< action :: #self_name >;
                                        (&mut *ptr).result_client().receive_response()
                                    };
                                    match recv {
                                        Ok(Some((req_id, response))) => {
                                            // Route the response to its goal; drop responses we
                                            // have no pending request for (they belong to another
                                            // client sharing the topic).
                                            let goal_id = match pending.lock() {
                                                Ok(mut guard) => guard
                                                    .iter()
                                                    .position(|(pid, _)| *pid == req_id)
                                                    .map(|pos| guard.remove(pos).1),
                                                Err(_) => None,
                                            };
                                            if let Some(goal_id) = goal_id {
                                                let status: ffi::ActionStatusEnum = response.status.into();
                                                let event = #result_event_name {
                                                    goal_id,
                                                    status,
                                                    result: response.result,
                                                };
                                                if result_tx.send(Ok(event)).is_err() {
                                                    break;
                                                }
                                            }
                                        }
                                        Ok(None) => {
                                            std::thread::sleep(std::time::Duration::from_millis(2));
                                        }
                                        Err(e) => {
                                            let _ = result_tx.send(Err(eyre::eyre!(
                                                "failed to receive action result: {e:?}"
                                            )));
                                            std::thread::sleep(std::time::Duration::from_millis(2));
                                        }
                                    }
                                }
                            });
                        }

                        let feedback_id = {
                            let stream = futures_lite::stream::unfold(Arc::clone(&client), |client| async {
                                // SAFETY:
                                // The ros2_client crate only provides &mut access to the feedback subscription, but
                                // the mutable permission is not required at all.
                                // There is no other position that will access the feedback subscription.
                                // There is no modification on the feedback_subscription.
                                // The Arc ensures the client is not dropped while the stream is active.
                                // The async_take() method only requires immutable access to the subscription struct.
                                let item = unsafe {
                                    let ptr = Arc::as_ptr(&client) as *mut crate::ros2_client::action::ActionClient< action :: #self_name>;
                                    let sub = (&mut *ptr).feedback_subscription();
                                    sub.async_take().await
                                };
                                let item = item.map(
                                    |(crate::ros2_client::action::FeedbackMessage {goal_id, feedback}, _msg_info)| {
                                        Box::new(#feedback_event_name {
                                            goal_id: ActionGoalId { id: goal_id },
                                            feedback
                                        })
                                    }
                                ).map_err(|e| eyre::eyre!("Failed to read feedback: {}", e));
                                let item_boxed: Box<dyn std::any::Any + 'static> = Box::new(item);
                                Some((item_boxed, client))
                            });
                            events.events.merge(Box::pin(stream))
                        };

                        let status_id = {
                            let client = Arc::clone(&client);
                            let stream = futures_lite::stream::unfold(client, |client| async {
                                Some((client.async_receive_status().await, client))
                            }).filter_map(|status_res| async {
                                if let Ok(status_arr) = status_res {
                                    Some(futures::stream::iter(status_arr.status_list.into_iter().map(|goal_status| {
                                        let item = #status_event_name {
                                            goal_id: ActionGoalId { id: goal_status.goal_info.goal_id },
                                            status: goal_status.status.into()
                                        };
                                        Box::new(item) as Box<dyn std::any::Any + 'static>
                                    })))
                                } else {
                                    None
                                }
                            }).flatten();
                            events.events.merge(Box::pin(stream))
                        };
                        (#client_backend::Dds { client, pending }, feedback_id, status_id)
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { node: zenoh_node, compatibility } => {
                        use dora_ros2_bridge::transport::{action::zenoh::ActionCallError, zenoh::pubsub::PubSubError};

                        // The feedback subscription's QoS (reliability, durability,
                        // history) is the one the C++ caller chooses; the status topic
                        // always uses the fixed action status QoS. Both queues hold 8
                        // messages whatever the QoS depth.
                        let feedback_qos = neutral_qos(&qos.feedback_subscription)?;
                        let (keys, tokens) = dora_ros2_bridge::transport::action::zenoh::resolve_action_endpoints(
                            zenoh_node.domain(),
                            *compatibility,
                            &zenoh_full_name(zenoh_node, name.to_string()),
                            #package_name,
                            #self_name_str,
                            &feedback_qos,
                            &dora_ros2_bridge::transport::zenoh::compatibility::TypeDescriptionResolver::from_ament_prefix_path(),
                        )?;
                        let client = Arc::new(futures::executor::block_on(<#zenoh_client>::declare(
                            zenoh_node,
                            &keys,
                            tokens,
                            &feedback_qos,
                            ZENOH_ACTION_MAX_PAYLOAD,
                        ))?);

                        // The client holds its subscriptions, so `Closed` doesn't happen
                        // while these streams run; if it ever does, the stream ends
                        // instead of yielding the same error forever.
                        let feedback_id = {
                            let stream = futures_lite::stream::unfold(Arc::clone(&client), |client| async move {
                                let item = match client.recv_feedback().await {
                                    Ok((goal_id, feedback)) => Ok(Box::new(#feedback_event_name {
                                        goal_id: ActionGoalId { id: goal_id },
                                        feedback,
                                    })),
                                    Err(ActionCallError::Topic(PubSubError::Closed)) => return None,
                                    Err(e) => Err(eyre::eyre!("Failed to read feedback: {e}")),
                                };
                                let item_boxed: Box<dyn std::any::Any + 'static> = Box::new(item);
                                Some((item_boxed, client))
                            });
                            events.events.merge(Box::pin(stream))
                        };

                        let status_id = {
                            let stream = futures_lite::stream::unfold(Arc::clone(&client), |client| async move {
                                match client.recv_status().await {
                                    Err(ActionCallError::Topic(PubSubError::Closed)) => None,
                                    status => Some((status, client)),
                                }
                            }).filter_map(|status| async {
                                // Like the DDS stream, skip status messages that fail to decode.
                                status.ok().map(|status_list| futures::stream::iter(status_list.into_iter().map(|goal_status| {
                                    let item = #status_event_name {
                                        goal_id: ActionGoalId { id: goal_status.goal_info.goal_id },
                                        status: goal_status.status.into(),
                                    };
                                    Box::new(item) as Box<dyn std::any::Any + 'static>
                                })))
                            }).flatten();
                            events.events.merge(Box::pin(stream))
                        };

                        (#client_backend::Zenoh { client, result_tx, executor: node.executor.clone(), results_waiting: Default::default() }, feedback_id, status_id)
                    }
                };
                let result_stream = result_rx.into_stream()
                    .map(|v| Box::new(v) as Box<dyn std::any::Any + 'static>);
                let result_id = events.events.merge(Box::pin(result_stream));

                Ok(Box::new(#client_name {
                    backend,
                    result_id,
                    feedback_id,
                    status_id,
                }))
            }

            #[allow(non_camel_case_types)]
            pub struct #client_name {
                backend: #client_backend,
                result_id: u32,
                feedback_id: u32,
                status_id: u32,
            }

            #[allow(non_camel_case_types)]
            enum #client_backend {
                Dds {
                    client: std::sync::Arc<crate::ros2_client::action::ActionClient< action :: #self_name>>,
                    pending: std::sync::Arc<std::sync::Mutex<Vec<(crate::ros2_client::service::RmwRequestId, ffi::ActionGoalId)>>>,
                },
                #[cfg(feature = "rmw-zenoh")]
                Zenoh {
                    client: std::sync::Arc<#zenoh_client>,
                    // `request_result` answers come back through the result stream.
                    result_tx: flume::Sender<eyre::Result<#result_event_name>>,
                    executor: std::sync::Arc<futures::executor::ThreadPool>,
                    /// `request_result` calls still waiting for their goal or for
                    /// their answer to be read, at most MAX_PENDING_REQUESTS, so a
                    /// caller that never reads the result events can't pile up
                    /// tasks holding results.
                    results_waiting: std::sync::Arc<std::sync::atomic::AtomicUsize>,
                },
            }

            impl Drop for #client_name {
                fn drop(&mut self) {
                    // The `request_result` tasks hold their own reference to the
                    // Zenoh client; end the ones still waiting for a goal.
                    #[cfg(feature = "rmw-zenoh")]
                    if let #client_backend::Zenoh { client, .. } = &self.backend {
                        client.close();
                    }
                }
            }

            #[allow(non_camel_case_types)]
            pub struct #result_event_name {
                goal_id: ffi::ActionGoalId,
                status: ffi::ActionStatusEnum,
                result: ffi::#result_type_raw,
            }

            impl #result_event_name {
                #[allow(non_snake_case)]
                pub fn #result_event_matches(&self, goal_id: &Box<ffi::ActionGoalId>) -> bool {
                    self.goal_id.id == goal_id.id
                }
                #[allow(non_snake_case)]
                pub fn #result_event_get_status(&self) -> ffi::ActionStatusEnum {
                    self.status
                }
                #[allow(non_snake_case)]
                pub fn #result_event_get_result(&self) -> &ffi::#result_type_raw {
                    &self.result
                }
            }

            #[allow(non_camel_case_types)]
            pub struct #feedback_event_name {
                goal_id: ffi::ActionGoalId,
                feedback: ffi::#feedback_type_raw,
            }

            impl #feedback_event_name {
                #[allow(non_snake_case)]
                pub fn #feedback_event_matches(&self, goal_id: &Box<ffi::ActionGoalId>) -> bool {
                    self.goal_id.id == goal_id.id
                }
                #[allow(non_snake_case)]
                pub fn #feedback_event_get_feedback(&self) -> &ffi::#feedback_type_raw {
                    &self.feedback
                }
            }

            #[allow(non_camel_case_types)]
            pub struct #status_event_name {
                goal_id: ffi::ActionGoalId,
                status: ffi::ActionStatusEnum,
            }

            impl #status_event_name {
                #[allow(non_snake_case)]
                pub fn #status_event_matches(&self, goal_id: &Box<ffi::ActionGoalId>) -> bool {
                    self.goal_id.id == goal_id.id
                }
                #[allow(non_snake_case)]
                pub fn #status_event_get_status(&self) -> ffi::ActionStatusEnum {
                    self.status
                }
            }

            impl #client_name {
                #[allow(non_snake_case)]
                fn #send_goal(&mut self, request: ffi::#goal_type_raw) -> eyre::Result<Box<ffi::ActionGoalId>> {
                    self.#send_goal_with_timeout(request, 0)
                }

                #[allow(non_snake_case)]
                fn #send_goal_with_timeout(
                    &mut self, request: ffi::#goal_type_raw,
                    timeout: u64)
                -> eyre::Result<Box<ffi::ActionGoalId>> {
                    let timeout = if timeout == 0 {
                        None
                    } else {
                        Some(core::time::Duration::from_nanos(timeout))
                    };

                    let client_ref = match &self.backend {
                        #client_backend::Dds { client, .. } => client,
                        #[cfg(feature = "rmw-zenoh")]
                        #client_backend::Zenoh { client, .. } => {
                            // A timeout of 0 means "wait as long as it takes", as on
                            // DDS; GET_RESULT_TIMEOUT is that "forever" for a Zenoh call.
                            // Unlike DDS, a goal sent while no server is matched fails at
                            // once ("service request timed out"): call `wait_for_action`
                            // first.
                            let timeout = timeout.unwrap_or(dora_ros2_bridge::transport::action::zenoh::GET_RESULT_TIMEOUT);
                            return match futures::executor::block_on(client.send_goal(&request, timeout)) {
                                Ok(goal_id) => Ok(Box::new(ffi::ActionGoalId { id: goal_id })),
                                Err(dora_ros2_bridge::transport::action::zenoh::ActionCallError::Rejected) => {
                                    Err(eyre::eyre!("Goal was rejected by the server."))
                                }
                                Err(e) => Err(eyre::eyre!("Failed to send goal: {e}")),
                            };
                        }
                    };
                    let send_goal_fut = async move {
                        let send_fut = client_ref.async_send_goal(request);
                        futures::pin_mut!(send_fut);
                        if let Some(timeout) = timeout {
                            let timeout = futures_timer::Delay::new(timeout);
                            match futures::future::select(send_fut, timeout).await {
                                futures::future::Either::Left((result, _)) => result.map_err(|e| eyre::eyre!("Failed to send goal: {:?}", e)),
                                futures::future::Either::Right(_) => Err(eyre::eyre!("Timeout")),
                            }
                        } else {
                            send_fut.await.map_err(|e| eyre::eyre!("Failed to send goal: {:?}", e))
                        }
                    };

                    let (goal_id, send_goal_response) = futures::executor::block_on(send_goal_fut)?;

                    if !send_goal_response.accepted {
                        return Err(eyre::eyre!("Goal was rejected by the server."));
                    }

                    Ok(Box::new(ffi::ActionGoalId{ id: goal_id }))
                }

                #[allow(non_snake_case)]
                fn #wait_for_action(&self, node: &Box<Ros2Node>) -> eyre::Result<()> {
                    let client = match &self.backend {
                        #client_backend::Dds { client, .. } => client,
                        #[cfg(feature = "rmw-zenoh")]
                        #client_backend::Zenoh { client, .. } => {
                            let GeneratedNode::Zenoh { node: zenoh_node, .. } = &node.node else {
                                eyre::bail!("action client belongs to a different ROS2 transport");
                            };
                            // Same 20 s budget as the DDS loop below (10 x 2 s).
                            return futures::executor::block_on(client.wait_for_server(
                                zenoh_node,
                                std::time::Instant::now() + std::time::Duration::from_secs(20),
                            ))
                            .map_err(|e| eyre::eyre!("action server not available: {e}"));
                        }
                    };
                    // Wait for every part of the action server, like
                    // `rcl_action_server_is_available`: the goal, result and
                    // cancel services and the feedback and status publishers.
                    // A request sent before its service is matched is lost, so
                    // waiting for the goal service alone let a `request_result`
                    // right after `send_goal` vanish, leaving the client without
                    // a result and the server blocked in `send_result`.
                    let dds_node = node.dds_node()?;
                    let server_ready = async {
                        for _ in 0..10 {
                            let ready = async {
                                futures::join!(
                                    client.goal_client().wait_for_service(dds_node),
                                    client.result_client().wait_for_service(dds_node),
                                    client.cancel_client().wait_for_service(dds_node),
                                    client.feedback_subscription().wait_for_publisher(dds_node),
                                    client.status_subscription().wait_for_publisher(dds_node),
                                )
                            };
                            futures::pin_mut!(ready);
                            let timeout = futures_timer::Delay::new(std::time::Duration::from_secs(2));
                            match futures::future::select(ready, timeout).await {
                                futures::future::Either::Left(_) => {
                                    return Ok(());
                                }
                                futures::future::Either::Right(_) => {
                                    eprintln!("timeout while waiting for action server, retrying");
                                }
                            }
                        }
                        eyre::bail!("action server not available");
                    };
                    futures::executor::block_on(server_ready)
                }

                #[allow(non_snake_case)]
                fn #cancel_goal(&mut self, goal_id: &Box<ffi::ActionGoalId>) -> eyre::Result<()> {
                    // Fire-and-forget cancel. Completion can be tracked
                    // by the caller via the existing status event stream
                    // (the goal transitions to GoalStatusEnum::Canceled
                    // once the server acknowledges).
                    let goal_id = goal_id.id;
                    match &self.backend {
                        #client_backend::Dds { client, .. } => {
                            client.cancel_goal(goal_id)
                                .map_err(|e| eyre::eyre!("Failed to cancel goal: {:?}", e))?;
                        }
                        #[cfg(feature = "rmw-zenoh")]
                        #client_backend::Zenoh { client, executor, .. } => {
                            use futures::task::SpawnExt as _;
                            let client = client.clone();
                            executor.spawn(async move {
                                if let Err(e) = client.cancel_goal(goal_id, SERVICE_RESPONSE_TIMEOUT).await {
                                    eprintln!(concat!("failed to cancel ", #self_name_str, " goal: {}"), e);
                                }
                            }).map_err(|e| eyre::eyre!("Failed to cancel goal: {e}"))?;
                        }
                    }
                    Ok(())
                }

                #[allow(non_snake_case)]
                fn #request_result(&self, goal_id: &Box<ActionGoalId>) -> eyre::Result<()> {
                    // dora #1972 (same class as #1970): register this goal's GetResult
                    // request id -> goal_id under the same lock the result pump uses,
                    // holding it across the synchronous send so a result that arrives
                    // before registration completes is not dropped as "not ours". The
                    // single pump (see the client constructor) then routes responses by
                    // request id, instead of a per-goal receiver that would steal other
                    // goals' responses.
                    let goal_id = (**goal_id).clone();
                    let (client, pending) = match &self.backend {
                        #client_backend::Dds { client, pending } => (client, pending),
                        // Each request waits for its own goal's result, so there
                        // is nothing to demultiplex on Zenoh.
                        #[cfg(feature = "rmw-zenoh")]
                        #client_backend::Zenoh { client, result_tx, executor, results_waiting } => {
                            use futures::task::SpawnExt as _;
                            use std::sync::atomic::Ordering;
                            if results_waiting.fetch_add(1, Ordering::SeqCst) >= MAX_PENDING_REQUESTS {
                                results_waiting.fetch_sub(1, Ordering::SeqCst);
                                return Err(eyre::eyre!(
                                    "{MAX_PENDING_REQUESTS} action result requests are still waiting for their goal or for their answer to be read"
                                ));
                            }
                            // Gives the slot back when the task ends or is dropped
                            // unrun (spawn failed, executor shut down).
                            struct Waiting(std::sync::Arc<std::sync::atomic::AtomicUsize>);
                            impl Drop for Waiting {
                                fn drop(&mut self) {
                                    self.0.fetch_sub(1, Ordering::SeqCst);
                                }
                            }
                            let waiting = Waiting(results_waiting.clone());
                            let client = client.clone();
                            let result_tx = result_tx.clone();
                            return executor.spawn(async move {
                                let _waiting = waiting;
                                match client.get_result(goal_id.id).await {
                                    // The client is gone: nobody reads the answer.
                                    Err(dora_ros2_bridge::transport::action::zenoh::ActionCallError::Closed) => {}
                                    result => {
                                        let result = result
                                            .map(|(status, result)| #result_event_name {
                                                goal_id,
                                                status: status.into(),
                                                result,
                                            })
                                            .map_err(|e| eyre::eyre!("failed to receive action result: {e}"));
                                        let _ = result_tx.send_async(result).await;
                                    }
                                }
                            }).map_err(|e| eyre::eyre!("Failed to request result: {e}"));
                        }
                    };
                    let mut pending = pending
                        .lock()
                        .map_err(|_| eyre::eyre!("action client result state poisoned"))?;
                    let req_id = client
                        .request_result(goal_id.id)
                        .map_err(|e| eyre::eyre!("Failed to request result: {:?}", e))?;
                    pending.push((req_id, goal_id));
                    Ok(())
                }

                #[allow(non_snake_case)]
                fn #result_matches(&self, event: &crate::ffi::CombinedEvent) -> bool {
                    match &event.event.as_ref().0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.result_id => true,
                        _ => false
                    }
                }

                #[allow(non_snake_case)]
                fn #feedback_matches(&self, event: &crate::ffi::CombinedEvent) -> bool {
                    match &event.event.as_ref().0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.feedback_id => true,
                        _ => false
                    }
                }

                #[allow(non_snake_case)]
                fn #status_matches(&self, event: &crate::ffi::CombinedEvent) -> bool {
                    match &event.event.as_ref().0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.status_id => true,
                        _ => false
                    }
                }

                #[allow(non_snake_case)]
                fn #downcast_result(&self, event: crate::ffi::CombinedEvent) -> eyre::Result<Box<#result_event_name>> {
                    use eyre::WrapErr as _;

                    match (*event.event).0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.result_id => {
                            let result = event.event.downcast::<eyre::Result<#result_event_name>>()
                                .map_err(|_| eyre::eyre!("downcast to {} failed", #result_type_raw_str))?;

                            let data = result.with_context(|| format!("failed to receive {} response", #self_name_str))
                                .map_err(|e| eyre::eyre!("{e:?}"))?;
                            Ok(Box::new(data))
                        },
                        _ => eyre::bail!("not a {} response event", #self_name_str),
                    }
                }

                #[allow(non_snake_case)]
                fn #downcast_feedback(self: &mut #client_name, event: crate::ffi::CombinedEvent) -> eyre::Result<Box<#feedback_event_name>> {
                    use eyre::WrapErr;

                    match (*event.event).0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.feedback_id => {
                            let result = event.event.downcast::<eyre::Result<Box<#feedback_event_name>>>()
                                .map_err(|e| eyre::eyre!("downcast to {} failed: {:?}", #feedback_type_raw_str, e))?;

                            let data = result.with_context(|| format!("failed to receive {} feedback", #self_name_str))
                                .map_err(|e| eyre::eyre!("{e:?}"))?;
                            Ok(data)
                        },
                        _ => eyre::bail!("not a {} feedback event", #self_name_str),
                    }
                }

                #[allow(non_snake_case)]
                fn #downcast_status(self: &mut #client_name, event: crate::ffi::CombinedEvent) -> eyre::Result<Box<#status_event_name>> {
                    match (*event.event).0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.status_id => {
                            let result = event.event.downcast::<#status_event_name>()
                                .map_err(|_| eyre::eyre!("downcast to {} failed", "action_msgs__GoalStatus"))?;

                            Ok(result)
                        },
                        _ => eyre::bail!("not a {} status event", #self_name_str),
                    }
                }
            }
        };
        (def, imp)
    }

    /// C++ FFI for hosting a ROS2 action **server**. Mirrors the daemon's
    /// `run_action_server` FSM: a dedicated-thread pump (beside the spinner)
    /// receives goals, auto-accepts + starts executing them, stores the
    /// `ExecutingGoalHandle` keyed by goal id, and forwards `(goal_id, goal)` to
    /// the merged event stream. The C++ side does `matches` -> `downcast` (-> a
    /// `GoalEvent` exposing `get_goal_id()` + `get_goal()`), then
    /// `publish_feedback(goal_id, feedback)` and `send_result(goal_id, status,
    /// result)`, which look the handle back up by goal id. `ActionGoalId` (the
    /// existing shared cxx type) carries the goal id across the FFI.
    ///
    /// The handle map is bounded two ways so a C++ server that abandons goals
    /// cannot wedge: goals past `ACTION_MAX_CONCURRENT_GOALS` in flight are rejected,
    /// and a goal not finished within `ACTION_GOAL_TIMEOUT` is aborted + evicted.
    /// `send_result` is itself time-bounded so a never-arriving client
    /// get_result request can't freeze the caller's event loop.
    ///
    /// That is the DDS backend. On native Zenoh, `TypedActionServer` does the
    /// same job: its pump runs on the node's executor, it applies the same two
    /// limits (a new goal aborts the goals past `ACTION_GOAL_TIMEOUT`, feedback
    /// or not), and `send_result` stores the result and returns without
    /// waiting for the client.
    pub fn cxx_action_server_creation_functions(
        &self,
        package_name: &str,
    ) -> (impl ToTokens, impl ToTokens) {
        let server_name = format_ident!("ActionServer__{package_name}__{}", self.name);
        let cxx_server_name = format!("Actionserver_{}", self.name);
        let create_server = format_ident!("new_ActionServer__{package_name}__{}", self.name);
        let cxx_create_server = format!("create_{}_action_server", self.name);

        let goal_event_name = format_ident!("ActionServerGoalEvent__{package_name}__{}", self.name);
        let cxx_goal_event = format!("GoalEvent_{}", self.name);
        let goal_event_get_goal_id = format_ident!(
            "ActionServerGoalEvent_get_goal_id__{package_name}__{}",
            self.name
        );
        let goal_event_get_goal = format_ident!(
            "ActionServerGoalEvent_get_goal__{package_name}__{}",
            self.name
        );

        let matches = format_ident!("action_server_matches__{package_name}__{}", self.name);
        let downcast = format_ident!("action_server_downcast__{package_name}__{}", self.name);
        let publish_feedback = format_ident!(
            "action_server_publish_feedback__{package_name}__{}",
            self.name
        );
        let send_result = format_ident!("action_server_send_result__{package_name}__{}", self.name);

        let self_name = format_ident!("{}", self.name);
        let self_name_str = &self.name;

        let goal_type_raw = format_ident!("{package_name}__{}_Goal", self.name);
        let feedback_type_raw = format_ident!("{package_name}__{}_Feedback", self.name);
        let result_type_raw = format_ident!("{package_name}__{}_Result", self.name);
        let goal_type_raw_str = goal_type_raw.to_string();

        let ros_service_mapping = crate::detect_ros_service_mapping_ident();

        let server_backend = format_ident!("ActionServerBackend__{package_name}__{}", self.name);
        let zenoh_server = quote! {
            dora_ros2_bridge::transport::action::zenoh::TypedActionServer<
                ffi::#goal_type_raw, ffi::#result_type_raw, ffi::#feedback_type_raw
            >
        };

        let def = quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_goal_event]
            type #goal_event_name;
            #[namespace = #package_name]
            #[cxx_name = get_goal_id]
            fn #goal_event_get_goal_id(self: &#goal_event_name) -> Box<ActionGoalId>;
            #[namespace = #package_name]
            #[cxx_name = get_goal]
            fn #goal_event_get_goal(self: &#goal_event_name) -> &#goal_type_raw;

            #[namespace = #package_name]
            #[cxx_name = #cxx_server_name]
            type #server_name;
            #[namespace = #package_name]
            #[cxx_name = #cxx_create_server]
            fn #create_server(node: &mut Ros2Node, name_space: &str, base_name: &str, qos: Ros2QosPolicies, events: &mut CombinedEvents) -> Result<Box<#server_name>>;
            #[namespace = #package_name]
            #[cxx_name = matches]
            fn #matches(self: &#server_name, event: &CombinedEvent) -> bool;
            #[namespace = #package_name]
            #[cxx_name = downcast]
            fn #downcast(self: &#server_name, event: CombinedEvent) -> Result<Box<#goal_event_name>>;
            #[namespace = #package_name]
            #[cxx_name = publish_feedback]
            fn #publish_feedback(self: &mut #server_name, goal_id: &Box<ActionGoalId>, feedback: #feedback_type_raw) -> Result<()>;
            #[namespace = #package_name]
            #[cxx_name = send_result]
            fn #send_result(self: &mut #server_name, goal_id: &Box<ActionGoalId>, status: ActionStatusEnum, result: #result_type_raw) -> Result<()>;
        };

        let imp = quote! {
            #[allow(non_snake_case)]
            pub fn #create_server(node: &mut Ros2Node, name_space: &str, base_name: &str, qos: ffi::Ros2QosPolicies, events: &mut crate::ffi::CombinedEvents) -> eyre::Result<Box<#server_name>> {
                use futures::StreamExt as _;
                use std::sync::Arc;

                let name = crate::ros2_client::Name::new(name_space, base_name)
                    .map_err(|e| eyre::eyre!("invalid ROS2 name/namespace: {e:?}"))?;
                let (backend, stream_id) = match &mut node.node {
                    GeneratedNode::Dds(dds_node) => {
                        // One QoS profile for all five action endpoints (mirrors the
                        // native Rust action-server example).
                        let q: crate::rustdds::QosPolicies = qos.into();
                        let server = dds_node.create_action_server::< action :: #self_name >(
                            crate::ros2_client::ServiceMapping:: #ros_service_mapping,
                            &name,
                            &crate::ros2_client::ActionTypeName::new(#package_name, #self_name_str),
                            crate::ros2_client::action::ActionServerQosPolicies {
                                goal_service: q.clone(),
                                result_service: q.clone(),
                                cancel_service: q.clone(),
                                feedback_publisher: q.clone(),
                                status_publisher: q,
                            },
                        ).map_err(|e| eyre::eyre!("{e:?}"))?;
                        let server = Arc::new(crate::ros2_client::action::AsyncActionServer::new(server));
                        // goal id (uuid string) -> (handle, accept time) for dispatching
                        // feedback/result. The accept time bounds the map: a goal whose
                        // C++ handler never calls `send_result` (crash, early return,
                        // abandoned goal) is aborted after a timeout so the server cannot
                        // wedge at the in-flight cap.
                        let executing: Arc<std::sync::Mutex<std::collections::HashMap<String, (crate::ros2_client::action::ExecutingGoalHandle< ffi :: #goal_type_raw >, std::time::Instant)>>> =
                            Arc::new(std::sync::Mutex::new(std::collections::HashMap::new()));
                        let (goal_tx, goal_rx) = flume::bounded(16);
                        let stream = goal_rx.into_stream().map(|v: eyre::Result<(ffi::ActionGoalId, ffi::#goal_type_raw)>| Box::new(v) as Box<dyn std::any::Any + 'static>);
                        let stream_id = events.events.merge(Box::pin(stream));

                        // Goal pump: receive -> (bound check) accept -> execute -> store
                        // handle -> forward to C++. Runs on a dedicated thread (not the
                        // node executor) so it never competes with the spinner that
                        // announces + drives the action services -- on a thread-starved
                        // executor the spinner could otherwise be blocked, leaving the
                        // server undiscoverable.
                        {
                            let server = server.clone();
                            let executing = executing.clone();
                            std::thread::spawn(move || futures::executor::block_on(async move {
                                loop {
                                    if goal_tx.is_disconnected() {
                                        break;
                                    }
                                    let new_goal = match server.receive_new_goal().await {
                                        Ok(h) => h,
                                        Err(e) => {
                                            eprintln!("action server: failed to receive goal: {e:?}");
                                            // Back off so a persistent receive error can't
                                            // busy-spin / flood the log.
                                            futures_timer::Delay::new(std::time::Duration::from_millis(2)).await;
                                            continue;
                                        }
                                    };
                                    // Evict (abort) goals abandoned past the timeout so the
                                    // map can't wedge at the cap, then reject if still full.
                                    let now = std::time::Instant::now();
                                    let stale: Vec<_> = match executing.lock() {
                                        Ok(mut map) => {
                                            let keys: Vec<String> = map
                                                .iter()
                                                .filter(|(_, (_, t))| now.duration_since(*t) >= ACTION_GOAL_TIMEOUT)
                                                .map(|(k, _)| k.clone())
                                                .collect();
                                            keys.into_iter().filter_map(|k| map.remove(&k).map(|(h, _)| h)).collect()
                                        }
                                        Err(_) => Vec::new(),
                                    };
                                    for handle in stale {
                                        eprintln!("action server: goal abandoned past timeout, aborting");
                                        let _ = server.abort_executing_goal(handle).await;
                                    }
                                    let full = executing.lock().map(|m| m.len() >= ACTION_MAX_CONCURRENT_GOALS).unwrap_or(false);
                                    if full {
                                        eprintln!("action server: {ACTION_MAX_CONCURRENT_GOALS} goals in flight, rejecting new goal");
                                        let _ = server.reject_goal(new_goal).await;
                                        continue;
                                    }
                                    let goal_data = server.get_new_goal(new_goal.clone());
                                    let accepted = match server.accept_goal(new_goal).await {
                                        Ok(h) => h,
                                        Err(e) => {
                                            eprintln!("action server: failed to accept goal: {e:?}");
                                            continue;
                                        }
                                    };
                                    let handle = match server.start_executing_goal(accepted).await {
                                        Ok(h) => h,
                                        Err(e) => {
                                            eprintln!("action server: failed to start executing goal: {e:?}");
                                            continue;
                                        }
                                    };
                                    let goal_id = handle.goal_id();
                                    let goal = match goal_data {
                                        Some(g) => g,
                                        None => {
                                            eprintln!("action server: goal had no data, aborting");
                                            let _ = server.abort_executing_goal(handle).await;
                                            continue;
                                        }
                                    };
                                    if let Ok(mut map) = executing.lock() {
                                        map.insert(goal_id.uuid.to_string(), (handle, std::time::Instant::now()));
                                    }
                                    if goal_tx.send(Ok((ffi::ActionGoalId { id: goal_id }, goal))).is_err() {
                                        break;
                                    }
                                }
                            }));
                        }
                        (#server_backend::Dds { server, executing }, stream_id)
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { node: zenoh_node, compatibility } => {
                        use futures::task::SpawnExt as _;

                        let qos = neutral_qos(&qos)?;
                        let (keys, tokens) = dora_ros2_bridge::transport::action::zenoh::resolve_action_endpoints(
                            zenoh_node.domain(),
                            *compatibility,
                            &zenoh_full_name(zenoh_node, name.to_string()),
                            #package_name,
                            #self_name_str,
                            &qos,
                            &dora_ros2_bridge::transport::zenoh::compatibility::TypeDescriptionResolver::from_ament_prefix_path(),
                        )?;
                        // Same limits as the DDS server. The pump answers the action's
                        // services and stops when the server object is dropped, so the
                        // action leaves the ROS graph.
                        let (server, pump) = futures::executor::block_on(<#zenoh_server>::declare(
                            zenoh_node,
                            &keys,
                            tokens,
                            &qos,
                            dora_ros2_bridge::transport::action::zenoh::ActionServerOptions {
                                max_goals: ACTION_MAX_CONCURRENT_GOALS,
                                goal_timeout: ACTION_GOAL_TIMEOUT,
                            },
                        ))?;
                        node.executor.spawn(pump).map_err(|e| eyre::eyre!("failed to spawn action server pump: {e}"))?;
                        let stream = server.goals().clone().into_stream().map(|(goal_id, goal)| {
                            let event: eyre::Result<(ffi::ActionGoalId, ffi::#goal_type_raw)> =
                                Ok((ffi::ActionGoalId { id: goal_id }, goal));
                            Box::new(event) as Box<dyn std::any::Any + 'static>
                        });
                        let stream_id = events.events.merge(Box::pin(stream));
                        (#server_backend::Zenoh(server), stream_id)
                    }
                };

                Ok(Box::new(#server_name {
                    backend,
                    stream_id,
                }))
            }

            #[allow(non_camel_case_types)]
            pub struct #server_name {
                backend: #server_backend,
                stream_id: u32,
            }

            #[allow(non_camel_case_types)]
            enum #server_backend {
                Dds {
                    server: std::sync::Arc<crate::ros2_client::action::AsyncActionServer< action :: #self_name >>,
                    executing: std::sync::Arc<std::sync::Mutex<std::collections::HashMap<String, (crate::ros2_client::action::ExecutingGoalHandle< ffi :: #goal_type_raw >, std::time::Instant)>>>,
                },
                #[cfg(feature = "rmw-zenoh")]
                Zenoh(#zenoh_server),
            }

            #[allow(non_camel_case_types)]
            pub struct #goal_event_name {
                goal_id: ffi::ActionGoalId,
                goal: ffi::#goal_type_raw,
            }

            impl #goal_event_name {
                #[allow(non_snake_case)]
                fn #goal_event_get_goal_id(&self) -> Box<ffi::ActionGoalId> {
                    Box::new(self.goal_id.clone())
                }
                #[allow(non_snake_case)]
                fn #goal_event_get_goal(&self) -> &ffi::#goal_type_raw {
                    &self.goal
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
                fn #downcast(&self, event: crate::ffi::CombinedEvent) -> eyre::Result<Box<#goal_event_name>> {
                    match (*event.event).0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.stream_id => {
                            let result = event.event.downcast::<eyre::Result<(ffi::ActionGoalId, ffi::#goal_type_raw)>>()
                                .map_err(|_| eyre::eyre!("downcast to {} failed", #goal_type_raw_str))?;
                            let (goal_id, goal) = (*result).map_err(|e| eyre::eyre!("{e:?}"))?;
                            Ok(Box::new(#goal_event_name { goal_id, goal }))
                        }
                        _ => eyre::bail!("not a {} goal event", #self_name_str),
                    }
                }

                #[allow(non_snake_case)]
                fn #publish_feedback(&mut self, goal_id: &Box<ffi::ActionGoalId>, feedback: ffi::#feedback_type_raw) -> eyre::Result<()> {
                    let (server, executing) = match &self.backend {
                        #server_backend::Dds { server, executing } => (server, executing),
                        #[cfg(feature = "rmw-zenoh")]
                        #server_backend::Zenoh(server) => {
                            return futures::executor::block_on(server.publish_feedback(goal_id.id, &feedback))
                                .map_err(|e| eyre::eyre!("failed to publish feedback: {e}"));
                        }
                    };
                    let key = goal_id.id.uuid.to_string();
                    let handle = executing
                        .lock()
                        .map_err(|_| eyre::eyre!("action server goal state poisoned"))?
                        .get(&key)
                        .map(|(handle, _)| handle.clone())
                        .ok_or_else(|| eyre::eyre!("no executing goal with id {key}"))?;
                    futures::executor::block_on(server.publish_feedback(handle, feedback))
                        .map_err(|e| eyre::eyre!("failed to publish feedback: {e:?}"))?;
                    Ok(())
                }

                #[allow(non_snake_case)]
                fn #send_result(&mut self, goal_id: &Box<ffi::ActionGoalId>, status: ffi::ActionStatusEnum, result: ffi::#result_type_raw) -> eyre::Result<()> {
                    let (server, executing) = match &self.backend {
                        #server_backend::Dds { server, executing } => (server, executing),
                        // Stores the result and answers the clients already
                        // waiting for it; later `get_result` requests are answered
                        // from the stored result, so this doesn't block.
                        #[cfg(feature = "rmw-zenoh")]
                        #server_backend::Zenoh(server) => {
                            let status = match status {
                                ffi::ActionStatusEnum::Succeeded => crate::ros2_client::action::GoalStatusEnum::Succeeded,
                                ffi::ActionStatusEnum::Canceled => crate::ros2_client::action::GoalStatusEnum::Canceled,
                                _ => crate::ros2_client::action::GoalStatusEnum::Aborted,
                            };
                            return futures::executor::block_on(server.finish(goal_id.id, status, &result))
                                .map_err(|e| eyre::eyre!("failed to send result: {e}"));
                        }
                    };
                    let key = goal_id.id.uuid.to_string();
                    // Clone the handle but KEEP the map entry until the send
                    // succeeds: on a timeout/error the goal stays recoverable (the
                    // caller can retry `send_result`) and evictable (the pump
                    // aborts it after ACTION_GOAL_TIMEOUT) rather than being orphaned.
                    let handle = executing
                        .lock()
                        .map_err(|_| eyre::eyre!("action server goal state poisoned"))?
                        .get(&key)
                        .map(|(handle, _)| handle.clone())
                        .ok_or_else(|| eyre::eyre!("no executing goal with id {key}"))?;
                    let end_status = match status {
                        ffi::ActionStatusEnum::Succeeded => crate::ros2_client::action::GoalEndStatus::Succeeded,
                        ffi::ActionStatusEnum::Canceled => crate::ros2_client::action::GoalEndStatus::Canceled,
                        _ => crate::ros2_client::action::GoalEndStatus::Aborted,
                    };
                    // `send_result_response` waits for the client's get_result
                    // request; bound it (like the daemon's ACTION_RESULT_TIMEOUT)
                    // so a missing/stalled request can't freeze the caller's event
                    // loop forever.
                    let send = server.send_result_response(handle, end_status, result);
                    futures::executor::block_on(async {
                        futures::pin_mut!(send);
                        let timeout = futures_timer::Delay::new(std::time::Duration::from_secs(300));
                        match futures::future::select(send, timeout).await {
                            futures::future::Either::Left((r, _)) => {
                                r.map_err(|e| eyre::eyre!("failed to send result: {e:?}"))
                            }
                            futures::future::Either::Right(_) => {
                                Err(eyre::eyre!("timed out waiting to send result for goal {key}"))
                            }
                        }
                    })?;
                    // Only drop the goal once the result was delivered.
                    let _ = executing.lock().map(|mut m| m.remove(&key));
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

    fn format_valid_rust(tokens: impl ToTokens) -> String {
        crate::format_token_stream(tokens.into_token_stream())
    }

    // #2027: the generated action client/server `create` fns used to
    // `Name::new(..).unwrap()`; they now propagate via `?`. Guard that the
    // generated code (a) still parses and (b) routes the `Name::new` error
    // through `map_err` rather than panic.
    #[test]
    fn action_creation_functions_propagate_name_errors() {
        let action = Action {
            package: "test_pkg".to_string(),
            name: "Fibonacci".to_string(),
            goal: empty_msg("Fibonacci_Goal"),
            result: empty_msg("Fibonacci_Result"),
            feedback: empty_msg("Fibonacci_Feedback"),
        };
        let needle = "invalid ROS2 name/namespace"; // must propagate via `?`, not panic
        let (_decl, imp) = action.cxx_action_creation_functions("test_pkg");
        assert!(format_valid_rust(imp).contains(needle), "client create fn");
        let (_decl, imp) = action.cxx_action_server_creation_functions("test_pkg");
        assert!(format_valid_rust(imp).contains(needle), "server create fn");
    }

    /// The `wait_for_service` / `wait_for_publisher` calls a function makes on
    /// the parts of an action server, as `(accessor, wait)` pairs, e.g.
    /// `("goal_client", "wait_for_service")` for
    /// `client.goal_client().wait_for_service(node)`. Read from the syntax
    /// tree, so it doesn't depend on formatting, and it follows `let`
    /// bindings (also through parentheses and the tail of a block or
    /// `unsafe` block) and the arguments of macros like `futures::join!`. A wait whose future is thrown away
    /// (`let _ = ..;` or `..;`, also in parentheses or braces) doesn't count:
    /// nothing waits for it.
    #[derive(Default)]
    struct EndpointWaits {
        bindings: std::collections::HashMap<String, String>,
        /// The thrown-away calls of the statements being walked.
        thrown_away: Vec<*const syn::ExprMethodCall>,
        waits: std::collections::BTreeSet<(String, String)>,
    }

    impl EndpointWaits {
        const ACCESSORS: [&str; 5] = [
            "goal_client",
            "result_client",
            "cancel_client",
            "feedback_subscription",
            "status_subscription",
        ];

        /// `expr` without its parentheses and block or `unsafe` braces.
        fn unwrap(expr: &syn::Expr) -> &syn::Expr {
            match expr {
                syn::Expr::Paren(inner) => Self::unwrap(&inner.expr),
                syn::Expr::Unsafe(syn::ExprUnsafe { block, .. })
                | syn::Expr::Block(syn::ExprBlock { block, .. }) => match block.stmts.last() {
                    Some(syn::Stmt::Expr(tail, None)) => Self::unwrap(tail),
                    _ => expr,
                },
                _ => expr,
            }
        }

        /// The action server part that `expr` evaluates to, if any.
        fn accessor(&self, expr: &syn::Expr) -> Option<String> {
            match Self::unwrap(expr) {
                syn::Expr::MethodCall(call) => Self::ACCESSORS
                    .into_iter()
                    .find(|accessor| call.method == accessor)
                    .map(str::to_string),
                syn::Expr::Path(path) => self
                    .bindings
                    .get(&path.path.get_ident()?.to_string())
                    .cloned(),
                _ => None,
            }
        }
    }

    impl<'ast> syn::visit::Visit<'ast> for EndpointWaits {
        fn visit_stmt(&mut self, stmt: &'ast syn::Stmt) {
            let discarded = match stmt {
                syn::Stmt::Expr(expr, Some(_)) => Some(expr),
                syn::Stmt::Local(syn::Local {
                    pat: syn::Pat::Wild(_),
                    init: Some(init),
                    ..
                }) => Some(&*init.expr),
                _ => None,
            };
            if let Some(syn::Expr::MethodCall(call)) = discarded.map(Self::unwrap) {
                self.thrown_away.push(call);
                syn::visit::visit_stmt(self, stmt);
                self.thrown_away.pop();
            } else {
                syn::visit::visit_stmt(self, stmt);
            }
        }

        fn visit_local(&mut self, local: &'ast syn::Local) {
            syn::visit::visit_local(self, local);
            if let (syn::Pat::Ident(name), Some(init)) = (&local.pat, &local.init)
                && let Some(accessor) = self.accessor(&init.expr)
            {
                self.bindings.insert(name.ident.to_string(), accessor);
            }
        }

        fn visit_expr_method_call(&mut self, call: &'ast syn::ExprMethodCall) {
            syn::visit::visit_expr_method_call(self, call);
            if (call.method == "wait_for_service" || call.method == "wait_for_publisher")
                && !self.thrown_away.contains(&std::ptr::from_ref(call))
                && let Some(accessor) = self.accessor(&call.receiver)
            {
                self.waits.insert((accessor, call.method.to_string()));
            }
        }

        fn visit_macro(&mut self, mac: &'ast syn::Macro) {
            use syn::parse::Parser as _;
            let args = syn::punctuated::Punctuated::<syn::Expr, syn::Token![,]>::parse_terminated;
            if let Ok(args) = args.parse2(mac.tokens.clone()) {
                args.iter().for_each(|arg| self.visit_expr(arg));
            }
        }
    }

    /// A `request_result` sent before the result service is matched is lost on
    /// DDS: the client never gets a result and the server waits in `send_result`.
    /// So `wait_for_action` has to wait for every part of the action server, like
    /// `rcl_action_server_is_available`, not only for the goal service.
    #[test]
    fn wait_for_action_waits_for_every_action_endpoint() {
        use syn::visit::Visit as _;
        let action = Action {
            package: "test_pkg".to_string(),
            name: "Fibonacci".to_string(),
            goal: empty_msg("Fibonacci_Goal"),
            result: empty_msg("Fibonacci_Result"),
            feedback: empty_msg("Fibonacci_Feedback"),
        };
        let (_decl, imp) = action.cxx_action_creation_functions("test_pkg");
        let file: syn::File = syn::parse2(imp.into_token_stream()).expect("generated code parses");
        let wait_for_action = file
            .items
            .iter()
            .filter_map(|item| match item {
                syn::Item::Impl(imp) => Some(&imp.items),
                _ => None,
            })
            .flatten()
            .find_map(|item| match item {
                syn::ImplItem::Fn(f) if f.sig.ident == "wait_for__test_pkg__Fibonacci" => Some(f),
                _ => None,
            })
            .expect("generated wait_for_action");
        let mut found = EndpointWaits::default();
        found.visit_impl_item_fn(wait_for_action);
        let expected = [
            ("goal_client", "wait_for_service"),
            ("result_client", "wait_for_service"),
            ("cancel_client", "wait_for_service"),
            ("feedback_subscription", "wait_for_publisher"),
            ("status_subscription", "wait_for_publisher"),
        ]
        .map(|(accessor, wait)| (accessor.to_string(), wait.to_string()));
        assert_eq!(found.waits, expected.into());
    }

    // #3764: the generated action client and server used to require DDS
    // (`dds_node_mut()`), so they failed on the native Zenoh transport.
    #[test]
    fn action_client_and_server_have_a_zenoh_backend() {
        let action = Action {
            package: "test_pkg".to_string(),
            name: "Fibonacci".to_string(),
            goal: empty_msg("Fibonacci_Goal"),
            result: empty_msg("Fibonacci_Result"),
            feedback: empty_msg("Fibonacci_Feedback"),
        };
        let (_decl, imp) = action.cxx_action_creation_functions("test_pkg");
        let client = format_valid_rust(imp);
        assert!(client.contains("GeneratedNode::Zenoh"), "client create fn");
        assert!(client.contains("TypedActionClient"), "client create fn");
        assert!(!client.contains("dds_node_mut"), "client create fn");
        assert!(
            client.contains("impl Drop for") && client.contains("client.close()"),
            "a dropped client must close its Zenoh client"
        );
        let (_decl, imp) = action.cxx_action_server_creation_functions("test_pkg");
        let server = format_valid_rust(imp);
        assert!(server.contains("GeneratedNode::Zenoh"), "server create fn");
        assert!(server.contains("TypedActionServer"), "server create fn");
        assert!(!server.contains("dds_node_mut"), "server create fn");
    }
}
