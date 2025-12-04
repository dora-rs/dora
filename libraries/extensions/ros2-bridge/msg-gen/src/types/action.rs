use heck::SnakeCase;
use quote::{ToTokens, format_ident, quote};
use syn::Ident;

use super::{Member, Message, Service, primitives::*};

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

    fn send_goal_srv(&self) -> Service {
        let common = format!("{}_SendGoal", self.name);

        let request = Message {
            package: self.package.clone(),
            name: format!("{common}_Request"),
            members: vec![
                goal_id_type(),
                Member {
                    name: "goal".into(),
                    r#type: NamespacedType {
                        package: self.package.clone(),
                        namespace: "action".into(),
                        name: format!("{}_Goal", self.name),
                    }
                    .into(),
                    default: None,
                },
            ],
            constants: vec![],
        };
        let response = Message {
            package: self.package.clone(),
            name: format!("{common}_Response"),
            members: vec![
                Member {
                    name: "accepted".into(),
                    r#type: BasicType::Bool.into(),
                    default: None,
                },
                Member {
                    name: "stamp".into(),
                    r#type: NamespacedType {
                        package: "builtin_interfaces".into(),
                        namespace: "msg".into(),
                        name: "Time".into(),
                    }
                    .into(),
                    default: None,
                },
            ],
            constants: vec![],
        };

        Service {
            package: self.package.clone(),
            name: common,
            request,
            response,
        }
    }

    fn get_result_srv(&self) -> Service {
        let common = format!("{}_GetResult", self.name);

        let request = Message {
            package: self.package.clone(),
            name: format!("{common}_Request"),
            members: vec![goal_id_type()],
            constants: vec![],
        };
        let response = Message {
            package: self.package.clone(),
            name: format!("{common}_Response"),
            members: vec![
                Member {
                    name: "status".into(),
                    r#type: BasicType::I8.into(),
                    default: None,
                },
                Member {
                    name: "result".into(),
                    r#type: NamespacedType {
                        package: self.package.clone(),
                        namespace: "action".into(),
                        name: format!("{}_Result", self.name),
                    }
                    .into(),
                    default: None,
                },
            ],
            constants: vec![],
        };

        Service {
            package: self.package.clone(),
            name: common,
            request,
            response,
        }
    }

    fn feedback_message_msg(&self) -> Message {
        Message {
            package: self.package.clone(),
            name: format!("{}_FeedbackMessage", self.name),
            members: vec![
                goal_id_type(),
                Member {
                    name: "feedback".into(),
                    r#type: NamespacedType {
                        package: self.package.clone(),
                        namespace: "action".into(),
                        name: format!("{}_Feedback", self.name),
                    }
                    .into(),
                    default: None,
                },
            ],
            constants: vec![],
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

                let client = node.node.create_action_client::< action :: #self_name >(
                    crate::ros2_client::ServiceMapping::Enhanced,
                    &crate::ros2_client::Name::new(name_space, base_name).unwrap(),
                    &crate::ros2_client::ActionTypeName::new(#package_name, #self_name_str),
                    qos.into(),
                ).map_err(|e| eyre::eyre!("{e:?}"))?;
                let client = Arc::new(client);
                let (result_tx, result_rx) = flume::bounded(1);

                let result_stream = result_rx.into_stream()
                    .map(|v: eyre::Result<_>| Box::new(v) as Box<dyn std::any::Any + 'static>);
                let result_id = events.events.merge(Box::pin(result_stream));

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

                Ok(Box::new(#client_name {
                    client,
                    result_tx: Arc::new(result_tx),
                    executor: node.executor.clone(),
                    result_id,
                    feedback_id,
                    status_id,
                }))
            }

            #[allow(non_camel_case_types)]
            pub struct #client_name {
                client: std::sync::Arc<crate::ros2_client::action::ActionClient< action :: #self_name>>,
                result_tx: std::sync::Arc<crate::flume::Sender<eyre::Result<#result_event_name>>>,
                executor: std::sync::Arc<crate::futures::executor::ThreadPool>,
                result_id: u32,
                feedback_id: u32,
                status_id: u32,
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

                    let client_ref = &self.client;
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
                    // SAFETY:
                    // The ros2_client crate only provides &mut access to the goal service client, but
                    // the mutable permission is not required at all.
                    // There is no modification on the goal service client.
                    // The other method will access the goal service client is send_goal() which is not required
                    // mutable access as well.
                    // The wait_for_service() method only requires immutable access to the subscription struct.
                    let service_client = unsafe {
                        let ptr = std::sync::Arc::as_ptr(&self.client)
                            as *mut crate::ros2_client::action::ActionClient< action :: #self_name >;
                        (&mut *ptr).goal_client()
                    };
                    let service_ready = async {
                        for _ in 0..10 {
                            let ready = service_client.wait_for_service(&node.node);
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
                fn #request_result(&self, goal_id: &Box<ActionGoalId>) -> eyre::Result<()> {
                    use eyre::WrapErr;
                    use futures::task::SpawnExt as _;

                    let request_result_handle = {
                        let client_ref = std::sync::Arc::clone(&self.client);
                        let result_tx = self.result_tx.clone();
                        let goal_id = *goal_id.clone();
                        async move {
                            let resp = client_ref.async_request_result(goal_id.id).await;
                            let resp = resp.map(|(status, response)| {
                                let status: ffi::ActionStatusEnum = status.into();
                                #result_event_name {
                                    goal_id: goal_id.clone(),
                                    status,
                                    result: response
                                }
                            }).map_err(|e| eyre::eyre!("Failed to request result: {:?}", e));
                            if result_tx.send_async(resp).await.is_err() {
                                tracing::warn!("failed to send action result");
                            }
                        }
                    };
                    self.executor.spawn(request_result_handle)
                        .context("failed to spawn response task").map_err(|e| eyre::eyre!("{e:?}"))?;
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
        let action_type = format_ident!("{}", self.name);
        let goal_type = format_ident!("{}_Goal", self.name);
        let result_type = format_ident!("{}_Result", self.name);
        let feedback_type = format_ident!("{}_Feedback", self.name);
        let send_goal_type = format_ident!("{}_SendGoal", self.name);
        let get_result_type = format_ident!("{}_GetResult", self.name);
        let feedback_message_type = format_ident!("{}_FeedbackMessage", self.name);

        let goal_body = self.goal.token_stream();
        let result_body = self.result.token_stream();
        let feedback_body = self.feedback.token_stream();
        let send_goal_body = self.send_goal_srv().token_stream();
        let get_result_body = self.get_result_srv().token_stream();
        let feedback_message_body = self.feedback_message_msg().token_stream();

        quote! {
            use std::os::raw::c_void;

            pub use self::goal::*;
            pub use self::result::*;
            pub use self::feedback::*;
            pub use self::send_goal::*;
            pub use self::get_result::*;
            pub use self::feedback_message::*;

            #[allow(non_camel_case_types)]
            #[derive(std::fmt::Debug)]
            pub struct #action_type;


            impl crate::_core::ActionT for #action_type {
                type Goal = #goal_type;
                type Result = #result_type;
                type Feedback = #feedback_type;
                type SendGoal = #send_goal_type;
                type GetResult = #get_result_type;
                type FeedbackMessage = #feedback_message_type;
            }

            mod goal {
                #goal_body
            }  // mod goal

            mod result {
                #result_body
            }  // mod result

            mod feedback {
                #feedback_body
            }  // mod feedback

            mod send_goal {
                #send_goal_body
            }  // mod send_goal

            mod get_result {
                #get_result_body
            }  // mod get_result

            mod feedback_message {
                #feedback_message_body
            }  // mod feedback_message

            #[cfg(test)]
            mod test {
                use super::*;
                use crate::_core::ActionT;

                #[test]
                fn test_type_support() {
                    let ptr = #action_type::type_support();
                    assert!(!ptr.is_null());
                }
            }
        }
    }
}

fn goal_id_type() -> Member {
    Member {
        name: "goal_id".into(),
        r#type: NamespacedType {
            package: "unique_identifier_msgs".into(),
            namespace: "msg".into(),
            name: "UUID".into(),
        }
        .into(),
        default: None,
    }
}
