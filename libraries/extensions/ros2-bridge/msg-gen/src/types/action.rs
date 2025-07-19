use heck::SnakeCase;
use quote::{format_ident, quote, ToTokens};
use syn::Ident;

use super::{primitives::*, Member, Message, Service};

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

            pub use super::super::ffi::#goal_type_raw as #goal_type;
            pub use super::super::ffi::#result_type_raw as #result_type;
            pub use super::super::ffi::#feedback_type_raw as #feedback_type;

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
        let cxx_client_name = format_ident!("Actionclient_{}", self.name);
        let create_client = format_ident!("new_ActionClient__{package_name}__{}", self.name);
        let cxx_create_client = format!("create_action_client_{package_name}_{}", self.name);

        let package = format_ident!("{package_name}");
        let self_name = format_ident!("{}", self.name);
        let self_name_str = &self.name;

        let send_goal = format_ident!("send_goal__{package_name}__{}", self.name);
        let cxx_send_goal = "send_goal".to_string();

        let matches = format_ident!("matches__{package_name}__{}", self.name);
        let cxx_matches = format_ident!("matches");
        let downcast = format_ident!("action_downcast__{package_name}__{}", self.name);
        let cxx_downcast = format_ident!("downcast");

        let goal_type_raw = format_ident!("{package_name}__{}_Goal", self.name);
        let result_type_raw = format_ident!("{package_name}__{}_Result", self.name);

        let result_type_raw_str = result_type_raw.to_string();

        let def = quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_client_name]
            type #client_name;

            #[cxx_name = #cxx_create_client]
            fn #create_client(self: &mut Ros2Node, name_space: &str, base_name: &str, qos:Ros2ActionClientQosPolicies, events: &mut CombinedEvents) -> Result<Box<#client_name>>;

            #[namespace = #package_name]
            #[cxx_name = #cxx_send_goal]
            fn #send_goal(self: &mut #client_name, request: #goal_type_raw) -> Result<()>;

            #[namespace = #package_name]
            #[cxx_name = #cxx_matches]
            fn #matches(self: &mut #client_name, event: &CombinedEvent) -> bool;

            #[namespace = #package_name]
            #[cxx_name = #cxx_downcast]
            fn #downcast(self: &mut #client_name, event: CombinedEvent) -> Result<#result_type_raw>;
        };

        let imp = quote! {
            impl Ros2Node {
                #[allow(non_snake_case)]
                pub fn #create_client(&mut self, name_space: &str, base_name: &str, qos: ffi::Ros2ActionClientQosPolicies, events: &mut crate::ffi::CombinedEvents) -> eyre::Result<Box<#client_name>> {
                    use futures::StreamExt as _;

                    let client = self.node.create_action_client::< #package :: action :: #self_name >(
                        ros2_client::ServiceMapping::Enhanced,
                        &ros2_client::Name::new(name_space, base_name).unwrap(),
                        &ros2_client::ActionTypeName::new(#package_name, #self_name_str),
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
                client: std::sync::Arc<ros2_client::action::ActionClient< #package :: action :: #self_name>>,
                response_tx: std::sync::Arc<flume::Sender<eyre::Result<ffi::#result_type_raw>>>,
                executor: std::sync::Arc<futures::executor::ThreadPool>,
                stream_id: u32,
            }

            impl #client_name {

                #[allow(non_snake_case)]
                fn #send_goal(&mut self, request: ffi::#goal_type_raw) -> eyre::Result<()> {
                    use eyre::WrapErr;
                    use futures::task::SpawnExt as _;
                    use futures::stream::StreamExt;
                    use futures::executor::block_on;
                    use std::sync::Arc;

                    let client_arc = Arc::new(self.client.clone());

                    let client_ref = Arc::clone(&client_arc);
                    let send_goal = async move {
                        client_ref.async_send_goal(request.clone()).await
                    };

                    let handle = self.executor.spawn_with_handle(send_goal)
                        .map_err(|e| eyre::eyre!("{e:?}"))?;

                    let (goal_id, send_goal_response) = block_on(handle)
                        .map_err(|e| eyre::eyre!("{e:?}"))?;

                    if !send_goal_response.accepted {
                        return Err(eyre::eyre!("Goal was rejected by the server."));
                    }

                    let feedback_handle = {
                        let client_ref = Arc::clone(&client_arc);
                        async move {
                            let feedback_stream = client_ref.feedback_stream(goal_id);
                            feedback_stream.for_each(|feedback| async {
                                match feedback {
                                    Ok(feedback) => println!("Received feedback: {:?}", feedback),
                                    Err(e) => eprintln!("Error while receive feedback: {:?}", e),
                                }
                            }).await;
                        }
                    };

                    self.executor.spawn(feedback_handle).context("failed to spawn feedback task")?;


                    let status_handle = {
                        let client_ref = Arc::clone(&client_arc);
                        async move {
                            let status_stream = client_ref.status_stream(goal_id);
                            status_stream.for_each(|status| async {
                                match status {
                                    Ok(status) => println!("Status update: {:?}", status),
                                    Err(e) => eprintln!("Error receiving status update: {:?}", e),
                                }
                            }).await;
                        }
                    };

                    self.executor.spawn(status_handle).context("failed to spawn status task")?;

                    let request_result_handle = {
                        let client_ref = Arc::clone(&client_arc);
                        let response_tx = self.response_tx.clone();
                        async move {
                            match client_ref.async_request_result(goal_id).await {
                                Ok((_status, result)) => {
                                    let response = Ok(result);
                                    if response_tx.send_async(response).await.is_err() {
                                        tracing::warn!("failed to send action result");
                                    }
                                },
                                Err(e) => {
                                    tracing::error!("Failed to receive response for request {goal_id:?}: {:?}", e);
                                }
                            }
                        }
                    };

                    self.executor.spawn(request_result_handle).context("failed to spawn response task").map_err(|e| eyre::eyre!("{e:?}"))?;
                    Ok(())
                }

                #[allow(non_snake_case)]
                fn #matches(&self, event: &crate::ffi::CombinedEvent) -> bool {
                    match &event.event.as_ref().0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.stream_id => true,
                        _ => false
                    }
                }

                #[allow(non_snake_case)]
                fn #downcast(&self, event: crate::ffi::CombinedEvent) -> eyre::Result<ffi::#result_type_raw> {
                    use eyre::WrapErr;

                    match (*event.event).0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.stream_id => {
                            let result = event.event.downcast::<eyre::Result<ffi::#result_type_raw>>()
                                .map_err(|_| eyre::eyre!("downcast to {} failed", #result_type_raw_str))?;

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
