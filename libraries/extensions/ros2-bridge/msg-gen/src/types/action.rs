use heck::SnakeCase;
use quote::{format_ident, quote, ToTokens};

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
    fn send_goal_srv(&self) -> Service {
        let common = format!("{}_SendGoal", self.name);

        let request = Message {
            package: self.package.clone(),
            name: format!("{}_Request", common),
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
            name: format!("{}_Response", common),
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
            name: format!("{}_Request", common),
            members: vec![goal_id_type()],
            constants: vec![],
        };
        let response = Message {
            package: self.package.clone(),
            name: format!("{}_Response", common),
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
