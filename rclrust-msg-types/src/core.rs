use crate::primitives::*;
use crate::{ConstantType, MemberType};

/// A member of a structure
#[derive(Debug, Clone, PartialEq)]
pub struct Member {
    /// The name of the member
    pub name: String,
    /// The type of the member
    pub r#type: MemberType,
    /// The default value of the member (optional)
    pub default: Option<String>,
}

/// A constant definition
#[derive(Debug, Clone)]
pub struct Constant {
    /// The name of the constant
    pub name: String,
    /// The type of the constant
    pub r#type: ConstantType,
    /// The value of the constant
    pub value: String,
}

/// A message definition
#[derive(Debug, Clone)]
pub struct Message {
    /// The package name
    pub package: String,
    /// The name of the message
    pub name: String,
    /// The list of the members
    pub members: Vec<Member>,
    /// The list of the constants
    pub constants: Vec<Constant>,
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
    pub fn send_goal_srv(&self) -> Service {
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

    pub fn get_result_srv(&self) -> Service {
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

    pub fn feedback_message_msg(&self) -> Message {
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
