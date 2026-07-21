use quote::{ToTokens, format_ident, quote};
use syn::Ident;

use super::{ConstantType, MemberType};

pub fn dds_name(package: &str, name: &str) -> String {
    format!("{package}::msg::dds_::{name}_")
}

/// A member of a structure
#[derive(Debug, Clone)]
pub struct Member {
    /// The name of the member
    pub name: String,
    /// The type of the member
    pub r#type: MemberType,
    /// The default value of the member (optional)
    pub default: Option<Vec<String>>,
}

impl Member {
    fn name_token(&self) -> impl ToTokens {
        if RUST_KEYWORDS.contains(&self.name.as_str()) {
            format_ident!("{}_", self.name)
        } else {
            format_ident!("{}", self.name)
        }
    }

    fn rust_type_def(&self, package: &str) -> impl ToTokens {
        let name = self.name_token();
        let (attr, type_) = self.r#type.type_tokens(package);
        quote! { #attr pub #name: #type_, }
    }

    fn default_value(&self) -> impl ToTokens {
        let name = self.name_token();
        self.default.as_ref().map_or_else(
            || quote! { #name: crate::_core::InternalDefault::_default(), },
            |default| {
                let default = self.r#type.value_tokens(default);
                quote! { #name: #default, }
            },
        )
    }
}

/// A constant definition
#[derive(Debug, Clone)]
pub struct Constant {
    /// The name of the constant
    pub name: String,
    /// The type of the constant
    pub r#type: ConstantType,
    /// The value of the constant
    pub value: Vec<String>,
}

impl Constant {
    fn token_stream(&self) -> impl ToTokens {
        let name = format_ident!("{}", self.name);
        let type_ = self.r#type.type_tokens();
        let value = self.r#type.value_tokens(&self.value);
        quote! { pub const #name: #type_ = #value; }
    }

    fn cxx_method_def_token_stream(&self, struct_name: &str, package_name: &str) -> impl ToTokens {
        let name = format_ident!("const_{package_name}__{struct_name}_{}", self.name);
        let cxx_name = format_ident!("const_{struct_name}_{}", self.name);
        let type_ = self.r#type.type_tokens();
        quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_name]
            pub fn #name () -> #type_;
        }
    }

    fn cxx_method_impl_token_stream(&self, struct_raw_name: &Ident) -> impl ToTokens {
        let const_name = format_ident!("{}", self.name);
        let name = format_ident!("const_{struct_raw_name}_{}", self.name);
        let type_ = self.r#type.type_tokens();
        quote! {
            #[allow(non_snake_case, dead_code)]
            fn #name () -> #type_ { ffi::#struct_raw_name::#const_name }
        }
    }
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

impl Message {
    pub fn struct_token_stream(
        &self,
        package_name: &str,
        gen_cxx_bridge: bool,
    ) -> (impl ToTokens, impl ToTokens) {
        let cxx_name = format_ident!("{}", self.name);
        let struct_raw_name = format_ident!("{package_name}__{}", self.name);
        let struct_raw_vec_name = format_ident!("{package_name}_Vec_{}", self.name);

        let rust_type_def_inner = self.members.iter().map(|m| m.rust_type_def(&self.package));
        let constants_def_inner = self.constants.iter().map(|c| c.token_stream());
        let cxx_const_def_inner = self
            .constants
            .iter()
            .map(|c| c.cxx_method_def_token_stream(&self.name, package_name));
        let cxx_const_impl_inner = self
            .constants
            .iter()
            .map(|c| c.cxx_method_impl_token_stream(&struct_raw_name));
        let rust_type_default_inner = self.members.iter().map(|m| m.default_value());

        let (attributes, cxx_consts) = if gen_cxx_bridge {
            let attributes = quote! {
                #[namespace = #package_name]
                #[cxx_name = #cxx_name]
            };
            let consts = quote! {
                extern "Rust" {
                    #(#cxx_const_def_inner)*
                }
            };
            (attributes, consts)
        } else {
            (quote! {}, quote! {})
        };

        let msg_converter = match struct_raw_name.to_string().as_str() {
            "action_msgs__GoalStatus" => quote! {
                impl From<crate::ros2_client::action_msgs::GoalStatus> for ffi::action_msgs__GoalStatus {
                    fn from(status: crate::ros2_client::action_msgs::GoalStatus) -> Self {
                        use crate::ros2_client::action_msgs::GoalStatus;
                        let GoalStatus {goal_info, status} = status;
                        Self { goal_info: goal_info.into(), status: status as i8 }
                    }
                }
            },
            "action_msgs__GoalInfo" => quote! {
                impl From<crate::ros2_client::action_msgs::GoalInfo> for ffi::action_msgs__GoalInfo {
                    fn from(info: crate::ros2_client::action_msgs::GoalInfo) -> Self {
                        use crate::ros2_client::action_msgs::GoalInfo;
                        let GoalInfo {goal_id, stamp} = info;
                        Self { goal_id: goal_id.into(), stamp: stamp.into()}
                    }
                }
            },
            "unique_identifier_msgs__UUID" => quote! {
                impl From<crate::ros2_client::unique_identifier_msgs::UUID> for ffi::unique_identifier_msgs__UUID {
                    fn from(uuid: crate::ros2_client::unique_identifier_msgs::UUID) -> Self {
                        use crate::ros2_client::unique_identifier_msgs::UUID;
                        let UUID {uuid} = uuid;
                        Self { uuid: *uuid.as_bytes() }
                    }
                }
            },
            "builtin_interfaces__Time" => quote! {
                impl From<crate::ros2_client::builtin_interfaces::Time> for ffi::builtin_interfaces__Time {
                    fn from(time: crate::ros2_client::builtin_interfaces::Time) -> Self {
                        let t = time.to_nanos();
                        let quot = t / 1_000_000_000;
                        let rem = t % 1_000_000_000;

                        // https://doc.rust-lang.org/reference/expressions/operator-expr.html#arithmetic-and-logical-binary-operators
                        // "Rust uses a remainder defined with truncating division.
                        // Given remainder = dividend % divisor,
                        // the remainder will have the same sign as the dividend."

                        if rem >= 0 {
                            // positive time, no surprise here
                            // OR, negative time, but a whole number of seconds, fractional part is zero
                            Self {
                                // Saturate seconds to i32. This is different from C++ implementation
                                // in rclcpp, which just uses
                                // `ret.sec = static_cast<std::int32_t>(result.quot)`.
                                sec: if quot > (i32::MAX as i64) {
                                    tracing::warn!("rcl_interfaces::Time conversion overflow");
                                    i32::MAX
                                } else if quot < (i32::MIN as i64) {
                                    tracing::warn!("rcl_interfaces::Time conversion underflow");
                                    i32::MIN
                                } else {
                                    quot as i32
                                },
                                nanosec: rem as u32,
                            }
                        } else {
                            // Now `t` is negative AND `rem` is non-zero.
                            // We do some non-obvious arithmetic:

                            // saturate whole seconds
                            let quot_sat = if quot >= (i32::MIN as i64) {
                                quot as i32
                            } else {
                                tracing::warn!("rcl_interfaces::Time conversion underflow");
                                i32::MIN
                            };

                            // Now, `rem` is between -999_999_999 and -1, inclusive.
                            // Case rem = 0 is included in the positive branch.
                            //
                            // Adding 1_000_000_000 will make it positive, so cast to u32 is ok.
                            //
                            // It is also the right thing to do, because
                            // * 0.0 sec = 0 sec and 0 nanosec
                            // * -0.000_000_001 sec = -1 sec and 999_999_999 nanosec
                            // * ...
                            // * -0.99999999999 sec = -1 sec and 000_000_001 nanosec
                            // * -1.0           sec = -1 sec and 0 nanosec
                            // * -1.00000000001 sec = -2 sec and 999_999_999 nanosec
                            Self {
                                sec: quot_sat - 1, // note -1
                                nanosec: (1_000_000_000 + rem) as u32,
                            }
                        }
                    }
                }
            },
            _ => quote! {},
        };

        let def = if self.members.is_empty() {
            quote! {
                #[allow(non_camel_case_types)]
                #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
                #attributes
                pub struct #struct_raw_name {
                    #[serde(skip)]
                    pub(super) _dummy: u8,
                }

                #cxx_consts
            }
        } else {
            quote! {
                #[allow(non_camel_case_types)]
                #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
                #attributes
                pub struct #struct_raw_name {
                    #(#rust_type_def_inner)*
                }

                #[allow(non_camel_case_types)]
                #[allow(dead_code)]
                pub struct #struct_raw_vec_name {
                    data: Vec<#struct_raw_name>
                }

                #cxx_consts
            }
        };
        let default = if self.members.is_empty() {
            quote! {
                Self {
                    _dummy: 0,
                }
            }
        } else {
            quote! {
                Self {
                    #(#rust_type_default_inner)*
                }
            }
        };
        let impls = quote! {
            impl ffi::#struct_raw_name {
                #(#constants_def_inner)*

            }

            #msg_converter

            impl crate::_core::InternalDefault for ffi::#struct_raw_name {
                fn _default() -> Self {
                    #default
                }
            }

            impl std::default::Default for ffi::#struct_raw_name {
                #[inline]
                fn default() -> Self {
                    crate::_core::InternalDefault::_default()
                }
            }

            impl crate::ros2_client::Message for ffi::#struct_raw_name {}

            #(#cxx_const_impl_inner)*
        };

        (def, impls)
    }

    pub fn topic_def(&self, package_name: &str) -> (impl ToTokens, impl ToTokens) {
        if self.members.is_empty() {
            return (quote! {}, quote! {});
        };

        let topic_name = format_ident!("Topic__{package_name}__{}", self.name);
        let cxx_topic_name = format!("Topic_{}", self.name);
        let create_topic = format_ident!("new__Topic__{package_name}__{}", self.name);
        let cxx_create_topic = format!("create_topic_{package_name}_{}", self.name);

        let publisher_name = format_ident!("Publisher__{package_name}__{}", self.name);
        let cxx_publisher_name = format!("Publisher_{}", self.name);
        let create_publisher = format_ident!("new__Publisher__{package_name}__{}", self.name);
        let cxx_create_publisher = "create_publisher".to_string();

        let struct_raw_name = format_ident!("{package_name}__{}", self.name);
        let struct_raw_name_str = struct_raw_name.to_string();
        let self_name = &self.name;

        let publish = format_ident!("publish__{package_name}__{}", self.name);
        let cxx_publish = "publish";

        let subscription_name = format_ident!("Subscription__{package_name}__{}", self.name);
        let subscription_name_str = subscription_name.to_string();
        let cxx_subscription_name = format!("Subscription_{}", self.name);
        let create_subscription = format_ident!("new__Subscription__{package_name}__{}", self.name);
        let cxx_create_subscription = "create_subscription";

        let matches = format_ident!("matches__{package_name}__{}", self.name);
        let cxx_matches = "matches";
        let downcast = format_ident!("downcast__{package_name}__{}", self.name);
        let cxx_downcast = "downcast";

        let def = quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_topic_name]
            type #topic_name;
            #[cxx_name = #cxx_create_topic]
            fn #create_topic(node: &Ros2Node, name_space: &str, base_name: &str, qos: Ros2QosPolicies) -> Result<Box<#topic_name>>;
            #[cxx_name = #cxx_create_publisher]
            fn #create_publisher(node: &mut Ros2Node, topic: &Box<#topic_name>, qos: Ros2QosPolicies) -> Result<Box<#publisher_name>>;
            #[cxx_name = #cxx_create_subscription]
            fn #create_subscription(node: &mut Ros2Node, topic: &Box<#topic_name>, qos: Ros2QosPolicies, events: &mut CombinedEvents) -> Result<Box<#subscription_name>>;

            #[namespace = #package_name]
            #[cxx_name = #cxx_publisher_name]
            type #publisher_name;
            #[namespace = #package_name]
            #[cxx_name = #cxx_publish]
            fn #publish(self: &mut #publisher_name, message: #struct_raw_name) -> Result<()>;

            #[namespace = #package_name]
            #[cxx_name = #cxx_subscription_name]
            type #subscription_name;

            #[namespace = #package_name]
            #[cxx_name = #cxx_matches]
            fn #matches(self: &#subscription_name, event: &CombinedEvent) -> bool;
            #[namespace = #package_name]
            #[cxx_name = #cxx_downcast]
            fn #downcast(self: &#subscription_name, event: CombinedEvent) -> Result<#struct_raw_name>;
        };
        let imp = quote! {
            #[allow(non_camel_case_types)]
            pub enum #topic_name {
                Dds(rustdds::Topic),
                #[cfg(feature = "rmw-zenoh")]
                Zenoh {
                    name: String,
                    identity: dora_ros2_bridge::transport::zenoh::compatibility::RosTypeIdentity,
                },
            }

            #[allow(non_snake_case)]
            pub fn #create_topic(node: &Ros2Node, name_space: &str, base_name: &str, qos: ffi::Ros2QosPolicies) -> eyre::Result<Box<#topic_name>> {
                let name = crate::ros2_client::Name::new(name_space, base_name).map_err(|e| eyre::eyre!(e))?;
                let type_name = crate::ros2_client::MessageTypeName::new(#package_name, #self_name);
                let topic = match &node.node {
                    GeneratedNode::Dds(node) => #topic_name::Dds(node.create_topic(&name, type_name, &qos.into())?),
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { compatibility, .. } => {
                        let identity = dora_ros2_bridge::transport::zenoh::compatibility::resolve_message(
                            *compatibility,
                            #package_name,
                            #self_name,
                            &dora_ros2_bridge::transport::zenoh::compatibility::TypeDescriptionResolver::from_ament_prefix_path(),
                        )?;
                        #topic_name::Zenoh { name: name.to_string(), identity }
                    }
                };
                Ok(Box::new(topic))
            }

            #[allow(non_snake_case)]
            pub fn #create_publisher(node: &mut Ros2Node, topic: &Box<#topic_name>, qos: ffi::Ros2QosPolicies) -> eyre::Result<Box<#publisher_name>> {
                let publisher = match (&mut node.node, &**topic) {
                    (GeneratedNode::Dds(node), #topic_name::Dds(topic)) => {
                        #publisher_name::Dds(node.create_publisher(topic, Some(qos.into()))?)
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    (GeneratedNode::Zenoh { node, .. }, #topic_name::Zenoh { name, identity }) => {
                        let neutral = neutral_qos(&qos)?;
                        let key = dora_ros2_bridge::transport::zenoh::keyexpr::DataKey::new(node.domain(), name, identity)?;
                        let token = dora_ros2_bridge::transport::zenoh::keyexpr::TopicToken {
                            name: name.clone(),
                            type_name: identity.dds_name.clone(),
                            type_hash: identity.key_hash_component(),
                            qos: dora_ros2_bridge::transport::zenoh::qos::ZenohQosMapping::from_ros_qos(&neutral).to_string(),
                        };
                        #publisher_name::Zenoh(futures::executor::block_on(
                            dora_ros2_bridge::transport::zenoh::pubsub::NodePublisher::declare(node, key.as_str(), token, &neutral)
                        )?)
                    }
                    _ => eyre::bail!("topic belongs to a different ROS2 transport"),
                };
                Ok(Box::new(publisher))
            }

            #[allow(non_snake_case)]
            pub fn #create_subscription(node: &mut Ros2Node, topic: &Box<#topic_name>, qos: ffi::Ros2QosPolicies, events: &mut crate::ffi::CombinedEvents) -> eyre::Result<Box<#subscription_name>> {
                let stream: std::pin::Pin<Box<dyn futures::Stream<Item = Box<dyn std::any::Any>> + Send>> = match (&mut node.node, &**topic) {
                    (GeneratedNode::Dds(node), #topic_name::Dds(topic)) => {
                        let subscription = node.create_subscription::<ffi::#struct_raw_name>(topic, Some(qos.into()))?;
                        Box::pin(futures_lite::stream::unfold(subscription, |sub| async {
                            let item: Result<ffi::#struct_raw_name, String> = sub.async_take().await
                                .map(|(data, _)| data).map_err(|error| format!("{error:?}"));
                            Some((Box::new(item) as Box<dyn std::any::Any>, sub))
                        }))
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    (GeneratedNode::Zenoh { node, .. }, #topic_name::Zenoh { name, identity }) => {
                        let neutral = neutral_qos(&qos)?;
                        let key = dora_ros2_bridge::transport::zenoh::keyexpr::DataKey::new(node.domain(), name, identity)?;
                        let token = dora_ros2_bridge::transport::zenoh::keyexpr::TopicToken {
                            name: name.clone(), type_name: identity.dds_name.clone(),
                            type_hash: identity.key_hash_component(),
                            qos: dora_ros2_bridge::transport::zenoh::qos::ZenohQosMapping::from_ros_qos(&neutral).to_string(),
                        };
                        let decoder = std::sync::Arc::new(|payload: &[u8]| {
                            dora_ros2_bridge::transport::zenoh::deserialize_cdr::<ffi::#struct_raw_name>(payload)
                                .map_err(|error| dora_ros2_bridge::transport::zenoh::pubsub::PubSubError::Decode(error.to_string()))
                        });
                        let subscription = futures::executor::block_on(
                            dora_ros2_bridge::transport::zenoh::pubsub::NodeSubscription::declare(node, key.as_str(), token, &neutral, 32, 16 * 1024 * 1024, decoder)
                        )?;
                        Box::pin(futures_lite::stream::unfold(subscription, |sub| async {
                            let item: Result<ffi::#struct_raw_name, String> = sub.recv_async().await
                                .map(|(data, _)| data).map_err(|error| error.to_string());
                            Some((Box::new(item) as Box<dyn std::any::Any>, sub))
                        }))
                    }
                    _ => eyre::bail!("topic belongs to a different ROS2 transport"),
                };
                let id = events.events.merge(Box::pin(stream));

                Ok(Box::new(#subscription_name { id }))
            }

            #[allow(non_camel_case_types)]
            pub enum #publisher_name {
                Dds(crate::ros2_client::Publisher<ffi::#struct_raw_name>),
                #[cfg(feature = "rmw-zenoh")]
                Zenoh(dora_ros2_bridge::transport::zenoh::pubsub::NodePublisher),
            }

            impl #publisher_name {
                #[allow(non_snake_case)]
                fn #publish(&mut self, message: ffi::#struct_raw_name) -> eyre::Result<()> {
                    match self {
                        Self::Dds(publisher) => publisher.publish(message).map_err(|e| eyre::eyre!("publish failed: {e:?}")),
                        #[cfg(feature = "rmw-zenoh")]
                        Self::Zenoh(publisher) => {
                            let payload = dora_ros2_bridge::transport::zenoh::serialize_cdr(&message)?;
                            futures::executor::block_on(publisher.publish(&payload))?;
                            Ok(())
                        }
                    }
                }
            }

            #[allow(non_camel_case_types)]
            pub struct #subscription_name {
                id: u32,
            }

            impl #subscription_name {
                #[allow(non_snake_case)]
                fn #matches(&self, event: &crate::ffi::CombinedEvent) -> bool {
                    match &event.event.as_ref().0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.id  => true,
                        _ => false
                    }
                }
                #[allow(non_snake_case)]
                fn #downcast(&self, event: crate::ffi::CombinedEvent) -> eyre::Result<ffi::#struct_raw_name> {
                    match (*event.event).0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.id  => {
                            let result = event.event.downcast::<Result<ffi::#struct_raw_name, String>>()
                                .map_err(|_| eyre::eyre!("downcast to {} failed", #struct_raw_name_str))?;
                            result.map_err(|error| eyre::eyre!("failed to receive {} event: {error}", #subscription_name_str))
                        },
                        _ => eyre::bail!("not a {} event", #subscription_name_str),
                    }
                }
            }
        };
        (def, imp)
    }

    pub fn alias_token_stream(&self, package_name: &Ident) -> impl ToTokens + use<> {
        let cxx_name = format_ident!("{}", self.name);
        let struct_raw_name = format_ident!("{package_name}__{}", self.name);

        if self.members.is_empty() {
            quote! {}
        } else {
            quote! {
                pub use super::ffi::#struct_raw_name as #cxx_name;
            }
        }
    }
}

/// Keywords in Rust
///
/// <https://doc.rust-lang.org/reference/keywords.html>
const RUST_KEYWORDS: [&str; 51] = [
    // Strict keywords
    "as", "break", "const", "continue", "crate", "else", "enum", "extern", "false", "fn", "for",
    "if", "impl", "in", "let", "loop", "match", "mod", "move", "mut", "pub", "ref", "return",
    "self", "Self", "static", "struct", "super", "trait", "true", "type", "unsafe", "use", "where",
    "while", //
    // Strict keywords (2018+)
    "async", "await", "dyn", //
    // Reserved keywords
    "abstract", "become", "box", "do", "final", "macro", "override", "priv", "typeof", "unsized",
    "virtual", "yield", //
    // Reserved keywords (2018+)
    "try",
];
