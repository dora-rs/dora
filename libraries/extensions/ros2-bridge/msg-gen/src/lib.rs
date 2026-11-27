// Based on https://github.com/rclrust/rclrust/tree/3a48dbb8f23a3d67d3031351da3ed236a354f039/rclrust-msg-gen

#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

use std::path::Path;

use proc_macro2::Ident;
use quote::{format_ident, quote};

pub mod parser;
pub mod type_description;
pub mod types;

pub use crate::parser::get_packages;
use crate::types::Package;

/// Format a generated token stream as Rust source.
///
/// Uses `syn` + `prettyplease` directly (with prettyplease's `verbatim`
/// feature enabled in `Cargo.toml`) rather than going through `rust-format`.
/// The generated `extern "C++"` blocks contain foreign type aliases such as
/// `type CombinedEvents = crate::ffi::CombinedEvents;`, which `syn` represents
/// as `ForeignItem::Verbatim`; prettyplease prints those instead of panicking
/// only when the `verbatim` feature is on. See dora-rs/dora#2452.
pub fn format_token_stream(tokens: proc_macro2::TokenStream) -> String {
    let syntax_tree =
        syn::parse2::<syn::File>(tokens).expect("generated ROS2 bindings must parse as valid Rust");
    prettyplease::unparse(&syntax_tree)
}

/// Pick the `ros2_client::ServiceMapping` variant for the user's ROS2 middleware.
///
/// Reads `RMW_IMPLEMENTATION` (primary) and `ROS_DISTRO` (fallback). Falls
/// back to `Enhanced` with a warning if neither env var gives a usable
/// answer — `Enhanced` is the historical default and keeps existing builds
/// working when env is unset.
///
/// Restores the logic from `dora-rs/dora@e2c1370f`, which was lost during
/// the 1.0 consolidation. See dora-rs/dora#449.
pub fn detect_ros_service_mapping_ident() -> Ident {
    detect_ros_service_mapping_ident_from(
        std::env::var("RMW_IMPLEMENTATION"),
        std::env::var("ROS_DISTRO"),
    )
}

/// Pure inner version of [`detect_ros_service_mapping_ident`]: takes the
/// env-var results as arguments so it can be unit-tested without mutating
/// the process environment.
fn detect_ros_service_mapping_ident_from(
    rmw_implementation: Result<String, std::env::VarError>,
    ros_distro: Result<String, std::env::VarError>,
) -> Ident {
    let enhanced = || format_ident!("Enhanced");
    let cyclone = || format_ident!("Cyclone");

    match rmw_implementation {
        Ok(middleware) => match middleware.as_str() {
            "rmw_fastrtps_cpp" => return enhanced(),
            "rmw_cyclonedds_cpp" => return cyclone(),
            other => {
                eprintln!(
                    "cargo:warning=unknown RMW_IMPLEMENTATION `{other}`, \
                     falling back to ServiceMapping::Enhanced (see dora-rs/dora#449)"
                );
                return enhanced();
            }
        },
        Err(std::env::VarError::NotUnicode(_)) => {
            eprintln!(
                "cargo:warning=RMW_IMPLEMENTATION is not valid unicode, \
                 falling back to ServiceMapping::Enhanced"
            );
            return enhanced();
        }
        Err(std::env::VarError::NotPresent) => {}
    }

    ros_distro.map_or_else(
        |_| enhanced(),
        |distro| match distro.as_str() {
            "humble" | "iron" | "jazzy" | "kilted" | "rolling" => enhanced(),
            "galactic" => cyclone(),
            other => {
                eprintln!(
                    "cargo:warning=unknown ROS_DISTRO `{other}`, \
                     falling back to ServiceMapping::Enhanced (see dora-rs/dora#449)"
                );
                enhanced()
            }
        },
    )
}

#[allow(clippy::cognitive_complexity)]
pub fn generate_package(package: &Package, create_cxx_bridge: bool) -> proc_macro2::TokenStream {
    let mut package = package.clone();
    package.sort_messages();
    let package = &package;
    let mut shared_type_defs = Vec::new();
    let mut message_struct_impls = Vec::new();
    let mut message_topic_defs = Vec::new();
    let mut message_topic_impls = Vec::new();
    let mut service_defs = Vec::new();
    let mut service_impls = Vec::new();
    let mut service_creation_defs = Vec::new();
    let mut service_creation_impls = Vec::new();

    let mut action_defs = Vec::new();
    let mut action_impls = Vec::new();
    let mut action_creation_defs = Vec::new();
    let mut action_creation_impls = Vec::new();

    for message in &package.messages {
        let (def, imp) = message.struct_token_stream(&package.name, create_cxx_bridge);
        shared_type_defs.push(def);
        message_struct_impls.push(imp);
        if create_cxx_bridge {
            let (topic_def, topic_impl) = message.topic_def(&package.name);
            message_topic_defs.push(topic_def);
            message_topic_impls.push(topic_impl);
        }
    }

    for service in &package.services {
        let (def, imp) = service.struct_token_stream(&package.name, create_cxx_bridge);
        service_defs.push(def);
        service_impls.push(imp);
        if create_cxx_bridge {
            // Wrap each opaque `impl ToTokens` in `quote!` so client and server
            // creation functions share one `Vec<TokenStream>` element type
            // (distinct `impl Trait` returns are otherwise distinct opaque types).
            let (service_creation_def, service_creation_impl) =
                service.cxx_service_creation_functions(&package.name);
            service_creation_defs.push(quote! { #service_creation_def });
            service_creation_impls.push(quote! { #service_creation_impl });
            let (server_creation_def, server_creation_impl) =
                service.cxx_service_server_creation_functions(&package.name);
            service_creation_defs.push(quote! { #server_creation_def });
            service_creation_impls.push(quote! { #server_creation_impl });
        }
    }

    for action in &package.actions {
        let (def, imp) = action.struct_token_stream(&package.name, create_cxx_bridge);
        action_defs.push(def);
        action_impls.push(imp);
        if create_cxx_bridge {
            let (action_creation_def, action_creation_impl) =
                action.cxx_action_creation_functions(&package.name);
            let action_creation_def = quote! { #action_creation_def };
            let action_creation_impl = quote! { #action_creation_impl };
            action_creation_defs.push(action_creation_def);
            action_creation_impls.push(action_creation_impl);
            let (action_server_def, action_server_impl) =
                action.cxx_action_server_creation_functions(&package.name);
            action_creation_defs.push(quote! { #action_server_def });
            action_creation_impls.push(quote! { #action_server_impl });
        }
    }

    let aliases = package.aliases_token_stream();

    let (attributes, ffi_imports, extern_block, rust_imports) = if create_cxx_bridge {
        let reuse_bindings = package.reuse_bindings_token_stream();
        let rust_imports = generate_package_rust_imports_for_cxx();
        (
            quote! { #[cxx::bridge] },
            quote! {},
            quote! {
                extern "Rust" {
                    #(#message_topic_defs)*
                    #(#service_creation_defs)*
                    #(#action_creation_defs)*
                }
                extern "C++" {
                    include!("ros2-bridge/impl.rs.h");
                    include!("dora-node-api.h");
                    type CombinedEvents = crate::ffi::CombinedEvents;
                    type CombinedEvent = crate::ffi::CombinedEvent;

                    type Ros2Context = crate::ros2::default_impl::Ros2Context;
                    type Ros2Node = crate::ros2::default_impl::Ros2Node;
                    type Ros2ParameterClient = crate::ros2::default_impl::Ros2ParameterClient;
                    type Ros2ParameterEvent = crate::ros2::default_impl::ffi::Ros2ParameterEvent;

                    type Ros2Durability = crate::ros2::default_impl::ffi::Ros2Durability;
                    type Ros2Liveliness = crate::ros2::default_impl::ffi::Ros2Liveliness;
                    type Ros2ActionClientQosPolicies = crate::ros2::default_impl::ffi::Ros2ActionClientQosPolicies;
                    type Ros2QosPolicies = crate::ros2::default_impl::ffi::Ros2QosPolicies;

                    type U16String = crate::ros2::default_impl::ffi::U16String;

                    type ActionGoalId = crate::ros2::default_impl::ActionGoalId;
                    type ActionStatusEnum = crate::ros2::default_impl::ffi::ActionStatusEnum;

                    #reuse_bindings
                }
            },
            quote! {
                #rust_imports

                use crate::ros2::default_impl::Ros2Node;
                #[allow(unused_imports)]
                use crate::ros2::default_impl::Ros2ParameterClient;
                #[allow(unused_imports)]
                use crate::ros2::default_impl::ActionGoalId;
            },
        )
    } else {
        let dependencies = package.dependencies_import_token_stream();
        (
            quote! {},
            quote! {
                use serde::{Deserialize, Serialize};

                #[allow(unused_imports)]
                use crate::messages::default_impl::ffi::*;
                #dependencies
            },
            quote! {},
            quote! {},
        )
    };

    quote! {
        #rust_imports

        #attributes
        pub mod ffi {
            #ffi_imports

            #extern_block
            #(#shared_type_defs)*
            #(#service_defs)*
            #(#action_defs)*
        }

        #(#message_struct_impls)*

        #(#message_topic_impls)*
        #(#service_creation_impls)*
        #(#action_creation_impls)*

        #(#service_impls)*
        #(#action_impls)*

        #aliases
    }
}

#[allow(clippy::cognitive_complexity)]
pub fn generate<P>(paths: &[P], out_dir: &Path, create_cxx_bridge: bool) -> proc_macro2::TokenStream
where
    P: AsRef<Path>,
{
    let packages = get_packages(paths).unwrap();
    let mut mod_decl = vec![];
    let msg_dir = out_dir.join("msg");
    if !msg_dir.exists() {
        std::fs::create_dir(&msg_dir).unwrap();
    }
    // generate mod
    for package in packages.iter() {
        let mod_impl = generate_package(package, create_cxx_bridge);
        let generated_string = format_token_stream(mod_impl);
        let package_name = &package.name;
        let file_path = msg_dir.join(format!("{}.rs", package_name));
        std::fs::write(&file_path, generated_string).unwrap();
        let file_path_str = file_path.to_str().unwrap();
        let package_name_ident = format_ident!("{}", package_name);
        mod_decl.push(quote! {
            #[path = #file_path_str]
            pub mod #package_name_ident;
        });
    }

    {
        let generated_default_impls = generate_default_impls(create_cxx_bridge);
        let generated_string = format_token_stream(generated_default_impls);
        let file_path = out_dir.join("impl.rs");
        std::fs::write(&file_path, generated_string).unwrap();
        let file_path_str = file_path.to_str().unwrap();
        mod_decl.push(quote! {
            #[path = #file_path_str]
            pub mod default_impl;
        });
    }

    quote! {
        #(#mod_decl)*

        pub use default_impl::*;
    }
}

fn generate_default_impls(create_cxx_bridge: bool) -> proc_macro2::TokenStream {
    let cxx_ros2_decl = quote! {
        extern "Rust" {
            type Ros2Context;
            type Ros2Node;
            type ActionGoalId;
            type Ros2ParameterClient;
            fn init_ros2_context() -> Result<Box<Ros2Context>>;
            fn init_ros2_context_with_transport(transport: Ros2TransportConfig) -> Result<Box<Ros2Context>>;
            fn new_node(self: &Ros2Context, name_space: &str, base_name: &str) -> Result<Box<Ros2Node>>;
            fn qos_default() -> Ros2QosPolicies;
            fn actionqos_default() -> Ros2ActionClientQosPolicies;

            fn set_parameter(self: &Ros2Node, name: &str, value: &str) -> Result<()>;
            fn set_parameter_string(self: &Ros2Node, name: &str, value: &str) -> Result<()>;
            fn set_parameter_double(self: &Ros2Node, name: &str, value: f64) -> Result<()>;
            fn set_parameter_int(self: &Ros2Node, name: &str, value: i64) -> Result<()>;
            fn set_parameter_bool(self: &Ros2Node, name: &str, value: bool) -> Result<()>;
            fn get_parameter(self: &Ros2Node, name: &str) -> Result<String>;
            fn get_parameter_string(self: &Ros2Node, name: &str) -> Result<String>;
            fn get_parameter_double(self: &Ros2Node, name: &str) -> Result<f64>;
            fn get_parameter_int(self: &Ros2Node, name: &str) -> Result<i64>;
            fn get_parameter_bool(self: &Ros2Node, name: &str) -> Result<bool>;
            fn list_parameters(self: &Ros2Node) -> Result<Vec<String>>;
            fn has_parameter(self: &Ros2Node, name: &str) -> bool;

            fn create_parameter_client(self: &mut Ros2Node, target: &str, timeout_s: f64) -> Result<Box<Ros2ParameterClient>>;

            fn target(self: &Ros2ParameterClient) -> String;
            fn get(self: &Ros2ParameterClient, name: &str, timeout_s: f64) -> Result<String>;
            fn get_string(self: &Ros2ParameterClient, name: &str, timeout_s: f64) -> Result<String>;
            fn get_double(self: &Ros2ParameterClient, name: &str, timeout_s: f64) -> Result<f64>;
            fn get_int(self: &Ros2ParameterClient, name: &str, timeout_s: f64) -> Result<i64>;
            fn get_bool(self: &Ros2ParameterClient, name: &str, timeout_s: f64) -> Result<bool>;
            fn set(self: &Ros2ParameterClient, name: &str, value: &str, timeout_s: f64) -> Result<bool>;
            fn set_string(self: &Ros2ParameterClient, name: &str, value: &str, timeout_s: f64) -> Result<bool>;
            fn set_double(self: &Ros2ParameterClient, name: &str, value: f64, timeout_s: f64) -> Result<bool>;
            fn set_int(self: &Ros2ParameterClient, name: &str, value: i64, timeout_s: f64) -> Result<bool>;
            fn set_bool(self: &Ros2ParameterClient, name: &str, value: bool, timeout_s: f64) -> Result<bool>;
            fn list(self: &Ros2ParameterClient, prefix: &str, depth: u64, timeout_s: f64) -> Result<Vec<String>>;
            fn get_type(self: &Ros2ParameterClient, name: &str, timeout_s: f64) -> Result<String>;
            fn get_types(self: &Ros2ParameterClient, names: &Vec<String>, timeout_s: f64) -> Result<Vec<String>>;
            fn describe(self: &Ros2ParameterClient, name: &str, timeout_s: f64) -> Result<String>;
            fn take_event(self: &Ros2ParameterClient) -> Result<Ros2ParameterEvent>;
        }

        #[derive(Debug, Clone)]
        pub struct Ros2ParameterEvent {
            pub has_event: bool,
            pub new_names: Vec<String>,
            pub new_values: Vec<String>,
            pub changed_names: Vec<String>,
            pub changed_values: Vec<String>,
            pub deleted_names: Vec<String>,
        }

        #[derive(Debug, Clone, Copy)]
        pub enum Ros2TransportKind {
            Dds,
            ZenohHumble,
            ZenohRep2016,
        }

        #[derive(Debug, Clone)]
        pub struct Ros2TransportConfig {
            pub kind: Ros2TransportKind,
            /// Empty means use `ZENOH_SESSION_CONFIG_URI`, then the embedded default.
            pub config_uri: String,
        }

        #[derive(Debug, Clone)]
        pub struct Ros2QosPolicies {
            pub durability: Ros2Durability,
            pub liveliness: Ros2Liveliness,
            pub lease_duration: f64,
            pub reliable: bool,
            pub max_blocking_time: f64,
            pub keep_all: bool,
            pub keep_last: i32,
        }

        #[derive(Debug, Clone)]
        pub struct Ros2ActionClientQosPolicies {
            pub goal_service: Ros2QosPolicies,
            pub result_service: Ros2QosPolicies,
            pub cancel_service: Ros2QosPolicies,
            pub feedback_subscription: Ros2QosPolicies,
            pub status_subscription: Ros2QosPolicies,
        }

        /// DDS 2.2.3.4 DURABILITY
        #[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub enum Ros2Durability {
            Volatile,
            TransientLocal,
            Transient,
            Persistent,
        }

        /// DDS 2.2.3.11 LIVELINESS
        #[derive(Copy, Clone, Debug, PartialEq)]
        pub enum Ros2Liveliness {
            Automatic,
            ManualByParticipant,
            ManualByTopic,
        }

        /// Named goal statuses from [GoalStatus](https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html)
        #[derive(Clone, Copy, PartialEq, Debug)]
        #[repr(i8)]
        pub enum ActionStatusEnum {
            /// Let's use "Unknown" also for "New"
            Unknown = 0,
            Accepted = 1,
            Executing = 2,
            Canceling = 3,
            Succeeded = 4,
            Canceled = 5,
            Aborted = 6,
        }

        // It's used to auto-generate code about Box<ActionGoalId> with CXX crate
        pub struct ActionGoalIdBoxed {
            inner: Box<ActionGoalId>,
        }
    };
    let cxx_ros2_impl = quote! {
        pub struct Ros2Context{
            context: dora_ros2_bridge::transport::Context,
            compatibility: Option<dora_ros2_bridge::dora_message::descriptor::RmwZenohCompatibility>,
            executor: std::sync::Arc<futures::executor::ThreadPool>,
        }

        fn init_ros2_context() -> eyre::Result<Box<Ros2Context>> {
            init_ros2_context_with_transport(ffi::Ros2TransportConfig {
                kind: ffi::Ros2TransportKind::Dds,
                config_uri: String::new(),
            })
        }

        fn init_ros2_context_with_transport(transport: ffi::Ros2TransportConfig) -> eyre::Result<Box<Ros2Context>> {
            let config_uri = (!transport.config_uri.is_empty()).then(|| transport.config_uri.into());
            let (config, compatibility) = match transport.kind {
                ffi::Ros2TransportKind::Dds => (dora_ros2_bridge::dora_message::descriptor::Ros2TransportConfig::Dds, None),
                ffi::Ros2TransportKind::ZenohHumble => (dora_ros2_bridge::dora_message::descriptor::Ros2TransportConfig::Zenoh {
                    compatibility: dora_ros2_bridge::dora_message::descriptor::RmwZenohCompatibility::Humble,
                    config_uri: config_uri.clone(),
                }, Some(dora_ros2_bridge::dora_message::descriptor::RmwZenohCompatibility::Humble)),
                ffi::Ros2TransportKind::ZenohRep2016 => (dora_ros2_bridge::dora_message::descriptor::Ros2TransportConfig::Zenoh {
                    compatibility: dora_ros2_bridge::dora_message::descriptor::RmwZenohCompatibility::Rep2016,
                    config_uri,
                }, Some(dora_ros2_bridge::dora_message::descriptor::RmwZenohCompatibility::Rep2016)),
                _ => eyre::bail!("unknown ROS2 transport kind"),
            };
            Ok(Box::new(Ros2Context{
                context: futures::executor::block_on(dora_ros2_bridge::transport::Context::open(
                    &config,
                    std::env::var("ROS_DOMAIN_ID")
                        .ok()
                        .and_then(|value| value.parse().ok())
                        .unwrap_or(0),
                ))?,
                compatibility,
                executor: std::sync::Arc::new(futures::executor::ThreadPool::new()?),
            }))
        }

        impl Ros2Context {
            fn new_node(&self, name_space: &str, base_name: &str) -> eyre::Result<Box<Ros2Node>> {
                use futures::task::SpawnExt as _;
                use eyre::{ContextCompat as _, WrapErr as _};

                let options = crate::ros2_client::NodeOptions::new().enable_rosout(true);
                let node = futures::executor::block_on(self.context.new_named_node(name_space, base_name, options))
                    .map_err(|e| eyre::eyre!("failed to create ROS2 node: {e:?}"))?;

                let node = match node {
                    dora_ros2_bridge::transport::Node::Dds(node) => {
                        let mut node = (*node).into_inner();
                        let spinner = node.spinner().context("failed to create spinner")?;
                        self.executor.spawn(async {
                            if let Err(err) = spinner.spin().await {
                                eprintln!("ros2 spinner failed: {err:?}");
                            }
                        })
                        .context("failed to spawn ros2 spinner")?;
                        GeneratedNode::Dds(node)
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    dora_ros2_bridge::transport::Node::Zenoh(node) => GeneratedNode::Zenoh {
                        node,
                        compatibility: self.compatibility.context("Zenoh compatibility profile missing")?,
                        parameters: std::sync::Arc::new(std::sync::Mutex::new(None)),
                    }
                };

                Ok(Box::new(Ros2Node{ node, executor: self.executor.clone(), }))
            }
        }

        pub enum GeneratedNode {
            Dds(ros2_client::Node),
            #[cfg(feature = "rmw-zenoh")]
            Zenoh {
                node: dora_ros2_bridge::transport::zenoh::Node,
                compatibility: dora_ros2_bridge::dora_message::descriptor::RmwZenohCompatibility,
                parameters: std::sync::Arc<std::sync::Mutex<Option<dora_ros2_bridge::transport::zenoh::parameters::ParameterServer>>>,
            },
        }

        pub struct Ros2Node {
            pub node : GeneratedNode,
            pub executor: std::sync::Arc<futures::executor::ThreadPool>,
        }

        impl Ros2Node {
            pub fn dds_node(&self) -> eyre::Result<&ros2_client::Node> {
                match &self.node {
                    GeneratedNode::Dds(node) => Ok(node),
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { .. } => eyre::bail!("entity requires the DDS transport"),
                }
            }

            pub fn dds_node_mut(&mut self) -> eyre::Result<&mut ros2_client::Node> {
                match &mut self.node {
                    GeneratedNode::Dds(node) => Ok(node),
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { .. } => eyre::bail!("entity requires the DDS transport"),
                }
            }

            pub fn create_parameter_client(
                &mut self,
                target: &str,
                timeout_s: f64,
            ) -> eyre::Result<Box<Ros2ParameterClient>> {
                use dora_ros2_bridge::transport::parameters::{DdsParameterClient, ParameterClient};
                #[cfg(feature = "rmw-zenoh")]
                use dora_ros2_bridge::transport::zenoh::parameters::ParameterClient as ZenohParameterClient;

                let timeout = timeout_or(timeout_s, 20.0);
                let client = match &mut self.node {
                    GeneratedNode::Dds(node) => {
                        let client = DdsParameterClient::new(node, target)
                            .map_err(|error| eyre::eyre!(error))?;
                        futures::executor::block_on(client.wait_for_services(node, timeout))
                            .map_err(|error| eyre::eyre!(error))?;
                        ParameterClient::Dds(Box::new(client))
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { node, compatibility, .. } => {
                        let types = dora_ros2_bridge::transport::zenoh::parameters::ParameterTypes::resolve(
                            *compatibility,
                            &dora_ros2_bridge::dora_message::descriptor::TypeDescriptionResolver::from_ament_prefix_path(),
                        )
                        .map_err(|e| eyre::eyre!("a ROS2 parameter client on Zenoh needs the `rcl_interfaces` types: {e}"))?;
                        let client = futures::executor::block_on(ZenohParameterClient::declare(
                            node, target, &types,
                        ))
                        .map_err(|e| eyre::eyre!("failed to create a parameter client for `{target}`: {e}"))?;
                        futures::executor::block_on(client.wait_for_services(node, timeout))
                            .map_err(|e| eyre::eyre!("the parameter services of `{target}` are not available: {e}"))?;
                        ParameterClient::Zenoh(Box::new(client))
                    }
                };
                Ok(Box::new(Ros2ParameterClient { client }))
            }

            pub fn set_parameter(&self, name: &str, value: &str) -> eyre::Result<()> {
                if let Ok(b) = value.parse::<bool>() {
                    return self.set_parameter_bool(name, b);
                }
                if let Ok(i) = value.parse::<i64>() {
                    return self.set_parameter_int(name, i);
                }
                if let Ok(d) = value.parse::<f64>() {
                    return self.set_parameter_double(name, d);
                }
                self.set_parameter_string(name, value)
            }

            pub fn set_parameter_string(&self, name: &str, value: &str) -> eyre::Result<()> {
                let val = ros2_client::ParameterValue::String(value.to_owned());
                self.set_param_value(name, val)
            }

            pub fn set_parameter_double(&self, name: &str, value: f64) -> eyre::Result<()> {
                let val = ros2_client::ParameterValue::Double(value);
                self.set_param_value(name, val)
            }

            pub fn set_parameter_int(&self, name: &str, value: i64) -> eyre::Result<()> {
                let val = ros2_client::ParameterValue::Integer(value);
                self.set_param_value(name, val)
            }

            pub fn set_parameter_bool(&self, name: &str, value: bool) -> eyre::Result<()> {
                let val = ros2_client::ParameterValue::Boolean(value);
                self.set_param_value(name, val)
            }

            fn set_param_value(&self, name: &str, val: ros2_client::ParameterValue) -> eyre::Result<()> {
                match &self.node {
                    GeneratedNode::Dds(node) => {
                        node.set_parameter(name, val)
                            .map_err(|e| eyre::eyre!("failed to set parameter `{name}`: {e:?}"))
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { node, compatibility, parameters } => {
                        let mut guard = parameters.lock().unwrap();
                        if guard.is_none() {
                            let types = dora_ros2_bridge::transport::zenoh::parameters::ParameterTypes::resolve(
                                *compatibility,
                                &dora_ros2_bridge::dora_message::descriptor::TypeDescriptionResolver::from_ament_prefix_path(),
                            )
                            .map_err(|e| eyre::eyre!("failed to resolve parameter types: {e:?}"))?;
                            let node_name = ros2_client::NodeName::new(node.namespace(), node.name())
                                .map_err(|e| eyre::eyre!("invalid node name: {e:?}"))?;
                            let mut initial = std::collections::BTreeMap::new();
                            initial.insert(name.to_owned(), val.clone());
                            let server = futures::executor::block_on(
                                dora_ros2_bridge::transport::zenoh::parameters::ParameterServer::declare(
                                    node,
                                    &node_name,
                                    types,
                                    initial,
                                    None,
                                )
                            )
                            .map_err(|e| eyre::eyre!("failed to declare parameter server: {e:?}"))?;
                            *guard = Some(server);
                            return Ok(());
                        }
                        if let Some(ref server) = *guard {
                            return futures::executor::block_on(server.set(name, val))
                                .map_err(|e| eyre::eyre!("failed to set parameter `{name}`: {e:?}"));
                        }
                        eyre::bail!("failed to set parameter `{name}` on Zenoh node")
                    }
                }
            }

            pub fn get_parameter(&self, name: &str) -> eyre::Result<String> {
                let val = self.get_param_value(name)?;
                Ok(parameter_value_to_string(&val))
            }

            pub fn get_parameter_string(&self, name: &str) -> eyre::Result<String> {
                match self.get_param_value(name)? {
                    ros2_client::ParameterValue::String(s) => Ok(s),
                    other => Ok(parameter_value_to_string(&other)),
                }
            }

            pub fn get_parameter_double(&self, name: &str) -> eyre::Result<f64> {
                match self.get_param_value(name)? {
                    ros2_client::ParameterValue::Double(d) => Ok(d),
                    ros2_client::ParameterValue::Integer(i) => Ok(i as f64),
                    other => eyre::bail!("parameter `{name}` is not a double, got {other:?}"),
                }
            }

            pub fn get_parameter_int(&self, name: &str) -> eyre::Result<i64> {
                match self.get_param_value(name)? {
                    ros2_client::ParameterValue::Integer(i) => Ok(i),
                    other => eyre::bail!("parameter `{name}` is not an integer, got {other:?}"),
                }
            }

            pub fn get_parameter_bool(&self, name: &str) -> eyre::Result<bool> {
                match self.get_param_value(name)? {
                    ros2_client::ParameterValue::Boolean(b) => Ok(b),
                    other => eyre::bail!("parameter `{name}` is not a boolean, got {other:?}"),
                }
            }

            fn get_param_value(&self, name: &str) -> eyre::Result<ros2_client::ParameterValue> {
                match &self.node {
                    GeneratedNode::Dds(node) => {
                        node.get_parameter(name)
                            .ok_or_else(|| eyre::eyre!("parameter `{name}` not found"))
                    }
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { parameters, .. } => {
                        if let Ok(guard) = parameters.lock() {
                            if let Some(ref server) = *guard {
                                return server.get(name)
                                    .ok_or_else(|| eyre::eyre!("parameter `{name}` not found"));
                            }
                        }
                        eyre::bail!("parameter `{name}` not found (no parameter server active on Zenoh node)")
                    }
                }
            }

            pub fn list_parameters(&self) -> eyre::Result<Vec<String>> {
                match &self.node {
                    GeneratedNode::Dds(node) => Ok(node.list_parameters()),
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { parameters, .. } => {
                        if let Ok(guard) = parameters.lock() {
                            if let Some(ref server) = *guard {
                                return Ok(server.list());
                            }
                        }
                        Ok(Vec::new())
                    }
                }
            }

            pub fn has_parameter(&self, name: &str) -> bool {
                match &self.node {
                    GeneratedNode::Dds(node) => node.has_parameter(name),
                    #[cfg(feature = "rmw-zenoh")]
                    GeneratedNode::Zenoh { parameters, .. } => {
                        if let Ok(guard) = parameters.lock() {
                            if let Some(ref server) = *guard {
                                return server.has(name);
                            }
                        }
                        false
                    }
                }
            }
        }

        unsafe impl cxx::ExternType for Ros2Node {
            type Id = cxx::type_id!("Ros2Node");
            type Kind = cxx::kind::Opaque;
        }

        pub struct Ros2ParameterClient {
            pub client: dora_ros2_bridge::transport::parameters::ParameterClient,
        }

        unsafe impl cxx::ExternType for Ros2ParameterClient {
            type Id = cxx::type_id!("Ros2ParameterClient");
            type Kind = cxx::kind::Opaque;
        }

        impl Ros2ParameterClient {
            pub fn target(&self) -> String {
                self.client.target().to_owned()
            }

            pub fn get(&self, name: &str, timeout_s: f64) -> eyre::Result<String> {
                let timeout = timeout_or(timeout_s, 30.0);
                let names = vec![name.to_owned()];
                let values = futures::executor::block_on(self.client.get(&names, timeout))?;
                match values.into_iter().next().flatten() {
                    Some(v) => Ok(parameter_value_to_string(&v)),
                    None => eyre::bail!("parameter `{name}` not found on target `{}`", self.client.target()),
                }
            }

            pub fn get_double(&self, name: &str, timeout_s: f64) -> eyre::Result<f64> {
                let timeout = timeout_or(timeout_s, 30.0);
                let names = vec![name.to_owned()];
                let values = futures::executor::block_on(self.client.get(&names, timeout))?;
                match values.into_iter().next().flatten() {
                    Some(ros2_client::ParameterValue::Double(d)) => Ok(d),
                    Some(ros2_client::ParameterValue::Integer(i)) => Ok(i as f64),
                    Some(other) => eyre::bail!("parameter `{name}` is not a double, got {other:?}"),
                    None => eyre::bail!("parameter `{name}` not found on target `{}`", self.client.target()),
                }
            }

            pub fn get_string(&self, name: &str, timeout_s: f64) -> eyre::Result<String> {
                let timeout = timeout_or(timeout_s, 30.0);
                let names = vec![name.to_owned()];
                let values = futures::executor::block_on(self.client.get(&names, timeout))?;
                match values.into_iter().next().flatten() {
                    Some(ros2_client::ParameterValue::String(s)) => Ok(s),
                    Some(other) => Ok(parameter_value_to_string(&other)),
                    None => eyre::bail!("parameter `{name}` not found on target `{}`", self.client.target()),
                }
            }

            pub fn get_int(&self, name: &str, timeout_s: f64) -> eyre::Result<i64> {
                let timeout = timeout_or(timeout_s, 30.0);
                let names = vec![name.to_owned()];
                let values = futures::executor::block_on(self.client.get(&names, timeout))?;
                match values.into_iter().next().flatten() {
                    Some(ros2_client::ParameterValue::Integer(i)) => Ok(i),
                    Some(other) => eyre::bail!("parameter `{name}` is not an integer, got {other:?}"),
                    None => eyre::bail!("parameter `{name}` not found on target `{}`", self.client.target()),
                }
            }

            pub fn get_bool(&self, name: &str, timeout_s: f64) -> eyre::Result<bool> {
                let timeout = timeout_or(timeout_s, 30.0);
                let names = vec![name.to_owned()];
                let values = futures::executor::block_on(self.client.get(&names, timeout))?;
                match values.into_iter().next().flatten() {
                    Some(ros2_client::ParameterValue::Boolean(b)) => Ok(b),
                    Some(other) => eyre::bail!("parameter `{name}` is not a boolean, got {other:?}"),
                    None => eyre::bail!("parameter `{name}` not found on target `{}`", self.client.target()),
                }
            }

            pub fn set(&self, name: &str, value: &str, timeout_s: f64) -> eyre::Result<bool> {
                if let Ok(b) = value.parse::<bool>() {
                    return self.set_bool(name, b, timeout_s);
                }
                if let Ok(i) = value.parse::<i64>() {
                    return self.set_int(name, i, timeout_s);
                }
                if let Ok(d) = value.parse::<f64>() {
                    return self.set_double(name, d, timeout_s);
                }
                self.set_string(name, value, timeout_s)
            }

            pub fn set_double(&self, name: &str, value: f64, timeout_s: f64) -> eyre::Result<bool> {
                let timeout = timeout_or(timeout_s, 30.0);
                let values = vec![(name.to_owned(), ros2_client::ParameterValue::Double(value))];
                let results = futures::executor::block_on(self.client.set(values, timeout))?;
                match results.into_iter().next() {
                    Some(res) => {
                        if !res.successful {
                            eyre::bail!("failed to set parameter `{name}`: {}", res.reason);
                        }
                        Ok(true)
                    }
                    None => eyre::bail!("no response for setting parameter `{name}`"),
                }
            }

            pub fn set_string(&self, name: &str, value: &str, timeout_s: f64) -> eyre::Result<bool> {
                let timeout = timeout_or(timeout_s, 30.0);
                let values = vec![(name.to_owned(), ros2_client::ParameterValue::String(value.to_owned()))];
                let results = futures::executor::block_on(self.client.set(values, timeout))?;
                match results.into_iter().next() {
                    Some(res) => {
                        if !res.successful {
                            eyre::bail!("failed to set parameter `{name}`: {}", res.reason);
                        }
                        Ok(true)
                    }
                    None => eyre::bail!("no response for setting parameter `{name}`"),
                }
            }

            pub fn set_int(&self, name: &str, value: i64, timeout_s: f64) -> eyre::Result<bool> {
                let timeout = timeout_or(timeout_s, 30.0);
                let values = vec![(name.to_owned(), ros2_client::ParameterValue::Integer(value))];
                let results = futures::executor::block_on(self.client.set(values, timeout))?;
                match results.into_iter().next() {
                    Some(res) => {
                        if !res.successful {
                            eyre::bail!("failed to set parameter `{name}`: {}", res.reason);
                        }
                        Ok(true)
                    }
                    None => eyre::bail!("no response for setting parameter `{name}`"),
                }
            }

            pub fn set_bool(&self, name: &str, value: bool, timeout_s: f64) -> eyre::Result<bool> {
                let timeout = timeout_or(timeout_s, 30.0);
                let values = vec![(name.to_owned(), ros2_client::ParameterValue::Boolean(value))];
                let results = futures::executor::block_on(self.client.set(values, timeout))?;
                match results.into_iter().next() {
                    Some(res) => {
                        if !res.successful {
                            eyre::bail!("failed to set parameter `{name}`: {}", res.reason);
                        }
                        Ok(true)
                    }
                    None => eyre::bail!("no response for setting parameter `{name}`"),
                }
            }

            pub fn list(&self, prefix: &str, depth: u64, timeout_s: f64) -> eyre::Result<Vec<String>> {
                let timeout = timeout_or(timeout_s, 30.0);
                let prefixes = if prefix.is_empty() { vec![] } else { vec![prefix.to_owned()] };
                let listed = futures::executor::block_on(self.client.list(prefixes, depth, timeout))?;
                Ok(listed.names)
            }

            pub fn get_type(&self, name: &str, timeout_s: f64) -> eyre::Result<String> {
                let timeout = timeout_or(timeout_s, 30.0);
                let names = vec![name.to_owned()];
                let types = futures::executor::block_on(self.client.get_types(&names, timeout))?;
                match types.into_iter().next() {
                    Some(ty) => Ok(parameter_type_name(ty).to_owned()),
                    None => eyre::bail!("no type returned for parameter `{name}`"),
                }
            }

            pub fn get_types(&self, names: &Vec<String>, timeout_s: f64) -> eyre::Result<Vec<String>> {
                let timeout = timeout_or(timeout_s, 30.0);
                let types = futures::executor::block_on(self.client.get_types(names, timeout))?;
                Ok(types.into_iter().map(|t| parameter_type_name(t).to_owned()).collect())
            }

            pub fn describe(&self, name: &str, timeout_s: f64) -> eyre::Result<String> {
                let timeout = timeout_or(timeout_s, 30.0);
                let names = vec![name.to_owned()];
                let descriptors = futures::executor::block_on(self.client.describe(&names, timeout))?;
                match descriptors.into_iter().next().flatten() {
                    Some(d) => Ok(format!("{}: {}", d.name, d.description)),
                    None => eyre::bail!("no descriptor returned for parameter `{name}`"),
                }
            }

            pub fn take_event(&self) -> eyre::Result<ffi::Ros2ParameterEvent> {
                match self.client.try_next_change()? {
                    Some(change) => {
                        let mut new_names = Vec::new();
                        let mut new_values = Vec::new();
                        for (k, v) in change.new {
                            new_names.push(k);
                            new_values.push(parameter_value_to_string(&v));
                        }
                        let mut changed_names = Vec::new();
                        let mut changed_values = Vec::new();
                        for (k, v) in change.changed {
                            changed_names.push(k);
                            changed_values.push(parameter_value_to_string(&v));
                        }
                        Ok(ffi::Ros2ParameterEvent {
                            has_event: true,
                            new_names,
                            new_values,
                            changed_names,
                            changed_values,
                            deleted_names: change.deleted,
                        })
                    }
                    None => Ok(ffi::Ros2ParameterEvent {
                        has_event: false,
                        new_names: Vec::new(),
                        new_values: Vec::new(),
                        changed_names: Vec::new(),
                        changed_values: Vec::new(),
                        deleted_names: Vec::new(),
                    }),
                }
            }
        }

        fn timeout_or(timeout_s: f64, default_secs: f64) -> std::time::Duration {
            if timeout_s > 0.0 {
                std::time::Duration::from_secs_f64(timeout_s)
            } else {
                std::time::Duration::from_secs_f64(default_secs)
            }
        }

        fn parameter_value_to_string(value: &ros2_client::ParameterValue) -> String {
            match value {
                ros2_client::ParameterValue::NotSet => String::new(),
                ros2_client::ParameterValue::Boolean(b) => b.to_string(),
                ros2_client::ParameterValue::Integer(i) => i.to_string(),
                ros2_client::ParameterValue::Double(d) => d.to_string(),
                ros2_client::ParameterValue::String(s) => s.clone(),
                ros2_client::ParameterValue::ByteArray(b) => format!("{b:?}"),
                ros2_client::ParameterValue::BooleanArray(b) => format!("{b:?}"),
                ros2_client::ParameterValue::IntegerArray(i) => format!("{i:?}"),
                ros2_client::ParameterValue::DoubleArray(d) => format!("{d:?}"),
                ros2_client::ParameterValue::StringArray(s) => format!("{s:?}"),
            }
        }

        fn parameter_type_name(ty: u8) -> &'static str {
            match ty {
                0 => "not_set",
                1 => "bool",
                2 => "integer",
                3 => "double",
                4 => "string",
                5 => "byte_array",
                6 => "bool_array",
                7 => "integer_array",
                8 => "double_array",
                9 => "string_array",
                _ => "unknown",
            }
        }

        unsafe impl cxx::ExternType for Ros2Context {
            type Id = cxx::type_id!("Ros2Context");
            type Kind = cxx::kind::Opaque;
        }

        #[derive(Clone)]
        pub struct ActionGoalId {
            pub id: ros2_client::action::GoalId,
        }

        unsafe impl cxx::ExternType for ActionGoalId {
            type Id = cxx::type_id!("ActionGoalId");
            type Kind = cxx::kind::Opaque;
        }

        fn qos_default() -> ffi::Ros2QosPolicies {
            ffi::Ros2QosPolicies::new(None, None, None, None, None, None, None)
        }

        fn actionqos_default() -> ffi::Ros2ActionClientQosPolicies {
            ffi::Ros2ActionClientQosPolicies::new(
                Some(qos_default()),
                Some(qos_default()),
                Some(qos_default()),
                Some(qos_default()),
                Some(qos_default())
            )
        }

        impl ffi::Ros2QosPolicies {
            pub fn new(
                durability: Option<ffi::Ros2Durability>,
                liveliness: Option<ffi::Ros2Liveliness>,
                reliable: Option<bool>,
                keep_all: Option<bool>,
                lease_duration: Option<f64>,
                max_blocking_time: Option<f64>,
                keep_last: Option<i32>,
            ) -> Self {
                Self {
                    durability: durability.unwrap_or(ffi::Ros2Durability::Volatile),
                    liveliness: liveliness.unwrap_or(ffi::Ros2Liveliness::Automatic),
                    lease_duration: lease_duration.unwrap_or(f64::INFINITY),
                    reliable: reliable.unwrap_or(false),
                    max_blocking_time: max_blocking_time.unwrap_or(0.0),
                    keep_all: keep_all.unwrap_or(false),
                    keep_last: keep_last.unwrap_or(1),
                }
            }
        }

        /// Bounds on unanswered service requests, shared by the generated service
        /// clients and servers. The same values as the Python bridge and the
        /// YAML bridge (`binaries/ros2-bridge-node`) use.
        pub const SERVICE_RESPONSE_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(30);
        pub const MAX_PENDING_REQUESTS: usize = 64;

        /// Bounds of a generated action server: goals past
        /// `ACTION_MAX_CONCURRENT_GOALS` in flight are rejected, and a goal
        /// still running `ACTION_GOAL_TIMEOUT` (the daemon's
        /// ACTION_RESULT_TIMEOUT) after it was accepted is aborted when the
        /// next goal comes in, so abandoned goals can't wedge the server at
        /// the cap.
        pub const ACTION_MAX_CONCURRENT_GOALS: usize = 16;
        pub const ACTION_GOAL_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(300);

        /// Largest feedback or status message a generated action client
        /// accepts on native Zenoh.
        #[cfg(feature = "rmw-zenoh")]
        pub const ZENOH_ACTION_MAX_PAYLOAD: usize = 64 * 1024 * 1024;

        /// The transport's id for a request a generated service server handed to
        /// C++ under an opaque `u64` token.
        pub enum PendingRequestId {
            Dds(ros2_client::service::RmwRequestId),
            #[cfg(feature = "rmw-zenoh")]
            Zenoh(dora_ros2_bridge::transport::RequestId),
        }

        /// Records `id` under `token`, bounding the map: an entry whose C++
        /// handler never calls `send_response` (crash, early return, dropped
        /// request) is evicted, so a misbehaving server cannot leak unboundedly.
        pub fn record_request(
            requests: &std::sync::Mutex<std::collections::HashMap<u64, (PendingRequestId, std::time::Instant)>>,
            token: u64,
            id: PendingRequestId,
        ) {
            if let Ok(mut map) = requests.lock() {
                let now = std::time::Instant::now();
                // Evict entries whose response never came.
                map.retain(|_, (_, t)| now.duration_since(*t) < SERVICE_RESPONSE_TIMEOUT);
                // If still at capacity, drop the oldest (smallest token) to make room.
                if map.len() >= MAX_PENDING_REQUESTS
                    && let Some(&oldest) = map.keys().min()
                {
                    map.remove(&oldest);
                }
                map.insert(token, (id, now));
            }
        }

        /// The C++ QoS as backend-neutral policies, shared by the DDS and Zenoh
        /// paths. Durations are f64 seconds (a non-finite lease is infinite); an
        /// unknown liveliness value falls back to automatic.
        pub(crate) fn ros2_qos(value: &ffi::Ros2QosPolicies, durability: dora_ros2_bridge::transport::Durability) -> dora_ros2_bridge::transport::Ros2Qos {
            use dora_ros2_bridge::transport::{dds::duration_from_secs_f64, History, Liveliness, Reliability, Ros2Qos};
            let lease_duration = value.lease_duration.is_finite().then(|| duration_from_secs_f64(value.lease_duration));
            Ros2Qos {
                reliability: if value.reliable {
                    Reliability::Reliable { max_blocking_time: duration_from_secs_f64(value.max_blocking_time) }
                } else { Reliability::BestEffort },
                durability,
                history: if value.keep_all { History::KeepAll } else { History::KeepLast { depth: value.keep_last } },
                liveliness: match value.liveliness {
                    ffi::Ros2Liveliness::ManualByParticipant => Liveliness::ManualByParticipant { lease_duration },
                    ffi::Ros2Liveliness::ManualByTopic => Liveliness::ManualByTopic { lease_duration },
                    _ => Liveliness::Automatic { lease_duration },
                },
            }
        }

        #[cfg(feature = "rmw-zenoh")]
        pub fn neutral_qos(value: &ffi::Ros2QosPolicies) -> eyre::Result<dora_ros2_bridge::transport::Ros2Qos> {
            use dora_ros2_bridge::transport::Durability;
            let durability = match value.durability {
                ffi::Ros2Durability::Volatile => Durability::Volatile,
                ffi::Ros2Durability::TransientLocal => Durability::TransientLocal,
                _ => eyre::bail!("native Zenoh supports volatile and transient-local durability"),
            };
            if !matches!(
                value.liveliness,
                ffi::Ros2Liveliness::Automatic | ffi::Ros2Liveliness::ManualByParticipant | ffi::Ros2Liveliness::ManualByTopic
            ) {
                eyre::bail!("unknown ROS2 liveliness policy");
            }
            Ok(ros2_qos(value, durability))
        }

        /// `name` as the generated topic, service and action endpoints use it on
        /// native Zenoh: a relative name is under the node's namespace, as rcl
        /// resolves it (and DDS, via ros2-client `Name::to_dds_name`, except
        /// that it writes `//` under the root namespace).
        #[cfg(feature = "rmw-zenoh")]
        pub fn zenoh_full_name(node: &dora_ros2_bridge::transport::zenoh::Node, name: String) -> String {
            if name.starts_with('/') {
                name
            } else {
                format!("{}/{}", node.namespace().trim_end_matches('/'), name)
            }
        }

        /// Data key and graph token for a service endpoint on native Zenoh,
        /// shared by the generated service clients and servers.
        #[cfg(feature = "rmw-zenoh")]
        pub fn zenoh_service_endpoint(
            node: &dora_ros2_bridge::transport::zenoh::Node,
            compatibility: dora_ros2_bridge::dora_message::descriptor::RmwZenohCompatibility,
            package: &str,
            service: &str,
            name: String,
            qos: &ffi::Ros2QosPolicies,
        ) -> eyre::Result<(dora_ros2_bridge::transport::zenoh::keyexpr::DataKey, dora_ros2_bridge::transport::zenoh::keyexpr::TopicToken)> {
            let name = zenoh_full_name(node, name);
            let identity = dora_ros2_bridge::transport::zenoh::compatibility::resolve_service(
                compatibility,
                package,
                service,
                &dora_ros2_bridge::transport::zenoh::compatibility::TypeDescriptionResolver::from_ament_prefix_path(),
            )?;
            let neutral = neutral_qos(qos)?;
            let key = dora_ros2_bridge::transport::zenoh::keyexpr::DataKey::new(node.domain(), &name, &identity)?;
            let token = dora_ros2_bridge::transport::zenoh::keyexpr::TopicToken {
                name,
                type_name: identity.dds_name.clone(),
                type_hash: identity.key_hash_component(),
                qos: dora_ros2_bridge::transport::zenoh::qos::ZenohQosMapping::from_ros_qos(&neutral).to_string(),
            };
            Ok((key, token))
        }

        /// The C++ QoS as the `ros2-client` DDS profile, through the same neutral
        /// conversion as the Zenoh path and the YAML and Python bridges.
        impl From<ffi::Ros2QosPolicies> for crate::ros2_client::QosProfile {
            fn from(value: ffi::Ros2QosPolicies) -> Self {
                use dora_ros2_bridge::transport::{dds, Durability};
                // The neutral durability has no TRANSIENT/PERSISTENT, so it is set here.
                let durability = match value.durability {
                    ffi::Ros2Durability::Volatile => crate::ros2_client::qos::Durability::Volatile,
                    ffi::Ros2Durability::TransientLocal => crate::ros2_client::qos::Durability::TransientLocal,
                    _ => dds::transient_local_for_unsupported_durability(),
                };
                crate::ros2_client::QosProfile {
                    durability,
                    ..dds::to_qos_profile(&ros2_qos(&value, Durability::Volatile))
                }
            }
        }

        impl ffi::Ros2ActionClientQosPolicies {
            pub fn new(
                goal_service: Option<ffi::Ros2QosPolicies>,
                result_service: Option<ffi::Ros2QosPolicies>,
                cancel_service: Option<ffi::Ros2QosPolicies>,
                feedback_subscription: Option<ffi::Ros2QosPolicies>,
                status_subscription: Option<ffi::Ros2QosPolicies>,
            ) -> Self {
                Self {
                    goal_service: goal_service.unwrap_or_else(|| ffi::Ros2QosPolicies::new(None, None, None, None, None, None, None)),
                    result_service: result_service.unwrap_or_else(|| ffi::Ros2QosPolicies::new(None, None, None, None, None, None, None)),
                    cancel_service: cancel_service.unwrap_or_else(|| ffi::Ros2QosPolicies::new(None, None, None, None, None, None, None)),
                    feedback_subscription: feedback_subscription.unwrap_or_else(|| ffi::Ros2QosPolicies::new(None, None, None, None, None, None, None)),
                    status_subscription: status_subscription.unwrap_or_else(|| ffi::Ros2QosPolicies::new(None, None, None, None, None, None, None)),
                }
            }
        }

        impl From<ffi::Ros2ActionClientQosPolicies> for crate::ros2_client::action::ActionClientQosPolicies {
            fn from(value: ffi::Ros2ActionClientQosPolicies) -> Self {
                crate::ros2_client::action::ActionClientQosPolicies {
                    goal_service: value.goal_service.into(),
                    result_service: value.result_service.into(),
                    cancel_service: value.cancel_service.into(),
                    feedback_subscription: value.feedback_subscription.into(),
                    status_subscription: value.status_subscription.into(),
                }
            }
        }

        impl From<crate::ros2_client::action::GoalStatusEnum> for ffi::ActionStatusEnum {
            fn from(value: crate::ros2_client::action::GoalStatusEnum) -> Self {
                match value {
                    crate::ros2_client::action::GoalStatusEnum::Unknown => ffi::ActionStatusEnum::Unknown,
                    crate::ros2_client::action::GoalStatusEnum::Accepted => ffi::ActionStatusEnum::Accepted,
                    crate::ros2_client::action::GoalStatusEnum::Executing => ffi::ActionStatusEnum::Executing,
                    crate::ros2_client::action::GoalStatusEnum::Canceling => ffi::ActionStatusEnum::Canceling,
                    crate::ros2_client::action::GoalStatusEnum::Succeeded => ffi::ActionStatusEnum::Succeeded,
                    crate::ros2_client::action::GoalStatusEnum::Canceled => ffi::ActionStatusEnum::Canceled,
                    crate::ros2_client::action::GoalStatusEnum::Aborted => ffi::ActionStatusEnum::Aborted,
                }
            }
        }
    };
    let u16str_decl = quote! {
        #[derive(Debug, Default, Clone, PartialEq, Eq, Serialize, Deserialize)]
        pub struct U16String {
            pub chars: Vec<u16>,
        }
    };
    let u16str_impl = quote! {
        impl ffi::U16String {
            #[allow(dead_code, clippy::should_implement_trait)]
            pub fn from_str(arg: &str) -> Self {
                Self {
                    chars: crate::_core::widestring::U16String::from_str(arg).into_vec(),
                }
            }
        }

        impl crate::_core::InternalDefault for ffi::U16String {
            fn _default() -> Self {
                Default::default()
            }
        }
    };

    let (attribute, ffi_imports, rust_imports) = if create_cxx_bridge {
        (
            quote! {
                #[cxx::bridge]
            },
            quote! {},
            generate_rust_imports_for_cxx(),
        )
    } else {
        (
            quote! {},
            quote! {
                use serde::{Deserialize, Serialize};
            },
            quote! {},
        )
    };

    let mut declares = vec![u16str_decl];
    let mut implements = vec![u16str_impl];

    if create_cxx_bridge {
        declares.push(cxx_ros2_decl);
        implements.push(cxx_ros2_impl);
    }

    quote! {
        #rust_imports

        #attribute
        pub mod ffi {
            #ffi_imports
            #(#declares)*
        }

        #(#implements)*
    }
}

fn generate_rust_imports_for_cxx() -> proc_macro2::TokenStream {
    quote! {
        #[allow(unused_imports)]
        use crate::prelude::*;
    }
}

fn generate_package_rust_imports_for_cxx() -> proc_macro2::TokenStream {
    let common = generate_rust_imports_for_cxx();
    quote! {
        #common
        #[allow(unused_imports)]
        use crate::ros2::default_impl::{
            ACTION_GOAL_TIMEOUT, ACTION_MAX_CONCURRENT_GOALS, GeneratedNode, MAX_PENDING_REQUESTS,
            PendingRequestId, SERVICE_RESPONSE_TIMEOUT, record_request,
        };
        #[cfg(feature = "rmw-zenoh")]
        #[allow(unused_imports)]
        use crate::ros2::default_impl::{ZENOH_ACTION_MAX_PAYLOAD, neutral_qos};
        #[cfg(feature = "rmw-zenoh")]
        #[allow(unused_imports)]
        use crate::ros2::default_impl::{zenoh_full_name, zenoh_service_endpoint};
    }
}

#[cfg(test)]
mod tests {
    use std::env::VarError;
    use std::ffi::OsString;

    use super::*;

    fn detect(rmw: Result<&str, VarError>, distro: Result<&str, VarError>) -> String {
        detect_ros_service_mapping_ident_from(rmw.map(String::from), distro.map(String::from))
            .to_string()
    }

    // ---- RMW_IMPLEMENTATION ----

    #[test]
    fn rmw_fastrtps_returns_enhanced() {
        assert_eq!(
            detect(Ok("rmw_fastrtps_cpp"), Err(VarError::NotPresent)),
            "Enhanced"
        );
    }

    #[test]
    fn rmw_cyclonedds_returns_cyclone() {
        assert_eq!(
            detect(Ok("rmw_cyclonedds_cpp"), Err(VarError::NotPresent)),
            "Cyclone"
        );
    }

    #[test]
    fn rmw_unknown_falls_back_to_enhanced() {
        assert_eq!(
            detect(Ok("rmw_some_third_party_dds"), Err(VarError::NotPresent)),
            "Enhanced"
        );
    }

    #[test]
    fn rmw_not_unicode_falls_back_to_enhanced() {
        let bytes = OsString::from("placeholder");
        let result = detect_ros_service_mapping_ident_from(
            Err(VarError::NotUnicode(bytes)),
            Err(VarError::NotPresent),
        );
        assert_eq!(result.to_string(), "Enhanced");
    }

    // ---- RMW takes precedence over ROS_DISTRO ----

    #[test]
    fn rmw_takes_precedence_over_distro_for_enhanced() {
        // RMW says fastrtps -> Enhanced, even though distro says galactic.
        assert_eq!(detect(Ok("rmw_fastrtps_cpp"), Ok("galactic")), "Enhanced");
    }

    #[test]
    fn rmw_takes_precedence_over_distro_for_cyclone() {
        // RMW says cyclonedds -> Cyclone, even though distro says humble.
        assert_eq!(detect(Ok("rmw_cyclonedds_cpp"), Ok("humble")), "Cyclone");
    }

    // ---- ROS_DISTRO fallback ----

    #[test]
    fn distro_humble_returns_enhanced() {
        assert_eq!(detect(Err(VarError::NotPresent), Ok("humble")), "Enhanced");
    }

    #[test]
    fn distro_iron_returns_enhanced() {
        assert_eq!(detect(Err(VarError::NotPresent), Ok("iron")), "Enhanced");
    }

    #[test]
    fn distro_jazzy_returns_enhanced() {
        assert_eq!(detect(Err(VarError::NotPresent), Ok("jazzy")), "Enhanced");
    }

    #[test]
    fn distro_kilted_returns_enhanced() {
        assert_eq!(detect(Err(VarError::NotPresent), Ok("kilted")), "Enhanced");
    }

    #[test]
    fn distro_rolling_returns_enhanced() {
        assert_eq!(detect(Err(VarError::NotPresent), Ok("rolling")), "Enhanced");
    }

    #[test]
    fn distro_galactic_returns_cyclone() {
        assert_eq!(detect(Err(VarError::NotPresent), Ok("galactic")), "Cyclone");
    }

    #[test]
    fn distro_unknown_falls_back_to_enhanced() {
        assert_eq!(
            detect(Err(VarError::NotPresent), Ok("future_distro_99")),
            "Enhanced"
        );
    }

    // ---- No env at all ----

    #[test]
    fn no_env_falls_back_to_enhanced() {
        // Most common case: neither env var set. Preserves historical
        // hardcoded-Enhanced behavior so existing builds keep working.
        assert_eq!(
            detect(Err(VarError::NotPresent), Err(VarError::NotPresent)),
            "Enhanced"
        );
    }

    #[test]
    fn transport_initializers_are_generated_for_rust_and_cxx() {
        let generated = generate_default_impls(true).to_string();
        assert!(generated.contains("init_ros2_context"));
        assert!(generated.contains("init_ros2_context_with_transport"));
        assert!(generated.contains("ZenohHumble"));
        assert!(generated.contains("ZenohRep2016"));
        assert!(generated.contains("dora_ros2_bridge :: transport :: Context"));
    }

    #[test]
    fn parameter_client_and_node_hosting_are_generated_for_cxx() {
        let generated = generate_default_impls(true).to_string();
        assert!(generated.contains("Ros2ParameterClient"));
        assert!(generated.contains("create_parameter_client"));
        assert!(generated.contains("set_parameter"));
        assert!(generated.contains("get_parameter"));
        assert!(generated.contains("take_event"));
    }

    #[test]
    fn cxx_package_modules_import_transport_neutral_node_type() {
        let imports = generate_package_rust_imports_for_cxx().to_string();
        assert!(imports.contains("GeneratedNode"));
    }

    /// The cxx-bridge codegen emits an `extern "C++"` block whose foreign type
    /// aliases (`type CombinedEvents = crate::ffi::CombinedEvents;`) parse as
    /// `syn::ForeignItem::Verbatim`. `format_token_stream` must print them
    /// rather than panic — this only holds with prettyplease's `verbatim`
    /// feature enabled, and no CI job builds this path against a real ROS 2
    /// install, so guard it here. See dora-rs/dora#2452.
    #[test]
    fn format_token_stream_handles_cxx_bridge_foreign_type_aliases() {
        use std::path::PathBuf;
        let package = Package {
            name: "test_pkg".to_string(),
            path: PathBuf::new(),
            dependencies: vec![],
            messages: vec![],
            services: vec![],
            actions: vec![],
        };
        let formatted = format_token_stream(generate_package(&package, true));
        assert!(formatted.contains("extern \"C++\""));
        assert!(formatted.contains("type CombinedEvents = crate::ffi::CombinedEvents;"));
    }
}
