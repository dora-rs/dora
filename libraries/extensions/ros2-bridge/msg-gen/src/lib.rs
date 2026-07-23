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
    use rust_format::Formatter;
    let packages = get_packages(paths).unwrap();
    let mut mod_decl = vec![];
    let msg_dir = out_dir.join("msg");
    if !msg_dir.exists() {
        std::fs::create_dir(&msg_dir).unwrap();
    }
    // generate mod
    for package in packages.iter() {
        let mod_impl = generate_package(package, create_cxx_bridge);
        let generated_string = rust_format::PrettyPlease::default()
            .format_tokens(mod_impl)
            .unwrap();
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
        let generated_string = rust_format::PrettyPlease::default()
            .format_tokens(generated_default_impls)
            .unwrap();
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
            fn init_ros2_context() -> Result<Box<Ros2Context>>;
            fn init_ros2_context_with_transport(transport: Ros2TransportConfig) -> Result<Box<Ros2Context>>;
            fn new_node(self: &Ros2Context, name_space: &str, base_name: &str) -> Result<Box<Ros2Node>>;
            fn qos_default() -> Ros2QosPolicies;
            fn actionqos_default() -> Ros2ActionClientQosPolicies;
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
        }

        unsafe impl cxx::ExternType for Ros2Node {
            type Id = cxx::type_id!("Ros2Node");
            type Kind = cxx::kind::Opaque;
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

        #[cfg(feature = "rmw-zenoh")]
        pub fn neutral_qos(value: &ffi::Ros2QosPolicies) -> eyre::Result<dora_ros2_bridge::transport::Ros2Qos> {
            use dora_ros2_bridge::transport::{Durability, History, Liveliness, Reliability, Ros2Qos};
            let lease = value.lease_duration.is_finite().then(|| std::time::Duration::from_secs_f64(value.lease_duration.max(0.0)));
            let durability = match value.durability {
                ffi::Ros2Durability::Volatile => Durability::Volatile,
                ffi::Ros2Durability::TransientLocal => Durability::TransientLocal,
                _ => eyre::bail!("native Zenoh supports volatile and transient-local durability"),
            };
            let liveliness = match value.liveliness {
                ffi::Ros2Liveliness::Automatic => Liveliness::Automatic { lease_duration: lease },
                ffi::Ros2Liveliness::ManualByParticipant => Liveliness::ManualByParticipant { lease_duration: lease },
                ffi::Ros2Liveliness::ManualByTopic => Liveliness::ManualByTopic { lease_duration: lease },
                _ => eyre::bail!("unknown ROS2 liveliness policy"),
            };
            Ok(Ros2Qos {
                reliability: if value.reliable {
                    Reliability::Reliable { max_blocking_time: std::time::Duration::from_secs_f64(value.max_blocking_time.max(0.0)) }
                } else { Reliability::BestEffort },
                durability,
                history: if value.keep_all { History::KeepAll } else { History::KeepLast { depth: value.keep_last } },
                liveliness,
            })
        }

        impl From<ffi::Ros2QosPolicies> for rustdds::QosPolicies {
            fn from(value: ffi::Ros2QosPolicies) -> Self {
                rustdds::QosPolicyBuilder::new()
                    .durability(value.durability.into())
                    .liveliness(value.liveliness.convert(value.lease_duration))
                    .reliability(if value.reliable {
                        rustdds::policy::Reliability::Reliable {
                            max_blocking_time: rustdds::Duration::from_frac_seconds(
                                value.max_blocking_time,
                            ),
                        }
                    } else {
                        rustdds::policy::Reliability::BestEffort
                    })
                    .history(if value.keep_all {
                        rustdds::policy::History::KeepAll
                    } else {
                        rustdds::policy::History::KeepLast {
                            depth: value.keep_last,
                        }
                    })
                    .build()
            }
        }



        impl From<ffi::Ros2Durability> for rustdds::policy::Durability {
            fn from(value: ffi::Ros2Durability) -> Self {
                match value {
                    ffi::Ros2Durability::Volatile => rustdds::policy::Durability::Volatile,
                    ffi::Ros2Durability::TransientLocal => rustdds::policy::Durability::TransientLocal,
                    ffi::Ros2Durability::Transient => rustdds::policy::Durability::Transient,
                    ffi::Ros2Durability::Persistent => rustdds::policy::Durability::Persistent,
                    _ => unreachable!(), // required because enums are represented as integers in bridge
                }
            }
        }


        impl ffi::Ros2Liveliness {
            fn convert(self, lease_duration: f64) -> rustdds::policy::Liveliness {
                let lease_duration = if lease_duration.is_infinite() {
                    rustdds::Duration::INFINITE
                } else {
                    rustdds::Duration::from_frac_seconds(lease_duration)
                };
                match self {
                    ffi::Ros2Liveliness::Automatic => rustdds::policy::Liveliness::Automatic { lease_duration },
                    ffi::Ros2Liveliness::ManualByParticipant => {
                        rustdds::policy::Liveliness::ManualByParticipant { lease_duration }
                    }
                    ffi::Ros2Liveliness::ManualByTopic => rustdds::policy::Liveliness::ManualByTopic { lease_duration },
                    _ => unreachable!(), // required because enums are represented as integers in bridge
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
        use crate::ros2::default_impl::GeneratedNode;
        #[cfg(feature = "rmw-zenoh")]
        #[allow(unused_imports)]
        use crate::ros2::default_impl::neutral_qos;
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
    fn cxx_package_modules_import_transport_neutral_node_type() {
        let imports = generate_package_rust_imports_for_cxx().to_string();
        assert!(imports.contains("GeneratedNode"));
    }
}
