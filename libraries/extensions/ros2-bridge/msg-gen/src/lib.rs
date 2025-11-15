// Based on https://github.com/rclrust/rclrust/tree/3a48dbb8f23a3d67d3031351da3ed236a354f039/rclrust-msg-gen

#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

use std::path::Path;

use quote::{ToTokens, format_ident, quote};

pub mod parser;
pub mod types;

pub use crate::parser::get_packages;
use crate::types::Package;

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
            let (service_creation_def, service_creation_impl) =
                service.cxx_service_creation_functions(&package.name);
            service_creation_defs.push(service_creation_def);
            service_creation_impls.push(service_creation_impl);
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
        }
    }

    let aliases = package.aliases_token_stream();

    let (attributes, ffi_imports, extern_block, rust_imports) = if create_cxx_bridge {
        let reuse_bindings = package.reuse_bindings_token_stream();
        let rust_imports = generate_rust_imports_for_cxx();
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

                    #reuse_bindings
                }
            },
            quote! {
                #rust_imports

                use crate::ros2::default_impl::Ros2Node;
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
            fn init_ros2_context() -> Result<Box<Ros2Context>>;
            fn new_node(self: &Ros2Context, name_space: &str, base_name: &str) -> Result<Box<Ros2Node>>;
            fn qos_default() -> Ros2QosPolicies;
            fn actionqos_default() -> Ros2ActionClientQosPolicies;
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
    };
    let cxx_ros2_impl = quote! {
        pub struct Ros2Context{
            context: crate::ros2_client::Context,
            executor: std::sync::Arc<futures::executor::ThreadPool>,
        }

        fn init_ros2_context() -> eyre::Result<Box<Ros2Context>> {
            Ok(Box::new(Ros2Context{
                context: crate::ros2_client::Context::new()?,
                executor: std::sync::Arc::new(futures::executor::ThreadPool::new()?),
            }))
        }

        impl Ros2Context {
            fn new_node(&self, name_space: &str, base_name: &str) -> eyre::Result<Box<Ros2Node>> {
                use futures::task::SpawnExt as _;
                use eyre::WrapErr as _;

                let name = crate::ros2_client::NodeName::new(name_space, base_name).map_err(|e| eyre::eyre!(e))?;
                let options = crate::ros2_client::NodeOptions::new().enable_rosout(true);
                let mut node = self.context.new_node(name, options)
                    .map_err(|e| eyre::eyre!("failed to create ROS2 node: {e:?}"))?;

                let spinner = node.spinner().context("failed to create spinner")?;
                self.executor.spawn(async {
                    if let Err(err) = spinner.spin().await {
                        eprintln!("ros2 spinner failed: {err:?}");
                    }
                })
                .context("failed to spawn ros2 spinner")?;

                Ok(Box::new(Ros2Node{ node, executor: self.executor.clone(), }))
            }
        }

        pub struct Ros2Node {
            pub node : ros2_client::Node,
            pub executor: std::sync::Arc<futures::executor::ThreadPool>,
        }

        unsafe impl cxx::ExternType for Ros2Node {
            type Id = cxx::type_id!("Ros2Node");
            type Kind = cxx::kind::Opaque;
        }

        unsafe impl cxx::ExternType for Ros2Context {
            type Id = cxx::type_id!("Ros2Context");
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
    };
    let u16str_decl = quote! {
        #[derive(Debug, Default, Clone, PartialEq, Eq, Serialize, Deserialize)]
        pub struct U16String {
            pub chars: Vec<u16>,
        }
    };
    let u16str_impl = quote! {
        impl ffi::U16String {
            #[allow(dead_code)]
            fn from_str(arg: &str) -> Self {
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
