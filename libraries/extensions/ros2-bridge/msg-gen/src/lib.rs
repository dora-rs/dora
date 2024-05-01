// Based on https://github.com/rclrust/rclrust/tree/3a48dbb8f23a3d67d3031351da3ed236a354f039/rclrust-msg-gen

#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

use std::path::Path;

use quote::quote;

pub mod parser;
pub mod types;

pub use crate::parser::get_packages;

pub fn gen<P>(paths: &[P], create_cxx_bridge: bool) -> proc_macro2::TokenStream
where
    P: AsRef<Path>,
{
    let packages = get_packages(paths).unwrap();
    let mut shared_type_defs = Vec::new();
    let mut message_struct_impls = Vec::new();
    let mut message_topic_defs = Vec::new();
    let mut message_topic_impls = Vec::new();
    let mut service_defs = Vec::new();
    let mut service_impls = Vec::new();
    let mut service_creation_defs = Vec::new();
    let mut service_creation_impls = Vec::new();
    let mut aliases = Vec::new();
    for package in &packages {
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

        aliases.push(package.aliases_token_stream());
    }

    let (attributes, imports_and_functions, cxx_bridge_impls) = if create_cxx_bridge {
        (
            quote! { #[cxx::bridge] },
            quote! {
                #[allow(dead_code)]
                extern "C++" {
                    type CombinedEvents = crate::ffi::CombinedEvents;
                    type CombinedEvent = crate::ffi::CombinedEvent;
                }

                extern "Rust" {
                    type Ros2Context;
                    type Ros2Node;
                    fn init_ros2_context() -> Result<Box<Ros2Context>>;
                    fn new_node(self: &Ros2Context, name_space: &str, base_name: &str) -> Result<Box<Ros2Node>>;
                    fn qos_default() -> Ros2QosPolicies;

                    #(#message_topic_defs)*
                    #(#service_creation_defs)*
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
            },
            quote! {
                struct Ros2Context{
                    context: ros2_client::Context,
                    executor: std::sync::Arc<futures::executor::ThreadPool>,
                }

                fn init_ros2_context() -> eyre::Result<Box<Ros2Context>> {
                    Ok(Box::new(Ros2Context{
                        context: ros2_client::Context::new()?,
                        executor: std::sync::Arc::new(futures::executor::ThreadPool::new()?),
                    }))
                }

                impl Ros2Context {
                    fn new_node(&self, name_space: &str, base_name: &str) -> eyre::Result<Box<Ros2Node>> {
                        use futures::task::SpawnExt as _;
                        use eyre::WrapErr as _;

                        let name = ros2_client::NodeName::new(name_space, base_name).map_err(|e| eyre::eyre!(e))?;
                        let options = ros2_client::NodeOptions::new().enable_rosout(true);
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

                struct Ros2Node {
                    node : ros2_client::Node,
                    executor: std::sync::Arc<futures::executor::ThreadPool>,
                }

                fn qos_default() -> ffi::Ros2QosPolicies {
                    ffi::Ros2QosPolicies::new(None, None, None, None, None, None, None)
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
            },
        )
    } else {
        (
            quote! {},
            quote! {
                use serde::{Serialize, Deserialize};
            },
            quote! {},
        )
    };

    quote! {
        #attributes
        mod ffi {
            #imports_and_functions

            #[derive(Debug, Default, Clone, PartialEq, Eq, Serialize, Deserialize)]
            pub struct U16String {
                pub chars: Vec<u16>,
            }

            #(#shared_type_defs)*
            #(#service_defs)*
        }


        impl crate::_core::InternalDefault for ffi::U16String {
            fn _default() -> Self {
                Default::default()
            }
        }

        impl ffi::U16String {
            fn from_str(arg: &str) -> Self {
                Self { chars: crate::_core::widestring::U16String::from_str(arg).into_vec()}
            }
        }

        #(#message_struct_impls)*

        #cxx_bridge_impls
        #(#message_topic_impls)*
        #(#service_creation_impls)*


        #(#service_impls)*

        #(#aliases)*
    }
}
