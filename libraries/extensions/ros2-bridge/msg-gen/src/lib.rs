// Based on https://github.com/rclrust/rclrust/tree/3a48dbb8f23a3d67d3031351da3ed236a354f039/rclrust-msg-gen

#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

use std::path::Path;

use proc_macro2::Span;
use quote::quote;
use syn::Ident;

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
        aliases.push(package.aliases_token_stream());
    }

    let (attributes, imports_and_functions, cxx_bridge_impls) = if create_cxx_bridge {
        (
            quote! { #[cxx::bridge] },
            quote! {
                extern "Rust" {
                    type Ros2Context;
                    type Ros2Node;
                    fn init_ros2_context() -> Result<Box<Ros2Context>>;
                    fn new_node(self: &Ros2Context, name_space: &str, base_name: &str) -> Result<Box<Ros2Node>>;
                    #(#message_topic_defs)*
                }
            },
            quote! {
                struct Ros2Context(ros2_client::Context);

                fn init_ros2_context() -> eyre::Result<Box<Ros2Context>> {
                    Ok(Box::new(Ros2Context(ros2_client::Context::new()?)))
                }

                impl Ros2Context {
                    fn new_node(&self, name_space: &str, base_name: &str) -> eyre::Result<Box<Ros2Node>> {
                        let name = ros2_client::NodeName::new(name_space, base_name).map_err(|e| eyre::eyre!(e))?;
                        let options = ros2_client::NodeOptions::new().enable_rosout(true);
                        let node = self.0.new_node(name, options)?;
                        Ok(Box::new(Ros2Node(node)))
                    }
                }

                struct Ros2Node(ros2_client::Node);
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
                chars: Vec<u16>,
            }

            #(#shared_type_defs)*
        }


        impl crate::_core::InternalDefault for ffi::U16String {
            fn _default() -> Self {
                Default::default()
            }
        }

        #(#message_struct_impls)*

        #cxx_bridge_impls
        #(#message_topic_impls)*


        #(#aliases)*
    }
}
