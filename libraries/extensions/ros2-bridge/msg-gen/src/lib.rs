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
        let package_name = Ident::new(&package.name, Span::call_site());
        for message in &package.messages {
            let (def, imp) = message.struct_token_stream(&package_name, create_cxx_bridge);
            shared_type_defs.push(def);
            message_struct_impls.push(imp);
            if create_cxx_bridge {
                let (topic_def, topic_impl) = message.topic_def(&package_name);
                message_topic_defs.push(topic_def);
                message_topic_impls.push(topic_impl);
            }
        }
        aliases.push(package.aliases_token_stream());
    }

    let (attributes, imports_and_functions) = if create_cxx_bridge {
        (
            quote! { #[cxx::bridge] },
            quote! {
                extern "Rust" {
                    #(#message_topic_defs)*
                }
            },
        )
    } else {
        (
            quote! {},
            quote! {
                use serde::{Serialize, Deserialize};
            },
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
        #(#message_topic_impls)*

        #(#aliases)*
    }
}
