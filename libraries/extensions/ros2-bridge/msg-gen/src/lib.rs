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

pub fn gen(paths: &[&Path], create_cxx_bridge: bool) -> proc_macro2::TokenStream {
    let message_structs = get_packages(&paths)
        .unwrap()
        .iter()
        .map(|v| v.message_structs(create_cxx_bridge))
        .collect::<Vec<_>>();
    let message_struct_defs = message_structs.iter().map(|(s, _)| s);
    let message_struct_impls = message_structs.iter().map(|(_, i)| i);

    let aliases = get_packages(&paths)
        .unwrap()
        .iter()
        .map(|v| v.aliases_token_stream())
        .collect::<Vec<_>>();
    let packages = get_packages(&paths)
        .unwrap()
        .iter()
        .map(|v| v.token_stream(create_cxx_bridge))
        .collect::<Vec<_>>();

    let (attributes, imports) = if create_cxx_bridge {
        (quote! { #[cxx::bridge] }, quote! {})
    } else {
        (
            quote! {},
            quote! {
                use serde::{Serialize, Deserialize};
            },
        )
    };

    (quote! {
        #attributes
        pub mod ffi {
            #imports

            #[derive(Debug, Default, Clone, PartialEq, Eq, Serialize, Deserialize)]
            pub struct U16String {
                chars: Vec<u16>,
            }

            #(#message_struct_defs)*
        }


        impl crate::_core::InternalDefault for ffi::U16String {
            fn _default() -> Self {
                Default::default()
            }
        }

        #(#message_struct_impls)*

        #(#aliases)*


        // #(#packages)*
    })
    .into()
}
