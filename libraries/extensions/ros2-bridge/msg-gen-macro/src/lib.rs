// Based on https://github.com/rclrust/rclrust/tree/3a48dbb8f23a3d67d3031351da3ed236a354f039/rclrust-msg-gen

#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

use std::path::Path;

use dora_ros2_bridge_msg_gen::get_packages;
use proc_macro::TokenStream;
use quote::quote;
use syn::{punctuated::Punctuated, MetaNameValue, Token};

struct Config {
    create_cxx_bridge: bool,
}

impl syn::parse::Parse for Config {
    fn parse(input: syn::parse::ParseStream<'_>) -> syn::Result<Self> {
        let punctuated = Punctuated::<MetaNameValue, Token![,]>::parse_terminated(input)?;

        let mut config = Self {
            create_cxx_bridge: false,
        };

        for item in &punctuated {
            let ident = item.path.require_ident()?;
            if ident == "cxx_bridge" {
                let syn::Expr::Lit(lit) = &item.value else {
                    return Err(syn::Error::new_spanned(&item.value, "invalid value, expected bool"));
                };
                let syn::Lit::Bool(b) = &lit.lit else {
                    return Err(syn::Error::new_spanned(&lit.lit, "invalid value, expected bool"));
                };
                config.create_cxx_bridge = b.value;
            } else {
                return Err(syn::Error::new_spanned(&item.path, "invalid argument"));
            }
        }

        Ok(config)
    }
}

#[proc_macro]
pub fn msg_include_all(input: TokenStream) -> TokenStream {
    let config = syn::parse_macro_input!(input as Config);

    let ament_prefix_path = std::env!("DETECTED_AMENT_PREFIX_PATH").trim();
    if ament_prefix_path.is_empty() {
        quote! {
            /// **No messages are available because the `AMENT_PREFIX_PATH` environment variable
            /// was not set during build.**
            pub const AMENT_PREFIX_PATH_NOT_SET: () = ();
        }
        .into()
    } else {
        let paths = ament_prefix_path
            .split(':')
            .map(Path::new)
            .collect::<Vec<_>>();

        let packages = get_packages(&paths)
            .unwrap()
            .iter()
            .map(|v| v.token_stream(config.create_cxx_bridge))
            .collect::<Vec<_>>();

        (quote! {
            #(#packages)*
        })
        .into()
    }
}
