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

#[proc_macro]
pub fn msg_include_all(_input: TokenStream) -> TokenStream {
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
            .map(|v| v.token_stream())
            .collect::<Vec<_>>();

        (quote! {
            #(#packages)*
        })
        .into()
    }
}
