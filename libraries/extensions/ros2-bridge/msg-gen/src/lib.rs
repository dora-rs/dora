// Based on https://github.com/rclrust/rclrust/tree/3a48dbb8f23a3d67d3031351da3ed236a354f039/rclrust-msg-gen

#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

pub mod parser;
pub mod types;

pub use crate::parser::get_packages;
