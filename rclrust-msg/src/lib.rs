#![warn(rust_2018_idioms, elided_lifetimes_in_paths)]
#![allow(clippy::all)]

pub mod _core;

rclrust_msg_gen::msg_include_all!();
