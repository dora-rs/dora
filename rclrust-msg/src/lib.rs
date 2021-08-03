#![warn(rust_2018_idioms, elided_lifetimes_in_paths)]
#![allow(clippy::all)]

pub use rclrust_msg_core::traits;
pub use rclrust_msg_core::widestring;

include!(concat!(env!("OUT_DIR"), "/gen.rs"));
