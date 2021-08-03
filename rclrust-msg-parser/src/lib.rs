#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

pub mod action;
pub(crate) mod constant;
pub mod error;
pub(crate) mod ident;
pub(crate) mod literal;
pub(crate) mod member;
pub mod msg;
pub mod srv;
pub(crate) mod types;

pub use action::parse_action_file;
pub use msg::parse_message_file;
pub use srv::parse_service_file;
