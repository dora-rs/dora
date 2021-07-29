#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

pub mod clock;
pub mod error;
pub(crate) mod internal;
pub mod log;
pub mod time;

pub use clock::{Clock, ClockType};
pub use time::Time;
