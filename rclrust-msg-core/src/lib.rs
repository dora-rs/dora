#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

pub use widestring;

pub mod sequence;
pub mod string;
pub mod traits;

pub use sequence::{FFISeq, OwnedFFISeq, RefFFISeq};
pub use string::{FFIString, FFIWString, OwnedFFIString, OwnedFFIWString};
