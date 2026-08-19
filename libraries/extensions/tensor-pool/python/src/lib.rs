//! Pinned-host / CUDA tensor-pool transport for dora Python nodes.
//!
//! **Not covered by dora's 1.0 compatibility guarantees.** This is an opt-in
//! extension: its API may change or break in a minor release, and it carries
//! known open defects. See `libraries/extensions/tensor-pool/README.md`.
//!
//! It reaches dora only through the public extension channel
//! (`docs/extensions.md`) — dora has no knowledge of pools, CUDA or the
//! DORADMA segment layout.

pub mod seam;
pub mod transport;

pub use transport::Pool;
