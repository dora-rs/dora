//! Shared boundary between the Dora framework and the TUI.
//!
//! See [`ADR-001`](../../../docs/tui-boundary/adr-001-tui-interface-boundary.md) for context.

pub mod data;
pub mod error;
pub mod services;

pub use data::*;
pub use error::*;
pub use services::*;
#[cfg(any(test, feature = "test-utils"))]
pub mod mocks;
#[cfg(any(test, feature = "test-utils"))]
pub use mocks::*;
