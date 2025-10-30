//! Shared boundary between the Dora framework and the TUI.
//!
//! See [`ADR-001`](../../../docs/tui-boundary/adr-001-tui-interface-boundary.md) for context.

pub mod data;
pub mod services;
pub mod error;

pub use data::*;
pub use services::*;
pub use error::*;
