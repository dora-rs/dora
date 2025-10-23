// Configuration system for Dora CLI hybrid architecture
// Provides hierarchical configuration management with user preferences

pub mod manager;
pub mod structs;

pub use manager::{ConfigManager, ConfigPaths};
pub use structs::*;

/// Standard result type for configuration operations
pub type Result<T> = std::result::Result<T, anyhow::Error>;