// Issue #16: User Preference Handling System
// Implements advanced preference resolution with learning and context-awareness

pub mod types;
pub mod preference_resolver;
pub mod context_preferences;
pub mod preference_learning;
pub mod preference_manager;

// Re-export main types
pub use types::*;
pub use preference_resolver::PreferenceResolver;
pub use context_preferences::{ContextAnalyzer, PreferenceContextMatcher};
pub use preference_learning::PreferenceLearningEngine;
pub use preference_manager::PreferenceManager;
