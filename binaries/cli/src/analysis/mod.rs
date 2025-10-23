// Issue #13: ML-based complexity analysis
pub mod complexity;
pub mod command_complexity;
pub mod dataflow_complexity;
pub mod system_complexity;
pub mod ml_model;
pub mod explainability;

// Issue #14: Resource analysis system
pub mod resource_analyzer;
pub mod monitors;
pub mod anomaly_detection;
pub mod prediction;
pub mod health;
pub mod time_series;

#[cfg(test)]
mod tests;

// Issue #13 exports
pub use complexity::*;
pub use command_complexity::*;
pub use dataflow_complexity::*;
pub use system_complexity::*;
pub use ml_model::*;
pub use explainability::*;

// Issue #14 exports
pub use resource_analyzer::*;
pub use monitors::*;
pub use anomaly_detection::*;
pub use prediction::*;
pub use health::*;
pub use time_series::*;