// Issue #18: Debug Command with Auto-TUI - Debug Module

pub mod complexity;
pub mod issue_detection;
pub mod session;
pub mod types;

pub use complexity::DebugComplexityAnalyzer;
pub use issue_detection::AutomaticIssueDetector;
pub use session::DebugSessionManager;
pub use types::*;
