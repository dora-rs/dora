// Issue #18: Debug Command with Auto-TUI - Debug Module

pub mod types;
pub mod issue_detection;
pub mod complexity;
pub mod session;

pub use types::*;
pub use issue_detection::AutomaticIssueDetector;
pub use complexity::DebugComplexityAnalyzer;
pub use session::DebugSessionManager;
