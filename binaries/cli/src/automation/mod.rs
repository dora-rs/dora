//! Automation Context Detection System - Issue #15
//! 
//! This module provides comprehensive automation context detection to identify CI/CD environments,
//! scripting contexts, and automated workflows. It ensures the hybrid CLI provides appropriate
//! behavior for automation scenarios while optimizing for human vs. machine interaction patterns.

pub mod detector;
pub mod ci_detectors;
pub mod script_detectors;
pub mod machine_detectors;
pub mod pattern_analyzer;
pub mod confidence_calculator;

#[cfg(test)]
pub mod integration_tests;

pub use detector::{AutomationDetector, AutomationResult, AutomationType, DetectedEnvironment};
pub use ci_detectors::{CiEnvironmentDetector, CiDetectionResult, BuildInformation};
pub use script_detectors::{ScriptContextDetector, ScriptDetectionResult, ExecutionEnvironment};
pub use machine_detectors::{MachineInteractionDetector, MachineInteractionResult};