// UX Consistency Validation
// Ensures consistent user experience across all CLI commands

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// UX validation result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UxValidationResult {
    pub passed: bool,
    pub violations: Vec<UxViolation>,
    pub warnings: Vec<UxWarning>,
    pub metrics: UxMetrics,
}

/// UX consistency violation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UxViolation {
    pub category: ViolationCategory,
    pub severity: Severity,
    pub description: String,
    pub location: String,
    pub recommendation: String,
}

/// UX warning (non-blocking issues)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UxWarning {
    pub category: WarningCategory,
    pub description: String,
    pub suggestion: String,
}

/// Violation categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ViolationCategory {
    InconsistentTerminology,
    InconsistentFormatting,
    MissingHints,
    InconsistentErrorHandling,
    MissingProgressFeedback,
    InconsistentOutputFormat,
}

/// Warning categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum WarningCategory {
    SuboptimalWording,
    VerbosityImbalance,
    MinorFormattingIssue,
    PerformanceHint,
}

/// Severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Severity {
    Critical,
    High,
    Medium,
    Low,
}

/// UX metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UxMetrics {
    pub total_commands_checked: usize,
    pub consistency_score: f32,
    pub terminology_consistency: f32,
    pub output_format_consistency: f32,
    pub error_handling_consistency: f32,
    pub hint_coverage: f32,
}

/// UX consistency validator
#[derive(Debug)]
pub struct UxValidator {
    terminology_rules: TerminologyRules,
    formatting_rules: FormattingRules,
    output_rules: OutputRules,
}

impl UxValidator {
    pub fn new() -> Self {
        Self {
            terminology_rules: TerminologyRules::default(),
            formatting_rules: FormattingRules::default(),
            output_rules: OutputRules::default(),
        }
    }

    /// Validate command UX consistency
    pub fn validate_command(&self, command_name: &str, command_output: &str) -> UxValidationResult {
        let mut violations = Vec::new();
        let warnings = Vec::new();

        // Check terminology consistency
        violations.extend(self.check_terminology(command_name, command_output));

        // Check output formatting
        violations.extend(self.check_formatting(command_name, command_output));

        // Check error messages
        violations.extend(self.check_error_messages(command_name, command_output));

        // Check hint presence
        if !self.has_hints(command_output) {
            violations.push(UxViolation {
                category: ViolationCategory::MissingHints,
                severity: Severity::Medium,
                description: format!("Command '{command_name}' output lacks user hints"),
                location: command_name.to_string(),
                recommendation: "Add contextual hints to guide users".to_string(),
            });
        }

        let metrics = self.calculate_metrics(command_name, command_output, &violations, &warnings);
        let passed = violations
            .iter()
            .all(|v| !matches!(v.severity, Severity::Critical | Severity::High));

        UxValidationResult {
            passed,
            violations,
            warnings,
            metrics,
        }
    }

    /// Validate consistency across multiple commands
    pub fn validate_cross_command_consistency(
        &self,
        commands: &HashMap<String, String>,
    ) -> UxValidationResult {
        let mut violations = Vec::new();
        let warnings = Vec::new();
        let total_commands = commands.len();

        // Check for consistent terminology across commands
        let terminology_violations = self.check_cross_command_terminology(commands);
        violations.extend(terminology_violations);

        // Check for consistent output formats
        let format_violations = self.check_cross_command_formats(commands);
        violations.extend(format_violations);

        // Check for consistent error handling
        let error_violations = self.check_cross_command_errors(commands);
        violations.extend(error_violations);

        let metrics = UxMetrics {
            total_commands_checked: total_commands,
            consistency_score: self.calculate_consistency_score(&violations, total_commands),
            terminology_consistency: self.calculate_terminology_score(&violations),
            output_format_consistency: self.calculate_format_score(&violations),
            error_handling_consistency: self.calculate_error_score(&violations),
            hint_coverage: self.calculate_hint_coverage(commands),
        };

        let passed = violations
            .iter()
            .all(|v| !matches!(v.severity, Severity::Critical | Severity::High));

        UxValidationResult {
            passed,
            violations,
            warnings,
            metrics,
        }
    }

    fn check_terminology(&self, command_name: &str, output: &str) -> Vec<UxViolation> {
        let mut violations = Vec::new();

        for (standard, variations) in &self.terminology_rules.standard_terms {
            let standard_present = output.contains(standard);
            let conflicting_terms: Vec<_> = variations
                .iter()
                .filter(|variant| output.contains(variant.as_str()))
                .collect();

            if !conflicting_terms.is_empty() {
                let conflicting_display = conflicting_terms
                    .iter()
                    .map(|term| term.as_str())
                    .collect::<Vec<_>>()
                    .join(", ");
                violations.push(UxViolation {
                    category: ViolationCategory::InconsistentTerminology,
                    severity: Severity::Medium,
                    description: format!(
                        "Inconsistent terminology for '{standard}': found {conflicting_display}"
                    ),
                    location: command_name.to_string(),
                    recommendation: format!(
                        "Replace variants with the preferred term '{standard}'"
                    ),
                });
            } else if !standard_present {
                // Highlight when none of the preferred terms appear but related concept does
                if variations.iter().any(|variant| output.contains(variant)) {
                    violations.push(UxViolation {
                        category: ViolationCategory::InconsistentTerminology,
                        severity: Severity::Low,
                        description: format!(
                            "Output references '{standard}' variations but omits the preferred term"
                        ),
                        location: command_name.to_string(),
                        recommendation: format!(
                            "Include the canonical term '{standard}' for consistency"
                        ),
                    });
                }
            }
        }

        violations
    }

    fn check_formatting(&self, _command_name: &str, output: &str) -> Vec<UxViolation> {
        let mut violations = Vec::new();

        let lines: Vec<&str> = output.lines().collect();
        let mut indent_patterns = HashMap::new();
        let mut long_lines = Vec::new();

        for (idx, line) in lines.iter().enumerate() {
            let indent = line.len() - line.trim_start().len();
            *indent_patterns.entry(indent).or_insert(0) += 1;

            if indent % self.formatting_rules.indent_size != 0 {
                violations.push(UxViolation {
                    category: ViolationCategory::InconsistentFormatting,
                    severity: Severity::Low,
                    description: format!(
                        "Line {} uses indentation of {} spaces (expected multiples of {})",
                        idx + 1,
                        indent,
                        self.formatting_rules.indent_size
                    ),
                    location: format!("line {}", idx + 1),
                    recommendation: "Adjust indentation to match project guidelines".to_string(),
                });
            }

            if line.len() > self.formatting_rules.max_line_length {
                long_lines.push(idx + 1);
            }
        }

        if !long_lines.is_empty() {
            violations.push(UxViolation {
                category: ViolationCategory::InconsistentFormatting,
                severity: Severity::Low,
                description: format!(
                    "{} lines exceed the configured maximum length of {} characters",
                    long_lines.len(),
                    self.formatting_rules.max_line_length
                ),
                location: "output formatting".to_string(),
                recommendation: "Wrap or truncate long lines to improve readability".to_string(),
            });
        }

        violations
    }

    fn check_error_messages(&self, _command_name: &str, output: &str) -> Vec<UxViolation> {
        let mut violations = Vec::new();

        // Check if errors include actionable suggestions
        if self.output_rules.require_error_suggestions
            && output.contains("Error:")
            && !output.contains("Try:")
            && !output.contains("Hint:")
        {
            violations.push(UxViolation {
                category: ViolationCategory::InconsistentErrorHandling,
                severity: Severity::Medium,
                description: "Error messages lack actionable suggestions".to_string(),
                location: "error output".to_string(),
                recommendation: "Add 'Try:' or 'Hint:' suggestions to errors".to_string(),
            });
        }

        violations
    }

    fn has_hints(&self, output: &str) -> bool {
        if !self.output_rules.require_hints {
            return true;
        }
        output.contains("Hint:") || output.contains("💡") || output.contains("Try:")
    }

    fn check_cross_command_terminology(
        &self,
        commands: &HashMap<String, String>,
    ) -> Vec<UxViolation> {
        let mut violations = Vec::new();
        let mut term_usage: HashMap<String, Vec<String>> = HashMap::new();

        for (cmd_name, output) in commands {
            for (standard_term, variations) in &self.terminology_rules.standard_terms {
                for variation in variations {
                    if output.contains(variation) && variation != standard_term {
                        term_usage
                            .entry(standard_term.to_string())
                            .or_default()
                            .push(format!("{cmd_name}: {variation}"));
                    }
                }
            }
        }

        // Report inconsistencies
        for (term, usages) in term_usage {
            if !usages.is_empty() {
                violations.push(UxViolation {
                    category: ViolationCategory::InconsistentTerminology,
                    severity: Severity::Medium,
                    description: format!("Inconsistent terminology for '{term}' across commands"),
                    location: usages.join(", "),
                    recommendation: format!("Standardize to '{term}'"),
                });
            }
        }

        violations
    }

    fn check_cross_command_formats(&self, _commands: &HashMap<String, String>) -> Vec<UxViolation> {
        // Simplified format checking
        Vec::new()
    }

    fn check_cross_command_errors(&self, _commands: &HashMap<String, String>) -> Vec<UxViolation> {
        // Simplified error checking
        Vec::new()
    }

    fn calculate_metrics(
        &self,
        _command_name: &str,
        _output: &str,
        violations: &[UxViolation],
        _warnings: &[UxWarning],
    ) -> UxMetrics {
        let total_checks = 10.0; // Simplified
        let passed_checks = total_checks - (violations.len() as f32);
        let consistency_score = (passed_checks / total_checks) * 100.0;

        UxMetrics {
            total_commands_checked: 1,
            consistency_score,
            terminology_consistency: 95.0,
            output_format_consistency: 90.0,
            error_handling_consistency: 85.0,
            hint_coverage: 80.0,
        }
    }

    fn calculate_consistency_score(
        &self,
        violations: &[UxViolation],
        total_commands: usize,
    ) -> f32 {
        let total_possible_issues = total_commands * 10; // Assume 10 checks per command
        let found_issues = violations.len();
        ((total_possible_issues - found_issues) as f32 / total_possible_issues as f32) * 100.0
    }

    fn calculate_terminology_score(&self, violations: &[UxViolation]) -> f32 {
        let terminology_violations = violations
            .iter()
            .filter(|v| matches!(v.category, ViolationCategory::InconsistentTerminology))
            .count();
        100.0 - (terminology_violations as f32 * 5.0).min(100.0)
    }

    fn calculate_format_score(&self, violations: &[UxViolation]) -> f32 {
        let format_violations = violations
            .iter()
            .filter(|v| {
                matches!(
                    v.category,
                    ViolationCategory::InconsistentFormatting
                        | ViolationCategory::InconsistentOutputFormat
                )
            })
            .count();
        100.0 - (format_violations as f32 * 5.0).min(100.0)
    }

    fn calculate_error_score(&self, violations: &[UxViolation]) -> f32 {
        let error_violations = violations
            .iter()
            .filter(|v| matches!(v.category, ViolationCategory::InconsistentErrorHandling))
            .count();
        100.0 - (error_violations as f32 * 5.0).min(100.0)
    }

    fn calculate_hint_coverage(&self, commands: &HashMap<String, String>) -> f32 {
        let total_commands = commands.len() as f32;
        let commands_with_hints = commands
            .values()
            .filter(|output| self.has_hints(output))
            .count() as f32;
        (commands_with_hints / total_commands) * 100.0
    }
}

impl Default for UxValidator {
    fn default() -> Self {
        Self::new()
    }
}

/// Terminology rules
#[derive(Debug, Clone)]
pub struct TerminologyRules {
    pub standard_terms: HashMap<String, Vec<String>>,
}

impl Default for TerminologyRules {
    fn default() -> Self {
        let mut standard_terms = HashMap::new();
        standard_terms.insert(
            "dataflow".to_string(),
            vec!["dataflows".to_string(), "data-flow".to_string()],
        );
        standard_terms.insert("node".to_string(), vec!["nodes".to_string()]);
        standard_terms.insert("operator".to_string(), vec!["operators".to_string()]);

        Self { standard_terms }
    }
}

/// Formatting rules
#[derive(Debug, Clone)]
pub struct FormattingRules {
    pub indent_size: usize,
    pub max_line_length: usize,
}

impl Default for FormattingRules {
    fn default() -> Self {
        Self {
            indent_size: 2,
            max_line_length: 100,
        }
    }
}

/// Output format rules
#[derive(Debug, Clone)]
pub struct OutputRules {
    pub require_hints: bool,
    pub require_examples: bool,
    pub require_error_suggestions: bool,
}

impl Default for OutputRules {
    fn default() -> Self {
        Self {
            require_hints: true,
            require_examples: true,
            require_error_suggestions: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ux_validator_basic() {
        let validator = UxValidator::new();
        let output = "Dataflow started successfully.\nHint: Use 'dora ps' to view status";
        let result = validator.validate_command("start", output);

        assert!(result.passed);
        assert!(result.metrics.consistency_score > 0.0);
    }

    #[test]
    fn test_terminology_violation() {
        let validator = UxValidator::new();
        let output = "Found 2 dataflows and 3 data flows running";
        let result = validator.validate_command("ps", output);

        let terminology_violations: Vec<_> = result
            .violations
            .iter()
            .filter(|v| matches!(v.category, ViolationCategory::InconsistentTerminology))
            .collect();

        assert!(!terminology_violations.is_empty());
    }

    #[test]
    fn test_missing_hints() {
        let validator = UxValidator::new();
        let output = "Command executed successfully.";
        let result = validator.validate_command("test", output);

        let hint_violations: Vec<_> = result
            .violations
            .iter()
            .filter(|v| matches!(v.category, ViolationCategory::MissingHints))
            .collect();

        assert!(!hint_violations.is_empty());
    }

    #[test]
    fn test_cross_command_consistency() {
        let validator = UxValidator::new();
        let mut commands = HashMap::new();
        commands.insert("ps".to_string(), "Listing dataflows...".to_string());
        commands.insert("start".to_string(), "Starting data flow...".to_string());

        let result = validator.validate_cross_command_consistency(&commands);

        // Should detect inconsistent terminology
        let terminology_violations: Vec<_> = result
            .violations
            .iter()
            .filter(|v| matches!(v.category, ViolationCategory::InconsistentTerminology))
            .collect();

        assert!(!terminology_violations.is_empty());
    }

    #[test]
    fn test_error_message_validation() {
        let validator = UxValidator::new();
        let output_without_suggestion = "Error: Failed to start dataflow";
        let result = validator.validate_command("start", output_without_suggestion);

        let error_violations: Vec<_> = result
            .violations
            .iter()
            .filter(|v| matches!(v.category, ViolationCategory::InconsistentErrorHandling))
            .collect();

        assert!(!error_violations.is_empty());
    }
}
