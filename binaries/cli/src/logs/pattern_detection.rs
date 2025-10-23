// Log Pattern Detection for Issue #20

use super::types::*;
use chrono::{DateTime, Duration, Utc};
use eyre::Result;
use std::collections::HashMap;

/// Log pattern detector with multiple specialized detectors
#[derive(Debug)]
pub struct LogPatternDetector {
    error_spike_detector: ErrorSpikeDetector,
    repeating_error_detector: RepeatingErrorDetector,
    pattern_buffer: Vec<LogEntry>,
    analysis_window: Duration,
    max_buffer_size: usize,
}

impl LogPatternDetector {
    pub fn new(buffer_size: usize) -> Self {
        Self {
            error_spike_detector: ErrorSpikeDetector::new(),
            repeating_error_detector: RepeatingErrorDetector::new(),
            pattern_buffer: Vec::with_capacity(buffer_size),
            analysis_window: Duration::minutes(5),
            max_buffer_size: buffer_size,
        }
    }

    /// Analyze a new log entry and check for escalation triggers
    pub async fn analyze_log_stream(&mut self, log_entry: &LogEntry) -> Result<Option<EscalationTrigger>> {
        // Add log to pattern buffer
        self.pattern_buffer.push(log_entry.clone());

        // Trim buffer if too large
        if self.pattern_buffer.len() > self.max_buffer_size {
            let remove_count = self.pattern_buffer.len() - self.max_buffer_size;
            self.pattern_buffer.drain(0..remove_count);
        }

        // Get recent logs within analysis window
        let recent_logs = self.get_recent_logs();

        // Run pattern detection
        let mut detected_patterns = Vec::new();

        // Error spike detection
        if let Ok(spike_patterns) = self.error_spike_detector.detect_patterns(&recent_logs).await {
            detected_patterns.extend(spike_patterns);
        }

        // Repeating error detection
        if let Ok(repeat_patterns) = self.repeating_error_detector.detect_patterns(&recent_logs).await {
            detected_patterns.extend(repeat_patterns);
        }

        // Check for escalation triggers
        for pattern in &detected_patterns {
            if pattern.escalation_trigger {
                return Ok(Some(EscalationTrigger {
                    trigger_type: EscalationTriggerType::PatternDetected,
                    severity: pattern.severity,
                    reason: format!("Critical pattern detected: {}", pattern.title),
                    detected_pattern: Some(pattern.clone()),
                    suggested_action: "Switch to interactive analysis for detailed investigation".to_string(),
                }));
            }
        }

        Ok(None)
    }

    fn get_recent_logs(&self) -> Vec<LogEntry> {
        let cutoff = Utc::now() - self.analysis_window;
        self.pattern_buffer
            .iter()
            .filter(|log| log.timestamp > cutoff)
            .cloned()
            .collect()
    }

    pub fn get_detected_patterns(&self) -> Vec<DetectedPattern> {
        // Return cached patterns - in real implementation would maintain this
        Vec::new()
    }
}

/// Error spike detector
#[derive(Debug)]
pub struct ErrorSpikeDetector {
    spike_threshold: f32,
    baseline_error_rate: f32,
    window_size: Duration,
}

impl ErrorSpikeDetector {
    pub fn new() -> Self {
        Self {
            spike_threshold: 3.0, // 3x baseline
            baseline_error_rate: 5.0, // 5 errors per minute baseline
            window_size: Duration::minutes(1),
        }
    }

    pub async fn detect_patterns(&self, logs: &[LogEntry]) -> Result<Vec<DetectedPattern>> {
        let mut patterns = Vec::new();

        if logs.is_empty() {
            return Ok(patterns);
        }

        // Count errors in time window
        let error_count = logs
            .iter()
            .filter(|log| matches!(log.level, LogLevel::Error | LogLevel::Fatal))
            .count();

        let error_rate = error_count as f32 / self.window_size.num_minutes() as f32;

        // Detect spike
        if error_rate > self.baseline_error_rate * self.spike_threshold {
            let first_error = logs
                .iter()
                .find(|log| matches!(log.level, LogLevel::Error | LogLevel::Fatal));

            let last_error = logs
                .iter()
                .rfind(|log| matches!(log.level, LogLevel::Error | LogLevel::Fatal));

            if let (Some(first), Some(last)) = (first_error, last_error) {
                let pattern = DetectedPattern {
                    pattern_id: format!("error-spike-{}", Utc::now().timestamp()),
                    pattern_type: LogPatternType::ErrorSpike,
                    severity: self.calculate_spike_severity(error_rate),
                    confidence: 0.9,
                    title: format!("Error Spike: {} errors detected", error_count),
                    description: format!(
                        "Error rate ({:.1}/min) is {:.1}x higher than baseline ({:.1}/min)",
                        error_rate,
                        error_rate / self.baseline_error_rate,
                        self.baseline_error_rate
                    ),
                    sample_logs: logs
                        .iter()
                        .filter(|log| matches!(log.level, LogLevel::Error | LogLevel::Fatal))
                        .take(5)
                        .cloned()
                        .collect(),
                    frequency: PatternFrequency {
                        count: error_count as u32,
                        rate_per_minute: error_rate,
                        trend: FrequencyTrend::Spiking,
                    },
                    first_seen: first.timestamp,
                    last_seen: last.timestamp,
                    suggested_actions: vec![
                        LogAction {
                            action: "Investigate recent changes or deployments".to_string(),
                            command: Some("dora debug --focus errors --auto-detect".to_string()),
                            urgency: ActionUrgency::High,
                        },
                        LogAction {
                            action: "Check system resource usage".to_string(),
                            command: Some("dora analyze --focus resource".to_string()),
                            urgency: ActionUrgency::Medium,
                        },
                    ],
                    escalation_trigger: error_rate > self.baseline_error_rate * 5.0,
                };

                patterns.push(pattern);
            }
        }

        Ok(patterns)
    }

    fn calculate_spike_severity(&self, error_rate: f32) -> PatternSeverity {
        let multiplier = error_rate / self.baseline_error_rate;
        if multiplier > 10.0 {
            PatternSeverity::Critical
        } else if multiplier > 5.0 {
            PatternSeverity::High
        } else if multiplier > 3.0 {
            PatternSeverity::Medium
        } else {
            PatternSeverity::Low
        }
    }
}

/// Repeating error detector
#[derive(Debug)]
pub struct RepeatingErrorDetector {
    similarity_threshold: f32,
    frequency_threshold: u32,
}

impl RepeatingErrorDetector {
    pub fn new() -> Self {
        Self {
            similarity_threshold: 0.8,
            frequency_threshold: 5,
        }
    }

    pub async fn detect_patterns(&self, logs: &[LogEntry]) -> Result<Vec<DetectedPattern>> {
        let mut patterns = Vec::new();

        // Group similar error messages
        let error_groups = self.group_similar_errors(logs);

        for group in error_groups {
            if group.count >= self.frequency_threshold {
                let pattern = DetectedPattern {
                    pattern_id: format!("repeating-error-{}", Self::hash_signature(&group.signature)),
                    pattern_type: LogPatternType::RepeatingError,
                    severity: self.calculate_repetition_severity(group.count),
                    confidence: group.similarity_score,
                    title: format!("Repeating Error: {} occurrences", group.count),
                    description: format!("Error pattern repeated {} times", group.count),
                    sample_logs: group.sample_logs.clone(),
                    frequency: PatternFrequency {
                        count: group.count,
                        rate_per_minute: group.rate_per_minute,
                        trend: group.trend,
                    },
                    first_seen: group.first_seen,
                    last_seen: group.last_seen,
                    suggested_actions: vec![LogAction {
                        action: "Analyze error pattern for root cause".to_string(),
                        command: Some(format!("dora logs --search '{}' --analyze", &group.signature[..30.min(group.signature.len())])),
                        urgency: ActionUrgency::High,
                    }],
                    escalation_trigger: group.count > self.frequency_threshold * 3,
                };

                patterns.push(pattern);
            }
        }

        Ok(patterns)
    }

    fn group_similar_errors(&self, logs: &[LogEntry]) -> Vec<ErrorGroup> {
        let mut groups: HashMap<String, Vec<LogEntry>> = HashMap::new();

        // Group errors by simplified message
        for log in logs {
            if matches!(log.level, LogLevel::Error | LogLevel::Fatal) {
                let signature = self.extract_error_signature(&log.message);
                groups.entry(signature).or_insert_with(Vec::new).push(log.clone());
            }
        }

        // Convert to ErrorGroup structures
        groups
            .into_iter()
            .filter(|(_, logs)| logs.len() >= self.frequency_threshold as usize)
            .map(|(signature, logs)| {
                let count = logs.len() as u32;
                let first_seen = logs.first().map(|l| l.timestamp).unwrap_or_else(Utc::now);
                let last_seen = logs.last().map(|l| l.timestamp).unwrap_or_else(Utc::now);
                let duration = (last_seen - first_seen).num_minutes().max(1);
                let rate_per_minute = count as f32 / duration as f32;

                ErrorGroup {
                    signature,
                    count,
                    similarity_score: self.similarity_threshold,
                    sample_logs: logs.iter().take(3).cloned().collect(),
                    rate_per_minute,
                    trend: self.determine_trend(&logs),
                    first_seen,
                    last_seen,
                }
            })
            .collect()
    }

    fn extract_error_signature(&self, message: &str) -> String {
        // Simple signature extraction - remove numbers and timestamps
        message
            .chars()
            .filter(|c| !c.is_numeric())
            .collect::<String>()
            .split_whitespace()
            .take(10)
            .collect::<Vec<_>>()
            .join(" ")
    }

    fn determine_trend(&self, logs: &[LogEntry]) -> FrequencyTrend {
        if logs.len() < 2 {
            return FrequencyTrend::Stable;
        }

        // Simple trend calculation based on first half vs second half
        let mid = logs.len() / 2;
        let first_half_count = mid;
        let second_half_count = logs.len() - mid;

        if second_half_count > first_half_count * 3 / 2 {
            FrequencyTrend::Increasing
        } else if second_half_count < first_half_count * 2 / 3 {
            FrequencyTrend::Decreasing
        } else {
            FrequencyTrend::Stable
        }
    }

    fn calculate_repetition_severity(&self, count: u32) -> PatternSeverity {
        if count > 50 {
            PatternSeverity::Critical
        } else if count > 20 {
            PatternSeverity::High
        } else if count > 10 {
            PatternSeverity::Medium
        } else {
            PatternSeverity::Low
        }
    }

    fn hash_signature(signature: &str) -> u64 {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        let mut hasher = DefaultHasher::new();
        signature.hash(&mut hasher);
        hasher.finish()
    }
}

impl Default for ErrorSpikeDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for RepeatingErrorDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_error_spike_detection() {
        let detector = ErrorSpikeDetector::new();

        // Create logs with error spike
        let mut logs = Vec::new();
        for i in 0..20 {
            logs.push(LogEntry {
                timestamp: Utc::now(),
                level: LogLevel::Error,
                source: Some("test".to_string()),
                message: format!("Error {}", i),
                fields: HashMap::new(),
            });
        }

        let patterns = detector.detect_patterns(&logs).await.unwrap();
        assert!(!patterns.is_empty());
        assert!(patterns.iter().any(|p| matches!(p.pattern_type, LogPatternType::ErrorSpike)));
    }

    #[tokio::test]
    async fn test_repeating_error_detection() {
        let detector = RepeatingErrorDetector::new();

        // Create logs with repeating error
        let mut logs = Vec::new();
        for _ in 0..10 {
            logs.push(LogEntry {
                timestamp: Utc::now(),
                level: LogLevel::Error,
                source: Some("test".to_string()),
                message: "Connection refused".to_string(),
                fields: HashMap::new(),
            });
        }

        let patterns = detector.detect_patterns(&logs).await.unwrap();
        assert!(!patterns.is_empty());
        assert!(patterns.iter().any(|p| matches!(p.pattern_type, LogPatternType::RepeatingError)));
    }
}
