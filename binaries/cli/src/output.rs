use std::hash::{DefaultHasher, Hash, Hasher};

use chrono::Local;
use colored::{Color, Colorize};
use dora_core::build::LogLevelOrStdout;
use dora_message::common::LogMessage;
pub fn print_log_message(
    log_message: LogMessage,
    print_dataflow_id: bool,
    print_daemon_name: bool,
) {
    let LogMessage {
        build_id: _,
        dataflow_id,
        node_id,
        daemon_id,
        level,
        target,
        module_path: _,
        file: _,
        line: _,
        message,
        timestamp,
        fields: _,
    } = log_message;
    let level = match level {
        LogLevelOrStdout::LogLevel(level) => match level {
            log::Level::Error => "ERROR ".red(),
            log::Level::Warn => "WARN  ".yellow(),
            log::Level::Info => "INFO  ".green(),
            log::Level::Debug => "DEBUG ".bright_blue(),
            log::Level::Trace => "TRACE ".dimmed(),
        },
        LogLevelOrStdout::Stdout => "stdout".bright_blue().italic().dimmed(),
    };

    let dataflow = match dataflow_id {
        Some(dataflow_id) if print_dataflow_id => format!("dataflow `{dataflow_id}` ").cyan(),
        _ => String::new().cyan(),
    };
    let daemon = match daemon_id {
        Some(id) if print_daemon_name => match id.machine_id() {
            Some(machine_id) => format!("on daemon `{machine_id}`"),
            None => "on default daemon ".to_string(),
        },
        None if print_daemon_name => "on default daemon".to_string(),
        _ => String::new(),
    }
    .bright_black();
    let time = format!("{}", timestamp.with_timezone(&Local).format("%H:%M:%S"));
    let colon = ":".bright_black().bold();
    let node = match node_id {
        Some(node_id) => {
            let node_id = node_id
                .to_string()
                .bold()
                .color(word_to_color(&node_id.to_string()));
            let padding = if daemon.is_empty() { "" } else { " " };
            format!("{node_id}{padding}{daemon}{colon} ")
        }
        None if daemon.is_empty() => "".into(),
        None => format!("{daemon}{colon} "),
    };
    let target = match target {
        Some(target) => format!("{target} ").dimmed(),
        None => "".normal(),
    };
    println!("{time} {level} {dataflow} {node}{target} {message}");
}

/// Generate a color for a word based on its semantic features
/// Optimized for technical abbreviations (stt, tts, llm, vlm, etc.)
pub fn word_to_color(word: &str) -> Color {
    let word_lower = word.to_lowercase();

    // Create a simple hash for the word
    let mut hasher = DefaultHasher::new();
    word_lower.hash(&mut hasher);
    let hash = hasher.finish();

    // Extract features from the word for similarity
    let length_factor = (word_lower.len() as f32 / 5.0).min(1.0);

    // Count repeated characters (stt has 2 t's, tts has 2 t's)
    let repeat_ratio = calculate_repeat_ratio(&word_lower);

    // Character diversity - unique chars / total chars
    let diversity = calculate_char_diversity(&word_lower);

    // Sum of character positions in alphabet (normalized)
    let char_sum = calculate_char_sum(&word_lower);

    // Blend hash-based color with feature-based adjustments
    let base_r = ((hash >> 16) & 0xFF) as u8;
    let base_g = ((hash >> 8) & 0xFF) as u8;
    let base_b = (hash & 0xFF) as u8;

    // Adjust colors based on word features for similarity
    // Similar abbreviations will have similar features
    let r = (base_r as f32 * 0.5 + repeat_ratio * 255.0 * 0.2 + char_sum * 0.3) as u8;
    let g = (base_g as f32 * 0.5 + diversity * 255.0 * 0.25 + length_factor * 255.0 * 0.25) as u8;
    let b =
        (base_b as f32 * 0.5 + (1.0 - repeat_ratio) * 255.0 * 0.3 + (1.0 - char_sum) * 0.2) as u8;

    Color::TrueColor { r, g, b }
}

/// Calculate ratio of repeated characters
fn calculate_repeat_ratio(s: &str) -> f32 {
    if s.is_empty() {
        return 0.0;
    }

    let mut char_counts = std::collections::HashMap::new();
    for c in s.chars() {
        *char_counts.entry(c).or_insert(0) += 1;
    }

    let repeated = char_counts.values().filter(|&&count| count > 1).count();
    repeated as f32 / char_counts.len().max(1) as f32
}

/// Calculate character diversity (unique chars / total chars)
fn calculate_char_diversity(s: &str) -> f32 {
    if s.is_empty() {
        return 0.0;
    }

    let unique: std::collections::HashSet<_> = s.chars().collect();
    unique.len() as f32 / s.len() as f32
}

/// Calculate normalized sum of character positions (a=1, z=26)
fn calculate_char_sum(s: &str) -> f32 {
    if s.is_empty() {
        return 0.0;
    }

    let sum: u32 = s
        .chars()
        .filter(|c| c.is_ascii_alphabetic())
        .map(|c| c.to_ascii_lowercase() as u32 - 'a' as u32 + 1)
        .sum();

    // Normalize by max possible sum for this length
    let max_sum = s.len() as u32 * 26;
    (sum as f32 / max_sum as f32).min(1.0)
}
