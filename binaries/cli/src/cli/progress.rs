use indicatif::{MultiProgress, ProgressBar, ProgressStyle as IndicatifProgressStyle};
use std::time::Duration;
use crate::cli::context::ExecutionContext;

#[derive(Debug, Clone)]
pub enum ProgressStyle {
    None,        // No progress indication
    Spinner,     // Simple spinner for indeterminate progress
    ProgressBar, // Progress bar for determinate progress
    Detailed,    // Detailed multi-phase progress
}

pub struct ProgressManager {
    style: ProgressStyle,
    multi_progress: MultiProgress,
}

impl ProgressManager {
    pub fn new(style: ProgressStyle) -> Self {
        Self {
            style,
            multi_progress: MultiProgress::new(),
        }
    }
    
    pub fn create_spinner(&self, message: &str) -> ProgressBar {
        let spinner = ProgressBar::new_spinner();
        spinner.set_style(
            IndicatifProgressStyle::default_spinner()
                .tick_chars("⠁⠂⠄⡀⢀⠠⠐⠈ ")
                .template("{spinner:.green} {msg}")
                .unwrap()
        );
        spinner.set_message(message.to_string());
        spinner.enable_steady_tick(Duration::from_millis(100));
        
        self.multi_progress.add(spinner)
    }
    
    pub fn create_progress_bar(&self, total: usize, message: &str) -> ProgressBar {
        let progress = ProgressBar::new(total as u64);
        progress.set_style(
            IndicatifProgressStyle::default_bar()
                .template("{msg} [{bar:40.cyan/blue}] {pos}/{len} ({eta})")
                .unwrap()
                .progress_chars("#>-")
        );
        progress.set_message(message.to_string());
        
        self.multi_progress.add(progress)
    }
    
    pub fn style(&self) -> &ProgressStyle {
        &self.style
    }
}

// Helper trait for commands to determine progress style
pub trait ProgressAware {
    fn determine_progress_style(&self, context: &ExecutionContext) -> ProgressStyle {
        if !context.is_tty || context.is_piped {
            return ProgressStyle::None;
        }
        
        // Default to spinner for interactive environments
        ProgressStyle::Spinner
    }
    
    fn estimate_operation_duration(&self) -> Duration {
        // Default estimate - commands can override
        Duration::from_secs(5)
    }
    
    fn should_show_progress(&self, context: &ExecutionContext) -> bool {
        context.is_tty && !context.is_piped
    }
}

// Progress feedback utilities
pub struct ProgressFeedback;

impl ProgressFeedback {
    pub fn for_context(context: &ExecutionContext, estimated_duration: Duration) -> ProgressStyle {
        if !context.is_tty || context.is_piped {
            return ProgressStyle::None;
        }
        
        match estimated_duration.as_secs() {
            0..=3 => ProgressStyle::None,
            4..=10 => ProgressStyle::Spinner,
            11..=30 => ProgressStyle::ProgressBar,
            _ => ProgressStyle::Detailed,
        }
    }
    
    pub fn create_simple_spinner(message: &str) -> ProgressBar {
        let spinner = ProgressBar::new_spinner();
        spinner.set_style(
            IndicatifProgressStyle::default_spinner()
                .tick_chars("⠁⠂⠄⡀⢀⠠⠐⠈ ")
                .template("{spinner:.green} {msg}")
                .unwrap()
        );
        spinner.set_message(message.to_string());
        spinner.enable_steady_tick(Duration::from_millis(100));
        spinner
    }
    
    pub fn create_simple_progress_bar(total: u64, message: &str) -> ProgressBar {
        let progress = ProgressBar::new(total);
        progress.set_style(
            IndicatifProgressStyle::default_bar()
                .template("{msg} [{bar:40.cyan/blue}] {pos}/{len} ({eta})")
                .unwrap()
                .progress_chars("#>-")
        );
        progress.set_message(message.to_string());
        progress
    }
}

// Phase-based progress tracking for complex operations
pub struct PhaseTracker {
    phases: Vec<ProgressPhase>,
    current_phase: usize,
    multi_progress: MultiProgress,
}

#[derive(Debug, Clone)]
pub struct ProgressPhase {
    pub name: String,
    pub description: String,
    pub total_items: Option<u64>,
}

impl PhaseTracker {
    pub fn new(phases: Vec<ProgressPhase>) -> Self {
        Self {
            phases,
            current_phase: 0,
            multi_progress: MultiProgress::new(),
        }
    }
    
    pub fn start_phase(&mut self, phase_index: usize) -> Option<ProgressBar> {
        if phase_index >= self.phases.len() {
            return None;
        }
        
        self.current_phase = phase_index;
        let phase = &self.phases[phase_index];
        
        let progress = if let Some(total) = phase.total_items {
            let pb = ProgressBar::new(total);
            pb.set_style(
                IndicatifProgressStyle::default_bar()
                    .template(&format!(
                        "Phase {}/{}: {{msg}} [{{bar:40.cyan/blue}}] {{pos}}/{{len}} ({{eta}})",
                        phase_index + 1,
                        self.phases.len()
                    ))
                    .unwrap()
                    .progress_chars("#>-")
            );
            pb
        } else {
            let pb = ProgressBar::new_spinner();
            pb.set_style(
                IndicatifProgressStyle::default_spinner()
                    .tick_chars("⠁⠂⠄⡀⢀⠠⠐⠈ ")
                    .template(&format!(
                        "Phase {}/{}: {{spinner:.green}} {{msg}}",
                        phase_index + 1,
                        self.phases.len()
                    ))
                    .unwrap()
            );
            pb.enable_steady_tick(Duration::from_millis(100));
            pb
        };
        
        progress.set_message(phase.description.clone());
        Some(self.multi_progress.add(progress))
    }
    
    pub fn complete_phase(&mut self, phase_index: usize, message: &str) {
        if phase_index < self.phases.len() {
            println!("✅ Phase {}/{}: {}", phase_index + 1, self.phases.len(), message);
        }
    }
    
    pub fn fail_phase(&mut self, phase_index: usize, message: &str) {
        if phase_index < self.phases.len() {
            println!("❌ Phase {}/{}: {}", phase_index + 1, self.phases.len(), message);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_progress_style_selection() {
        use crate::cli::{UiMode, OutputFormat};
        use crate::cli::context::{ExecutionEnvironment, TerminalCapabilities};
        use std::collections::HashMap;
        use std::path::PathBuf;
        
        // Test context-based style selection
        let context_interactive = ExecutionContext {
            working_dir: PathBuf::from("."),
            output_format: OutputFormat::Auto,
            ui_mode: Some(UiMode::Auto),
            no_hints: false,
            verbose: false,
            quiet: false,
            is_tty: true,
            is_piped: false,
            is_scripted: false,
            terminal_size: Some((80, 24)),
            user_preference: UiMode::Auto,
            environment: ExecutionEnvironment {
                ci_environment: None,
                shell_type: Some("bash".to_string()),
                relevant_env_vars: HashMap::new(),
                is_ci: false,
                is_automation: false,
            },
            terminal_capabilities: TerminalCapabilities {
                colors: true,
                interactive: true,
                width: Some(80),
                height: Some(24),
                tui_capable: true,
                supports_color: true,
                supports_unicode: true,
                supports_mouse: false,
                terminal_type: Some("xterm".to_string()),
            },
        };
        
        let context_piped = ExecutionContext {
            working_dir: PathBuf::from("."),
            output_format: OutputFormat::Auto,
            ui_mode: Some(UiMode::Auto),
            no_hints: false,
            verbose: false,
            quiet: false,
            is_tty: false,
            is_piped: true,
            is_scripted: false,
            terminal_size: None,
            user_preference: UiMode::Auto,
            environment: ExecutionEnvironment {
                ci_environment: None,
                shell_type: Some("bash".to_string()),
                relevant_env_vars: HashMap::new(),
                is_ci: false,
                is_automation: false,
            },
            terminal_capabilities: TerminalCapabilities {
                colors: false,
                interactive: false,
                width: None,
                height: None,
                tui_capable: false,
                supports_color: false,
                supports_unicode: false,
                supports_mouse: false,
                terminal_type: None,
            },
        };
        
        let style_interactive = ProgressFeedback::for_context(
            &context_interactive, 
            Duration::from_secs(15)
        );
        let style_piped = ProgressFeedback::for_context(
            &context_piped, 
            Duration::from_secs(15)
        );
        
        assert!(!matches!(style_interactive, ProgressStyle::None));
        assert!(matches!(style_piped, ProgressStyle::None));
    }
    
    #[test]
    fn test_duration_based_style() {
        use crate::cli::{UiMode, OutputFormat};
        use crate::cli::context::{ExecutionEnvironment, TerminalCapabilities};
        use std::collections::HashMap;
        use std::path::PathBuf;
        
        let context = ExecutionContext {
            working_dir: PathBuf::from("."),
            output_format: OutputFormat::Auto,
            ui_mode: Some(UiMode::Auto),
            no_hints: false,
            verbose: false,
            quiet: false,
            is_tty: true,
            is_piped: false,
            is_scripted: false,
            terminal_size: Some((80, 24)),
            user_preference: UiMode::Auto,
            environment: ExecutionEnvironment {
                ci_environment: None,
                shell_type: Some("bash".to_string()),
                relevant_env_vars: HashMap::new(),
                is_ci: false,
                is_automation: false,
            },
            terminal_capabilities: TerminalCapabilities {
                colors: true,
                interactive: true,
                width: Some(80),
                height: Some(24),
                tui_capable: true,
                supports_color: true,
                supports_unicode: true,
                supports_mouse: false,
                terminal_type: Some("xterm".to_string()),
            },
        };
        
        let short = ProgressFeedback::for_context(&context, Duration::from_secs(2));
        let medium = ProgressFeedback::for_context(&context, Duration::from_secs(8));
        let long = ProgressFeedback::for_context(&context, Duration::from_secs(20));
        let very_long = ProgressFeedback::for_context(&context, Duration::from_secs(60));
        
        assert!(matches!(short, ProgressStyle::None));
        assert!(matches!(medium, ProgressStyle::Spinner));
        assert!(matches!(long, ProgressStyle::ProgressBar));
        assert!(matches!(very_long, ProgressStyle::Detailed));
    }
    
    #[test]
    fn test_phase_tracker() {
        let phases = vec![
            ProgressPhase {
                name: "validation".to_string(),
                description: "Validating configuration".to_string(),
                total_items: None,
            },
            ProgressPhase {
                name: "building".to_string(),
                description: "Building nodes".to_string(),
                total_items: Some(5),
            },
        ];
        
        let mut tracker = PhaseTracker::new(phases);
        
        // Start first phase
        let phase1 = tracker.start_phase(0);
        assert!(phase1.is_some());
        
        // Complete first phase
        tracker.complete_phase(0, "Configuration validated");
        
        // Start second phase
        let phase2 = tracker.start_phase(1);
        assert!(phase2.is_some());
        
        // Complete second phase
        tracker.complete_phase(1, "All nodes built");
    }
}