use crate::automation::{AutomationDetector, AutomationResult, AutomationType};
use crate::cli::{OutputFormat, UiMode};
use std::io::IsTerminal;
use std::{collections::HashMap, path::PathBuf};

/// Execution context for commands - Enhanced for Issue #2
#[derive(Debug, Clone)]
pub struct ExecutionContext {
    /// Current working directory
    pub working_dir: PathBuf,

    /// Output format preference
    pub output_format: OutputFormat,

    /// UI mode preference
    pub ui_mode: Option<UiMode>,

    /// Whether hints are disabled
    pub no_hints: bool,

    /// Verbose mode
    pub verbose: bool,

    /// Quiet mode
    pub quiet: bool,

    /// Whether running in a TTY (interactive terminal)
    pub is_tty: bool,

    /// Whether output is being piped to another command
    pub is_piped: bool,

    /// Whether running in a scripting context (bash, CI, etc.)
    pub is_scripted: bool,

    /// Terminal size if available
    pub terminal_size: Option<(u16, u16)>,

    /// Current user preferences
    pub user_preference: UiMode,

    /// Detected terminal capabilities
    pub terminal_capabilities: TerminalCapabilities,

    /// Environment variables relevant to execution
    pub environment: ExecutionEnvironment,

    /// Comprehensive automation detection result (Issue #15)
    pub automation_result: Option<AutomationResult>,
}

/// CI environment types
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CiEnvironment {
    GitHubActions,
    GitLabCI,
    Jenkins,
    Buildkite,
    CircleCI,
    TravisCI,
    AppVeyor,
    AzurePipelines,
    TeamCity,
    Other(String),
}

#[derive(Debug, Clone)]
pub struct ExecutionEnvironment {
    /// CI/CD environment detected
    pub ci_environment: Option<CiEnvironment>,

    /// Shell type (bash, zsh, fish, etc.)
    pub shell_type: Option<String>,

    /// Environment variables affecting behavior
    pub relevant_env_vars: HashMap<String, String>,

    /// Is this a CI environment?
    pub is_ci: bool,

    /// Is this an automation context?
    pub is_automation: bool,
}

#[derive(Debug, Clone)]
pub struct TerminalCapabilities {
    /// Terminal supports colors
    pub colors: bool,

    /// Terminal supports interactive features
    pub interactive: bool,

    /// Terminal width
    pub width: Option<u16>,

    /// Terminal height
    pub height: Option<u16>,

    /// Terminal supports TUI
    pub tui_capable: bool,

    /// Supports color output
    pub supports_color: bool,

    /// Supports Unicode characters
    pub supports_unicode: bool,

    /// Supports mouse input
    pub supports_mouse: bool,

    /// Terminal type (xterm, screen, etc.)
    pub terminal_type: Option<String>,
}

impl ExecutionContext {
    /// Create context by detecting current environment
    pub fn detect() -> Self {
        let working_dir = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
        let terminal_capabilities = TerminalCapabilities::detect();
        let environment = ExecutionEnvironment::detect();

        Self {
            working_dir,
            output_format: OutputFormat::Auto,
            ui_mode: None,
            no_hints: false,
            verbose: false,
            quiet: false,
            is_tty: Self::detect_tty(),
            is_piped: Self::detect_piped(),
            is_scripted: Self::detect_scripted(),
            terminal_size: Self::detect_terminal_size(),
            user_preference: Self::load_user_preference(),
            terminal_capabilities,
            environment,
            automation_result: None,
        }
    }

    /// Create a new execution context from CLI arguments
    pub fn from_cli(cli: &crate::cli::Cli) -> Self {
        let mut context = Self::detect();

        // Override with CLI settings
        context.output_format = cli.output.clone();
        context.ui_mode = cli.ui_mode.clone();
        context.no_hints = cli.no_hints;
        context.verbose = cli.verbose;
        context.quiet = cli.quiet;

        // Update user preference if UI mode is specified
        if let Some(ui_mode) = &cli.ui_mode {
            context.user_preference = ui_mode.clone();
        }

        context
    }

    /// Fast synchronous detection for basic contexts
    pub fn detect_basic() -> Self {
        let working_dir = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
        let environment = ExecutionEnvironment::detect();
        let is_tty = Self::detect_tty();
        let is_piped = Self::detect_piped();
        let terminal_size = Self::detect_terminal_size();

        Self {
            working_dir,
            output_format: OutputFormat::Auto,
            ui_mode: None,
            no_hints: false,
            verbose: false,
            quiet: false,
            is_tty,
            is_piped,
            is_scripted: environment.is_ci || environment.is_automation,
            terminal_size,
            user_preference: UiMode::Auto,
            terminal_capabilities: TerminalCapabilities::detect_basic(),
            environment,
            automation_result: None,
        }
    }

    /// Comprehensive async detection with external tool checks
    pub async fn detect_comprehensive() -> Self {
        let mut context = Self::detect();
        context.terminal_capabilities = TerminalCapabilities::detect_comprehensive().await;
        context.environment = ExecutionEnvironment::detect_comprehensive().await;
        context = context.with_automation_detection();
        context
    }

    /// Run comprehensive automation detection and return updated context
    pub fn with_automation_detection(mut self) -> Self {
        let mut detector = AutomationDetector::new();
        self.automation_result = Some(detector.detect_automation_context(&self));
        self
    }

    /// Get automation type with fallback to basic detection
    pub fn get_automation_type(&self) -> AutomationType {
        self.automation_result
            .as_ref()
            .map(|result| result.automation_type.clone())
            .unwrap_or_else(|| {
                // Fallback to basic detection
                if self.environment.is_ci {
                    AutomationType::CiCdPipeline
                } else if self.is_scripted {
                    AutomationType::ScriptedExecution
                } else {
                    AutomationType::Interactive
                }
            })
    }

    /// Check if automation is detected with high confidence
    pub fn is_automated_context(&self) -> bool {
        self.automation_result
            .as_ref()
            .map(|result| result.is_automated && result.confidence >= 0.4)
            .unwrap_or_else(|| self.environment.is_automation)
    }

    // Detection methods
    fn detect_tty() -> bool {
        std::io::stdin().is_terminal() && std::io::stdout().is_terminal()
    }

    fn detect_piped() -> bool {
        !std::io::stdout().is_terminal()
    }

    fn detect_scripted() -> bool {
        // Check for CI environments
        if Self::is_ci_environment() {
            return true;
        }

        // Check for common scripting indicators
        if std::env::var("TERM").unwrap_or_default() == "dumb" {
            return true;
        }

        // Check parent process name
        if let Ok(parent) = Self::get_parent_process() {
            matches!(
                parent.as_str(),
                "bash" | "sh" | "zsh" | "fish" | "python" | "node"
            )
        } else {
            false
        }
    }

    fn detect_terminal_size() -> Option<(u16, u16)> {
        termsize::get().map(|size| (size.cols, size.rows))
    }

    fn load_user_preference() -> UiMode {
        // TODO: Load from config file or environment variable
        std::env::var("DORA_UI_MODE")
            .ok()
            .and_then(|mode| match mode.to_lowercase().as_str() {
                "cli" => Some(UiMode::Cli),
                "tui" => Some(UiMode::Tui),
                "minimal" => Some(UiMode::Minimal),
                "auto" => Some(UiMode::Auto),
                _ => None,
            })
            .unwrap_or(UiMode::Auto)
    }

    fn is_ci_environment() -> bool {
        std::env::var("CI").is_ok()
            || std::env::var("GITHUB_ACTIONS").is_ok()
            || std::env::var("GITLAB_CI").is_ok()
            || std::env::var("JENKINS_URL").is_ok()
            || std::env::var("BUILDKITE").is_ok()
            || std::env::var("CIRCLECI").is_ok()
            || std::env::var("TRAVIS").is_ok()
            || std::env::var("APPVEYOR").is_ok()
            || std::env::var("AZURE_PIPELINES").is_ok()
            || std::env::var("TEAMCITY_VERSION").is_ok()
    }

    fn get_parent_process() -> Result<String, Box<dyn std::error::Error>> {
        use sysinfo::{ProcessExt, System, SystemExt};

        let mut system = System::new();
        system.refresh_processes();

        let current_pid = sysinfo::get_current_pid()?;

        if let Some(process) = system.process(current_pid) {
            if let Some(parent_pid) = process.parent() {
                if let Some(parent_process) = system.process(parent_pid) {
                    return Ok(parent_process.name().to_string());
                }
            }
        }

        Err("Could not determine parent process".into())
    }

    /// Check if TUI mode is preferred or forced
    pub fn prefers_tui(&self) -> bool {
        matches!(self.ui_mode, Some(UiMode::Tui) | Some(UiMode::Auto))
            && self.terminal_capabilities.tui_capable
            && !self.environment.is_ci
    }

    /// Check if CLI mode is forced
    pub fn force_cli(&self) -> bool {
        matches!(self.ui_mode, Some(UiMode::Cli) | Some(UiMode::Minimal))
            || self.environment.is_ci
            || !self.terminal_capabilities.interactive
    }

    /// Should suggestions be shown?
    pub fn show_suggestions(&self) -> bool {
        !self.no_hints && !self.environment.is_ci && self.terminal_capabilities.interactive
    }

    /// Context-aware behavior helpers
    pub fn should_use_interactive_features(&self) -> bool {
        self.is_tty && !self.is_scripted && self.terminal_capabilities.supports_color
    }

    pub fn get_optimal_output_width(&self) -> usize {
        self.terminal_size
            .map(|(width, _)| width as usize)
            .unwrap_or(80) // Default width
    }

    pub fn supports_realtime_updates(&self) -> bool {
        self.is_tty && !self.is_piped && self.terminal_capabilities.supports_unicode
    }
}

impl Default for ExecutionContext {
    fn default() -> Self {
        Self::detect_basic()
    }
}

impl ExecutionEnvironment {
    /// Detect the current environment
    pub fn detect() -> Self {
        let env_vars: HashMap<String, String> = std::env::vars()
            .filter(|(key, _)| Self::is_relevant_env_key(key))
            .collect();

        let ci_environment = Self::detect_ci_environment(&env_vars);
        let is_ci = ci_environment.is_some();

        let shell_type = Self::detect_shell_type(&env_vars);

        let is_automation = is_ci
            || env_vars.contains_key("DORA_AUTOMATION")
            || env_vars.contains_key("ANSIBLE_INVENTORY")
            || env_vars.contains_key("PYTEST_CURRENT_TEST")
            || env_vars.contains_key("HTTP_USER_AGENT")
            || env_vars.get("TERM").map_or(false, |t| t == "dumb");

        Self {
            ci_environment,
            shell_type,
            relevant_env_vars: env_vars,
            is_ci,
            is_automation,
        }
    }

    /// Fast basic detection
    pub fn detect_basic() -> Self {
        Self::detect()
    }

    /// Comprehensive async detection
    pub async fn detect_comprehensive() -> Self {
        let env = Self::detect();

        // Could add external tool checks here if needed
        // For now, synchronous detection is comprehensive enough
        env
    }

    pub fn detect_ci_environment(env_vars: &HashMap<String, String>) -> Option<CiEnvironment> {
        if env_vars.contains_key("GITHUB_ACTIONS") {
            Some(CiEnvironment::GitHubActions)
        } else if env_vars.contains_key("GITLAB_CI") {
            Some(CiEnvironment::GitLabCI)
        } else if env_vars.contains_key("JENKINS_URL") {
            Some(CiEnvironment::Jenkins)
        } else if env_vars.contains_key("BUILDKITE") {
            Some(CiEnvironment::Buildkite)
        } else if env_vars.contains_key("CIRCLECI") {
            Some(CiEnvironment::CircleCI)
        } else if env_vars.contains_key("TRAVIS") {
            Some(CiEnvironment::TravisCI)
        } else if env_vars.contains_key("APPVEYOR") {
            Some(CiEnvironment::AppVeyor)
        } else if env_vars.contains_key("AZURE_PIPELINES") {
            Some(CiEnvironment::AzurePipelines)
        } else if env_vars.contains_key("TEAMCITY_VERSION") {
            Some(CiEnvironment::TeamCity)
        } else if env_vars.contains_key("CI") {
            Some(CiEnvironment::Other("Generic CI".to_string()))
        } else {
            None
        }
    }

    fn detect_shell_type(env_vars: &HashMap<String, String>) -> Option<String> {
        env_vars
            .get("SHELL")
            .and_then(|shell| shell.split('/').last().map(|s| s.to_string()))
            .or_else(|| {
                // Fallback to TERM_PROGRAM for some terminals
                env_vars.get("TERM_PROGRAM").cloned()
            })
    }

    fn is_relevant_env_key(key: &str) -> bool {
        // exact matches that we always care about
        const EXACT_KEYS: &[&str] = &["TERM", "SHELL", "COLORTERM", "TERM_PROGRAM", "_"];

        if EXACT_KEYS.contains(&key) {
            return true;
        }

        let upper = key.to_ascii_uppercase();

        const PREFIXES: &[&str] = &[
            "CI",
            "GITHUB",
            "GITLAB",
            "JENKINS",
            "BUILDKITE",
            "CIRCLE",
            "TRAVIS",
            "APPVEYOR",
            "AZURE",
            "TEAMCITY",
            "DORA",
            "ANSIBLE",
            "TF_",
            "TERRAFORM",
            "PUPPET",
            "CHEF",
            "SALT",
            "KUBERNETES",
            "KUBE_",
            "HELM",
            "DOCKER",
            "COMPOSE",
            "CONTAINER",
            "HTTP_",
            "NODE",
            "NPM",
            "PYTHON",
            "PYTEST",
            "VIRTUAL_ENV",
            "CONDA",
            "PS",
            "AWS_",
            "GOOGLE_",
        ];

        if PREFIXES.iter().any(|prefix| upper.starts_with(prefix)) {
            return true;
        }

        const SPECIFIC_KEYS: &[&str] = &[
            "API_KEY",
            "ACCESS_TOKEN",
            "AUTH_TOKEN",
            "BEARER_TOKEN",
            "CLIENT_ID",
            "CLIENT_SECRET",
            "WEBHOOK_URL",
            "API_SECRET",
            "OAUTH_TOKEN",
            "JWT_TOKEN",
            "CONTENT_TYPE",
            "ACCEPT",
            "npm_config_user_config",
            "npm_lifecycle_event",
            "PSModulePath",
            "PSExecutionPolicyPreference",
            "CURL_CA_BUNDLE",
            "WGETRC",
        ];

        SPECIFIC_KEYS
            .iter()
            .any(|entry| entry.eq_ignore_ascii_case(key))
    }
}

impl TerminalCapabilities {
    /// Detect terminal capabilities
    pub fn detect() -> Self {
        let supports_color = Self::detect_color_support();
        let interactive = Self::detect_interactive();
        let (width, height) = Self::detect_size();
        let supports_unicode = Self::detect_unicode_support();
        let supports_mouse = Self::detect_mouse_support();
        let terminal_type = Self::detect_terminal_type();

        let tui_capable =
            interactive && width.unwrap_or(0) >= 80 && height.unwrap_or(0) >= 24 && supports_color;

        Self {
            colors: supports_color,
            interactive,
            width,
            height,
            tui_capable,
            supports_color,
            supports_unicode,
            supports_mouse,
            terminal_type,
        }
    }

    /// Fast basic detection
    pub fn detect_basic() -> Self {
        let interactive = atty::is(atty::Stream::Stdout) && atty::is(atty::Stream::Stdin);
        let supports_color = std::env::var("NO_COLOR").is_err();

        Self {
            colors: supports_color,
            interactive,
            width: None,
            height: None,
            tui_capable: interactive,
            supports_color,
            supports_unicode: true, // Assume Unicode support for basic detection
            supports_mouse: false,  // Conservative assumption
            terminal_type: None,
        }
    }

    /// Comprehensive async detection
    pub async fn detect_comprehensive() -> Self {
        let caps = Self::detect();

        // Could add async terminal capability queries here
        // For now, synchronous detection is sufficient
        caps
    }

    fn detect_color_support() -> bool {
        // Check NO_COLOR environment variable (standard)
        if std::env::var("NO_COLOR").is_ok() {
            return false;
        }

        // Check TERM environment variable
        let term = std::env::var("TERM").unwrap_or_default();
        if term == "dumb" || term.is_empty() {
            return false;
        }

        // Check for color terminal indicators
        if term.contains("color")
            || term.contains("256")
            || term.contains("xterm")
            || term.contains("screen")
        {
            return true;
        }

        // Check COLORTERM environment variable
        if std::env::var("COLORTERM").is_ok() {
            return true;
        }

        // Check if stdout is a TTY
        atty::is(atty::Stream::Stdout)
    }

    fn detect_interactive() -> bool {
        atty::is(atty::Stream::Stdout) && atty::is(atty::Stream::Stdin)
    }

    fn detect_size() -> (Option<u16>, Option<u16>) {
        // Try multiple methods for size detection

        // Method 1: termsize crate
        if let Some(size) = termsize::get() {
            return (Some(size.cols), Some(size.rows));
        }

        // Method 2: term_size crate (fallback)
        if let Some((w, h)) = term_size::dimensions() {
            return (Some(w as u16), Some(h as u16));
        }

        // Method 3: crossterm
        if let Ok((w, h)) = crossterm::terminal::size() {
            return (Some(w), Some(h));
        }

        (None, None)
    }

    fn detect_unicode_support() -> bool {
        // Check locale settings
        let lang = std::env::var("LANG").unwrap_or_default();
        let lc_all = std::env::var("LC_ALL").unwrap_or_default();
        let lc_ctype = std::env::var("LC_CTYPE").unwrap_or_default();

        // Check for UTF-8 in locale variables
        if lang.contains("UTF-8") || lc_all.contains("UTF-8") || lc_ctype.contains("UTF-8") {
            return true;
        }

        // Check terminal type
        let term = std::env::var("TERM").unwrap_or_default();
        if term.contains("xterm") || term.contains("screen") {
            return true;
        }

        // Default to true for modern systems
        true
    }

    fn detect_mouse_support() -> bool {
        let term = std::env::var("TERM").unwrap_or_default();

        // Most modern terminals support mouse
        term.contains("xterm")
            || term.contains("screen")
            || term.contains("tmux")
            || std::env::var("TERM_PROGRAM").map_or(false, |prog| {
                prog.contains("iTerm") || prog.contains("Terminal") || prog.contains("Hyper")
            })
    }

    fn detect_terminal_type() -> Option<String> {
        // Primary: TERM environment variable
        if let Ok(term) = std::env::var("TERM") {
            if !term.is_empty() && term != "dumb" {
                return Some(term);
            }
        }

        // Secondary: TERM_PROGRAM environment variable
        if let Ok(term_program) = std::env::var("TERM_PROGRAM") {
            return Some(term_program);
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_context_detection() {
        let context = ExecutionContext::detect_basic();
        // Basic smoke test - ensure detection doesn't panic
        assert!(context.working_dir.exists() || context.working_dir == PathBuf::from("."));
    }

    #[test]
    fn test_tty_detection() {
        // This will vary based on test environment
        let is_tty = ExecutionContext::detect_tty();
        println!("TTY detected: {}", is_tty);
    }

    #[test]
    fn test_ci_environment_detection() {
        // Test CI environment detection with environment variables
        // Note: We test the logic without modifying global environment in tests

        // Create a mock environment for testing
        let mut env_vars = HashMap::new();

        // Test GitHub Actions detection
        env_vars.insert("GITHUB_ACTIONS".to_string(), "true".to_string());
        let ci_env = ExecutionEnvironment::detect_ci_environment(&env_vars);
        assert_eq!(ci_env, Some(CiEnvironment::GitHubActions));

        // Test GitLab CI detection
        env_vars.clear();
        env_vars.insert("GITLAB_CI".to_string(), "true".to_string());
        let ci_env = ExecutionEnvironment::detect_ci_environment(&env_vars);
        assert_eq!(ci_env, Some(CiEnvironment::GitLabCI));

        // Test generic CI detection
        env_vars.clear();
        env_vars.insert("CI".to_string(), "true".to_string());
        let ci_env = ExecutionEnvironment::detect_ci_environment(&env_vars);
        assert_eq!(ci_env, Some(CiEnvironment::Other("Generic CI".to_string())));

        // Test no CI environment
        env_vars.clear();
        let ci_env = ExecutionEnvironment::detect_ci_environment(&env_vars);
        assert_eq!(ci_env, None);
    }

    #[test]
    fn test_terminal_capabilities() {
        let caps = TerminalCapabilities::detect_basic();
        // Ensure basic detection works
        println!("Color support: {}", caps.supports_color);
        println!("Interactive: {}", caps.interactive);
    }

    #[test]
    fn test_environment_detection() {
        let env = ExecutionEnvironment::detect_basic();
        println!("CI detected: {}", env.is_ci);
        println!("Shell: {:?}", env.shell_type);
    }

    #[tokio::test]
    async fn test_comprehensive_detection() {
        let context = ExecutionContext::detect_comprehensive().await;
        // Validate comprehensive detection includes all fields
        assert!(context.working_dir.exists() || context.working_dir == PathBuf::from("."));
    }
}
