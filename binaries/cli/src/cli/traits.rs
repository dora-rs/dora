use crate::cli::context::ExecutionContext;
use eyre::Result;

/// Trait for command execution with context
#[allow(async_fn_in_trait)]
pub trait CommandExecute {
    async fn execute(&self, context: &ExecutionContext) -> Result<()>;
}

/// Trait for commands that can suggest interface escalation
pub trait InterfaceAware {
    /// Check if this command should suggest TUI mode
    fn should_suggest_tui(&self, context: &ExecutionContext) -> bool;

    /// Get the complexity score of this operation
    fn complexity_score(&self, context: &ExecutionContext) -> f64;

    /// Get TUI suggestion message
    fn tui_suggestion_message(&self) -> String {
        "this operation".to_string()
    }
}

/// Trait for commands that support progressive disclosure
pub trait ProgressiveDisclosure {
    /// Get the tier level of this command (1=basic, 2=enhanced, 3=TUI)
    fn tier_level(&self) -> u8;

    /// Check if this command can escalate to a higher tier
    fn can_escalate(&self) -> bool {
        self.tier_level() < 3
    }

    /// Get escalation suggestions
    fn escalation_suggestions(&self) -> Vec<String>;
}

/// Trait for backward compatibility with existing commands
pub trait BackwardCompatible {
    /// Convert to legacy command structure if needed
    fn to_legacy_command(&self) -> Option<crate::command::Command>;

    /// Check if this maintains backward compatibility
    fn is_backward_compatible(&self) -> bool {
        true
    }
}
