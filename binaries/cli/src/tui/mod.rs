pub mod app;
pub mod cli_integration;
pub mod command_executor;
pub mod command_mode;
pub mod components;
pub mod metrics;
pub mod preferences;
pub mod theme;
pub mod views;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod command_mode_tests;

pub use app::{AppState, DoraApp, ViewType};
pub use cli_integration::{CliContext, CommandMode};
pub use command_executor::{
    CommandModeExecutionResult, CommandModeViewAction, CommandResult, StateUpdate, StatusLevel,
    TuiCliExecutor,
};
pub use components::{Component, ComponentId, ComponentRegistry, ComponentType, EventDispatcher};
pub use preferences::CliPreferencesStore;
pub use theme::{ThemeConfig, ThemeManager};
pub use views::{View, ViewAction};

// Re-export common ratatui types for convenience
pub use ratatui::{
    Frame, Terminal,
    backend::{Backend, CrosstermBackend},
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span, Text},
    widgets::*,
};

pub use crossterm::{
    event::{
        self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEvent, KeyEventKind,
        KeyModifiers,
    },
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};

/// Standard error type for TUI operations
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;
