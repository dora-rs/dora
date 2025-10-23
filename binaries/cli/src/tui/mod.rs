pub mod app;
pub mod views;
pub mod theme;
pub mod cli_integration;

#[cfg(test)]
mod tests;

pub use app::{DoraApp, ViewType, AppState};
pub use theme::ThemeConfig;
pub use cli_integration::{CliContext, CommandMode};
pub use views::{View, ViewAction};

// Re-export common ratatui types for convenience
pub use ratatui::{
    backend::{Backend, CrosstermBackend},
    layout::{Constraint, Direction, Layout, Rect, Alignment},
    style::{Color, Modifier, Style},
    Frame,
    Terminal,
    widgets::*,
    text::{Line, Span, Text},
};

pub use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};

/// Standard error type for TUI operations
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;