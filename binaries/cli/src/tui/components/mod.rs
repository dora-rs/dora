use crossterm::event::KeyEvent;
/// TUI Component System - Issue #23
///
/// This module implements a reusable component system for the TUI,
/// allowing complex UI elements to be composed from smaller building blocks.
use ratatui::{Frame, layout::Rect};
use std::collections::HashMap;

use crate::tui::{Result, app::AppState, theme::ThemeConfig, views::ViewAction};

pub mod dashboard;
pub mod dataflow_list;
pub mod log_viewer_component;
pub mod metrics_chart;
pub mod status_bar;

pub use dashboard::{
    DataflowSummaryComponent, PerformanceChartsComponent, RecentActivityComponent,
    SystemOverviewComponent,
};
pub use dataflow_list::DataflowListComponent;
pub use log_viewer_component::LogViewerComponent;
pub use metrics_chart::MetricsChartComponent;
pub use status_bar::StatusBarComponent;

/// Unique identifier for a component instance
#[derive(Debug, Clone, Hash, Eq, PartialEq)]
pub struct ComponentId(pub String);

impl ComponentId {
    pub fn new(id: impl Into<String>) -> Self {
        Self(id.into())
    }
}

/// Component types for categorization
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ComponentType {
    DataflowList,
    NodeList,
    MetricsChart,
    LogViewer,
    StatusBar,
    CommandPalette,
    PropertyInspector,
    PerformanceGraph,
    ErrorList,
    HelpPanel,
}

/// Events that components can handle
#[derive(Debug, Clone)]
pub enum ComponentEvent {
    Key(KeyEvent),
    Focus(bool),
    Refresh,
    Custom(String),
}

impl ComponentEvent {
    pub fn from_key_event(key: KeyEvent) -> Self {
        ComponentEvent::Key(key)
    }
}

/// Core component trait - all UI components must implement this
///
/// Note: Using Future return types instead of async_trait to maintain object safety
pub trait Component: Send + Sync {
    /// Update component state
    fn update<'a>(
        &'a mut self,
        app_state: &'a AppState,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send + 'a>>;

    /// Render the component
    fn render(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig, app_state: &AppState);

    /// Handle component events
    fn handle_event<'a>(
        &'a mut self,
        event: ComponentEvent,
        app_state: &'a AppState,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<ViewAction>> + Send + 'a>>;

    /// Get component type
    fn component_type(&self) -> ComponentType;

    /// Whether this component can receive focus
    fn is_focusable(&self) -> bool {
        false
    }

    /// Whether this component currently has focus
    fn is_focused(&self) -> bool {
        false
    }

    /// Set focus state
    fn set_focus(&mut self, _focused: bool) {}

    /// Component initialization
    fn initialize<'a>(
        &'a mut self,
        _app_state: &'a AppState,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send + 'a>> {
        Box::pin(async { Ok(()) })
    }

    /// Component cleanup
    fn cleanup<'a>(
        &'a mut self,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send + 'a>> {
        Box::pin(async { Ok(()) })
    }
}

/// Component registry for managing component instances
pub struct ComponentRegistry {
    components: HashMap<ComponentId, Box<dyn Component>>,
}

impl ComponentRegistry {
    pub fn new() -> Self {
        Self {
            components: HashMap::new(),
        }
    }

    /// Register a component with an ID
    pub fn register(&mut self, id: ComponentId, component: Box<dyn Component>) {
        self.components.insert(id, component);
    }

    /// Get a component by ID
    pub fn get(&self, id: &ComponentId) -> Option<&dyn Component> {
        self.components.get(id).map(|c| c.as_ref())
    }

    /// Remove a component
    pub fn remove(&mut self, id: &ComponentId) -> Option<Box<dyn Component>> {
        self.components.remove(id)
    }

    /// Check if a component exists
    pub fn contains(&self, id: &ComponentId) -> bool {
        self.components.contains_key(id)
    }

    /// Get all component IDs
    pub fn component_ids(&self) -> Vec<ComponentId> {
        self.components.keys().cloned().collect()
    }

    /// Get number of components
    pub fn len(&self) -> usize {
        self.components.len()
    }

    /// Check if registry is empty
    pub fn is_empty(&self) -> bool {
        self.components.is_empty()
    }
}

impl Default for ComponentRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for ComponentRegistry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ComponentRegistry")
            .field("count", &self.components.len())
            .finish()
    }
}

/// Event dispatcher for routing events to components
pub struct EventDispatcher {
    focused_component: Option<ComponentId>,
}

impl EventDispatcher {
    pub fn new() -> Self {
        Self {
            focused_component: None,
        }
    }

    /// Set which component has focus
    pub fn set_focus(&mut self, component_id: Option<ComponentId>) {
        self.focused_component = component_id;
    }

    /// Get currently focused component
    pub fn get_focused(&self) -> Option<&ComponentId> {
        self.focused_component.as_ref()
    }

    /// Route an event to the appropriate component
    pub fn route_event(
        &self,
        event: &ComponentEvent,
        _registry: &ComponentRegistry,
    ) -> Option<ComponentId> {
        // For now, route to focused component
        // In the future, this could be more sophisticated (e.g., modal handling, global hotkeys)
        match event {
            ComponentEvent::Key(_) => self.focused_component.clone(),
            ComponentEvent::Focus(_) => None, // Focus events are handled separately
            ComponentEvent::Refresh => None,  // Refresh all components
            ComponentEvent::Custom(_) => self.focused_component.clone(),
        }
    }
}

impl Default for EventDispatcher {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for EventDispatcher {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EventDispatcher")
            .field("focused_component", &self.focused_component)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_component_id() {
        let id1 = ComponentId::new("test");
        let id2 = ComponentId::new("test");
        let id3 = ComponentId::new("other");

        assert_eq!(id1, id2);
        assert_ne!(id1, id3);
    }

    #[test]
    fn test_component_registry() {
        let registry = ComponentRegistry::new();
        assert!(registry.is_empty());
        assert_eq!(registry.len(), 0);

        let id = ComponentId::new("test");
        assert!(!registry.contains(&id));
    }

    #[test]
    fn test_event_dispatcher() {
        let mut dispatcher = EventDispatcher::new();
        assert!(dispatcher.get_focused().is_none());

        let id = ComponentId::new("test");
        dispatcher.set_focus(Some(id.clone()));
        assert_eq!(dispatcher.get_focused(), Some(&id));

        dispatcher.set_focus(None);
        assert!(dispatcher.get_focused().is_none());
    }
}
