# Issue #023: Build TUI Architecture Foundation

## 📋 Summary
Implement the foundational TUI architecture using Ratatui that provides a flexible, component-based framework for all interactive views. This architecture serves as the backbone for all TUI implementations in Phase 3, ensuring consistency, performance, and maintainability across all interactive interfaces.

## 🎯 Objectives
- Create a robust, component-based TUI architecture using Ratatui
- Implement flexible layout management and view composition system
- Add comprehensive event handling and state management
- Provide theming and styling system for consistent visual design
- Enable smooth integration with CLI commands and context preservation

**Success Metrics:**
- TUI launch time under 300ms from CLI escalation
- Frame rendering at consistent 60 FPS under normal load
- Memory usage under 50MB for typical TUI sessions
- Component reusability score above 80% across different views
- Theme and styling consistency score above 95%

## 🛠️ Technical Requirements

### What to Build

#### 1. Core TUI Application Framework
```rust
// src/tui/app.rs
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    backend::{Backend, CrosstermBackend},
    layout::{Constraint, Direction, Layout},
    style::{Color, Style},
    widgets::{Block, Borders},
    Frame, Terminal,
};
use std::io;
use tokio::sync::mpsc;

#[derive(Debug)]
pub struct DoraApp {
    should_quit: bool,
    view_stack: ViewStack,
    event_handler: EventHandler,
    state_manager: StateManager,
    theme_manager: ThemeManager,
    context: AppContext,
    performance_monitor: TuiPerformanceMonitor,
}

#[derive(Debug, Clone)]
pub struct AppContext {
    pub cli_context: CliContext,
    pub execution_context: ExecutionContext,
    pub user_preferences: UserPreferences,
    pub session_data: SessionData,
}

#[derive(Debug)]
pub struct ViewStack {
    views: Vec<Box<dyn View>>,
    current_view_index: usize,
    transition_state: TransitionState,
}

impl DoraApp {
    pub fn new_with_context(view_type: ViewType, cli_context: CliContext) -> Self {
        let context = AppContext {
            cli_context,
            execution_context: ExecutionContext::current(),
            user_preferences: UserPreferences::load_or_default(),
            session_data: SessionData::new(),
        };

        let initial_view = ViewFactory::create_view(view_type, &context);
        let mut view_stack = ViewStack::new();
        view_stack.push(initial_view);

        Self {
            should_quit: false,
            view_stack,
            event_handler: EventHandler::new(),
            state_manager: StateManager::new(),
            theme_manager: ThemeManager::load_default(),
            context,
            performance_monitor: TuiPerformanceMonitor::new(),
        }
    }

    pub async fn run(&mut self) -> io::Result<()> {
        // Setup terminal
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;

        // Create event channel
        let (event_tx, mut event_rx) = mpsc::unbounded_channel();
        
        // Spawn event handler
        let event_handler = self.event_handler.clone();
        tokio::spawn(async move {
            event_handler.run(event_tx).await;
        });

        // Initialize current view
        self.view_stack.current_view_mut()?.initialize(&self.context).await?;

        // Main application loop
        let mut last_frame = std::time::Instant::now();
        
        loop {
            // Handle events
            while let Ok(app_event) = event_rx.try_recv() {
                if self.handle_event(app_event).await? {
                    break;
                }
            }

            // Update current view
            let current_view = self.view_stack.current_view_mut()?;
            current_view.update(&mut self.context).await?;

            // Render frame
            let frame_start = std::time::Instant::now();
            terminal.draw(|f| self.render(f))?;
            
            // Monitor performance
            let frame_time = frame_start.elapsed();
            self.performance_monitor.record_frame_time(frame_time);

            // Target 60 FPS
            let target_frame_time = std::time::Duration::from_millis(16);
            if frame_time < target_frame_time {
                tokio::time::sleep(target_frame_time - frame_time).await;
            }

            if self.should_quit {
                break;
            }
        }

        // Cleanup
        disable_raw_mode()?;
        execute!(
            terminal.backend_mut(),
            LeaveAlternateScreen,
            DisableMouseCapture
        )?;
        terminal.show_cursor()?;

        Ok(())
    }

    fn render(&mut self, frame: &mut Frame) {
        let size = frame.size();
        
        // Apply theme
        let theme = self.theme_manager.current_theme();
        
        // Render current view
        if let Some(current_view) = self.view_stack.current_view() {
            current_view.render(frame, size, &theme, &self.context);
        }

        // Render overlays (modals, notifications, etc.)
        self.render_overlays(frame, size, &theme);
    }

    async fn handle_event(&mut self, event: AppEvent) -> io::Result<bool> {
        match event {
            AppEvent::Quit => {
                self.should_quit = true;
                return Ok(true);
            },
            
            AppEvent::Key(key_event) => {
                // Global key bindings
                match key_event.code {
                    KeyCode::Char('q') if key_event.modifiers.contains(event::KeyModifiers::CONTROL) => {
                        self.should_quit = true;
                        return Ok(true);
                    },
                    KeyCode::Esc => {
                        if self.view_stack.can_pop() {
                            self.pop_view().await?;
                        } else {
                            self.should_quit = true;
                            return Ok(true);
                        }
                    },
                    _ => {
                        // Forward to current view
                        if let Some(current_view) = self.view_stack.current_view_mut() {
                            let view_event = ViewEvent::Key(key_event);
                            let event_result = current_view.handle_event(view_event, &mut self.context).await?;
                            self.handle_view_event_result(event_result).await?;
                        }
                    }
                }
            },
            
            AppEvent::Mouse(mouse_event) => {
                if let Some(current_view) = self.view_stack.current_view_mut() {
                    let view_event = ViewEvent::Mouse(mouse_event);
                    let event_result = current_view.handle_event(view_event, &mut self.context).await?;
                    self.handle_view_event_result(event_result).await?;
                }
            },
            
            AppEvent::Resize(width, height) => {
                // Update context with new terminal size
                self.context.execution_context.terminal_size = Some((width, height));
                
                // Notify current view of resize
                if let Some(current_view) = self.view_stack.current_view_mut() {
                    current_view.handle_resize(width, height).await?;
                }
            },
            
            _ => {}
        }

        Ok(false)
    }

    async fn handle_view_event_result(&mut self, result: ViewEventResult) -> io::Result<()> {
        match result {
            ViewEventResult::None => {},
            
            ViewEventResult::PushView(view_type) => {
                let new_view = ViewFactory::create_view(view_type, &self.context);
                self.push_view(new_view).await?;
            },
            
            ViewEventResult::PopView => {
                self.pop_view().await?;
            },
            
            ViewEventResult::ReplaceView(view_type) => {
                let new_view = ViewFactory::create_view(view_type, &self.context);
                self.replace_view(new_view).await?;
            },
            
            ViewEventResult::Quit => {
                self.should_quit = true;
            },
            
            ViewEventResult::UpdateContext(context_update) => {
                self.apply_context_update(context_update);
            },
        }

        Ok(())
    }

    async fn push_view(&mut self, view: Box<dyn View>) -> io::Result<()> {
        view.initialize(&self.context).await?;
        self.view_stack.push(view);
        Ok(())
    }

    async fn pop_view(&mut self) -> io::Result<()> {
        if let Some(popped_view) = self.view_stack.pop() {
            popped_view.cleanup(&self.context).await?;
        }
        Ok(())
    }
}
```

#### 2. Component-Based View System
```rust
// src/tui/view.rs
use async_trait::async_trait;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Style},
    widgets::{Block, Borders, Widget},
    Frame,
};

#[async_trait]
pub trait View: Send + Sync {
    async fn initialize(&mut self, context: &AppContext) -> io::Result<()>;
    async fn update(&mut self, context: &mut AppContext) -> io::Result<()>;
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext);
    async fn handle_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult>;
    async fn handle_resize(&mut self, width: u16, height: u16) -> io::Result<()>;
    async fn cleanup(&self, context: &AppContext) -> io::Result<()>;
    
    fn view_type(&self) -> ViewType;
    fn title(&self) -> String;
    fn help_text(&self) -> Vec<HelpItem>;
}

#[derive(Debug, Clone)]
pub enum ViewType {
    Dashboard,
    DataflowList,
    DataflowDetail { dataflow_name: String },
    NodeInspector { dataflow_name: String, node_name: String },
    LogViewer { target: LogTarget, filters: LogFilters },
    PerformanceAnalyzer { analysis_type: AnalysisType },
    DebugSession { session_id: String, debug_mode: DebugMode },
    ResourceInspector { resource_id: String, resource_type: ResourceType },
    AnalysisExplorer { session_id: String, analysis_type: AnalysisType },
    HelpBrowser { topic: Option<String> },
    Settings,
}

#[derive(Debug, Clone)]
pub enum ViewEvent {
    Key(crossterm::event::KeyEvent),
    Mouse(crossterm::event::MouseEvent),
    Custom(CustomEvent),
}

#[derive(Debug, Clone)]
pub enum ViewEventResult {
    None,
    PushView(ViewType),
    PopView,
    ReplaceView(ViewType),
    Quit,
    UpdateContext(ContextUpdate),
}

// Base view implementation with common functionality
pub struct BaseView {
    pub layout_manager: LayoutManager,
    pub component_registry: ComponentRegistry,
    pub event_dispatcher: EventDispatcher,
    pub state: ViewState,
}

impl BaseView {
    pub fn new() -> Self {
        Self {
            layout_manager: LayoutManager::new(),
            component_registry: ComponentRegistry::new(),
            event_dispatcher: EventDispatcher::new(),
            state: ViewState::default(),
        }
    }

    pub fn add_component<T: Component + 'static>(&mut self, id: ComponentId, component: T) {
        self.component_registry.register(id, Box::new(component));
    }

    pub fn set_layout(&mut self, layout: LayoutConfig) {
        self.layout_manager.set_layout(layout);
    }

    pub fn render_components(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let layout_areas = self.layout_manager.calculate_layout(area);
        
        for (component_id, component_area) in layout_areas {
            if let Some(component) = self.component_registry.get(&component_id) {
                component.render(frame, component_area, theme, context);
            }
        }
    }

    pub async fn handle_component_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult> {
        // Determine which component should handle the event
        let target_component = self.event_dispatcher.route_event(&event, &self.component_registry);
        
        if let Some(component_id) = target_component {
            if let Some(component) = self.component_registry.get_mut(&component_id) {
                let component_event = ComponentEvent::from_view_event(event);
                return component.handle_event(component_event, context).await;
            }
        }

        Ok(ViewEventResult::None)
    }
}
```

#### 3. Reusable UI Components
```rust
// src/tui/components/mod.rs
use async_trait::async_trait;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Style},
    widgets::{Block, Borders, Clear, Gauge, List, ListItem, Paragraph, Table, Widget},
    Frame,
};

#[async_trait]
pub trait Component: Send + Sync {
    async fn update(&mut self, context: &AppContext) -> io::Result<()>;
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext);
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult>;
    
    fn component_type(&self) -> ComponentType;
    fn is_focusable(&self) -> bool;
    fn is_focused(&self) -> bool;
    fn set_focus(&mut self, focused: bool);
}

#[derive(Debug, Clone, Hash, Eq, PartialEq)]
pub struct ComponentId(pub String);

#[derive(Debug, Clone)]
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

// Dataflow List Component
pub struct DataflowListComponent {
    dataflows: Vec<DataflowInfo>,
    selected_index: usize,
    focused: bool,
    last_update: std::time::Instant,
    sort_by: DataflowSortBy,
    filter: DataflowFilter,
}

impl DataflowListComponent {
    pub fn new() -> Self {
        Self {
            dataflows: Vec::new(),
            selected_index: 0,
            focused: false,
            last_update: std::time::Instant::now(),
            sort_by: DataflowSortBy::Name,
            filter: DataflowFilter::All,
        }
    }

    fn render_dataflow_item(&self, dataflow: &DataflowInfo, selected: bool, theme: &Theme) -> ListItem {
        let status_icon = match dataflow.status {
            DataflowStatus::Running => "🟢",
            DataflowStatus::Stopped => "🔴",
            DataflowStatus::Starting => "🟡",
            DataflowStatus::Stopping => "🟠",
            DataflowStatus::Failed => "❌",
        };

        let style = if selected {
            theme.selected_item_style()
        } else {
            theme.normal_item_style()
        };

        let content = format!(
            "{} {} | Nodes: {} | Uptime: {}",
            status_icon,
            dataflow.name,
            dataflow.node_count,
            format_duration(dataflow.uptime)
        );

        ListItem::new(content).style(style)
    }
}

#[async_trait]
impl Component for DataflowListComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Refresh dataflow list periodically
        if self.last_update.elapsed() > std::time::Duration::from_secs(2) {
            // Fetch latest dataflow information
            if let Ok(daemon_client) = DaemonClient::connect().await {
                self.dataflows = daemon_client.list_dataflows().await
                    .unwrap_or_else(|_| Vec::new());
                
                // Apply current filter and sort
                self.apply_filter();
                self.apply_sort();
                
                // Ensure selected index is valid
                if self.selected_index >= self.dataflows.len() && !self.dataflows.is_empty() {
                    self.selected_index = self.dataflows.len() - 1;
                }
            }
            
            self.last_update = std::time::Instant::now();
        }

        Ok(())
    }

    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title("Dataflows")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });

        let items: Vec<ListItem> = self.dataflows.iter()
            .enumerate()
            .map(|(i, dataflow)| {
                self.render_dataflow_item(dataflow, i == self.selected_index, theme)
            })
            .collect();

        let list = List::new(items)
            .block(block)
            .highlight_style(theme.highlight_style())
            .highlight_symbol("▶ ");

        frame.render_stateful_widget(list, area, &mut ratatui::widgets::ListState::default().with_selected(Some(self.selected_index)));
    }

    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }

        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Up => {
                        if self.selected_index > 0 {
                            self.selected_index -= 1;
                        }
                    },
                    crossterm::event::KeyCode::Down => {
                        if self.selected_index + 1 < self.dataflows.len() {
                            self.selected_index += 1;
                        }
                    },
                    crossterm::event::KeyCode::Enter => {
                        if let Some(selected_dataflow) = self.dataflows.get(self.selected_index) {
                            return Ok(ViewEventResult::PushView(ViewType::DataflowDetail {
                                dataflow_name: selected_dataflow.name.clone()
                            }));
                        }
                    },
                    crossterm::event::KeyCode::Char('r') => {
                        // Refresh dataflows
                        self.last_update = std::time::Instant::now() - std::time::Duration::from_secs(3);
                    },
                    crossterm::event::KeyCode::Char('s') => {
                        // Cycle sort order
                        self.cycle_sort_order();
                        self.apply_sort();
                    },
                    crossterm::event::KeyCode::Char('f') => {
                        // Toggle filter
                        self.cycle_filter();
                        self.apply_filter();
                    },
                    _ => {}
                }
            },
            _ => {}
        }

        Ok(ViewEventResult::None)
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::DataflowList
    }

    fn is_focusable(&self) -> bool {
        true
    }

    fn is_focused(&self) -> bool {
        self.focused
    }

    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}

// Metrics Chart Component
pub struct MetricsChartComponent {
    metrics_data: VecDeque<MetricPoint>,
    chart_type: ChartType,
    time_window: Duration,
    focused: bool,
    auto_scale: bool,
    y_axis_range: (f64, f64),
}

impl MetricsChartComponent {
    pub fn new(chart_type: ChartType, time_window: Duration) -> Self {
        Self {
            metrics_data: VecDeque::new(),
            chart_type,
            time_window,
            focused: false,
            auto_scale: true,
            y_axis_range: (0.0, 100.0),
        }
    }

    fn create_chart_dataset(&self, theme: &Theme) -> Vec<ratatui::widgets::Dataset> {
        let data_points: Vec<(f64, f64)> = self.metrics_data.iter()
            .enumerate()
            .map(|(i, point)| (i as f64, point.value))
            .collect();

        vec![
            ratatui::widgets::Dataset::default()
                .name(&self.chart_type.display_name())
                .marker(ratatui::symbols::Marker::Braille)
                .style(theme.chart_line_style())
                .data(&data_points)
        ]
    }
}

#[async_trait]
impl Component for MetricsChartComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Add new metric points and maintain time window
        let now = std::time::Instant::now();
        let cutoff_time = now - self.time_window;
        
        // Remove old data points
        while let Some(front) = self.metrics_data.front() {
            if front.timestamp < cutoff_time {
                self.metrics_data.pop_front();
            } else {
                break;
            }
        }

        // Add new metric point (this would come from the metrics system)
        if let Some(new_value) = self.fetch_latest_metric_value().await? {
            self.metrics_data.push_back(MetricPoint {
                timestamp: now,
                value: new_value,
            });
        }

        // Auto-scale Y axis if enabled
        if self.auto_scale {
            self.calculate_y_axis_range();
        }

        Ok(())
    }

    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title(format!("{} - {}", self.chart_type.display_name(), format_duration(self.time_window)))
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });

        let datasets = self.create_chart_dataset(theme);
        
        let chart = ratatui::widgets::Chart::new(datasets)
            .block(block)
            .x_axis(
                ratatui::widgets::Axis::default()
                    .title("Time")
                    .style(theme.axis_style())
                    .bounds([0.0, self.metrics_data.len() as f64])
            )
            .y_axis(
                ratatui::widgets::Axis::default()
                    .title(self.chart_type.unit())
                    .style(theme.axis_style())
                    .bounds([self.y_axis_range.0, self.y_axis_range.1])
            );

        frame.render_widget(chart, area);
    }

    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }

        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Char('a') => {
                        self.auto_scale = !self.auto_scale;
                    },
                    crossterm::event::KeyCode::Char('+') => {
                        // Zoom in (decrease time window)
                        self.time_window = (self.time_window / 2).max(Duration::from_secs(10));
                    },
                    crossterm::event::KeyCode::Char('-') => {
                        // Zoom out (increase time window)
                        self.time_window = (self.time_window * 2).min(Duration::from_hours(24));
                    },
                    _ => {}
                }
            },
            _ => {}
        }

        Ok(ViewEventResult::None)
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::MetricsChart
    }

    fn is_focusable(&self) -> bool {
        true
    }

    fn is_focused(&self) -> bool {
        self.focused
    }

    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}
```

#### 4. Theme and Styling System
```rust
// src/tui/theme.rs
use ratatui::style::{Color, Modifier, Style};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Theme {
    pub name: String,
    pub colors: ColorScheme,
    pub styles: StyleScheme,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColorScheme {
    pub background: Color,
    pub foreground: Color,
    pub primary: Color,
    pub secondary: Color,
    pub accent: Color,
    pub success: Color,
    pub warning: Color,
    pub error: Color,
    pub info: Color,
    pub muted: Color,
    pub border: Color,
    pub border_focused: Color,
    pub selection: Color,
    pub highlight: Color,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StyleScheme {
    pub normal: StyleConfig,
    pub focused: StyleConfig,
    pub selected: StyleConfig,
    pub highlighted: StyleConfig,
    pub disabled: StyleConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StyleConfig {
    pub fg: Option<Color>,
    pub bg: Option<Color>,
    pub modifiers: Vec<Modifier>,
}

impl Theme {
    pub fn default_dark() -> Self {
        Self {
            name: "Dark".to_string(),
            colors: ColorScheme {
                background: Color::Black,
                foreground: Color::White,
                primary: Color::Blue,
                secondary: Color::Cyan,
                accent: Color::Magenta,
                success: Color::Green,
                warning: Color::Yellow,
                error: Color::Red,
                info: Color::Blue,
                muted: Color::DarkGray,
                border: Color::Gray,
                border_focused: Color::Blue,
                selection: Color::Blue,
                highlight: Color::Yellow,
            },
            styles: StyleScheme::default(),
        }
    }

    pub fn default_light() -> Self {
        Self {
            name: "Light".to_string(),
            colors: ColorScheme {
                background: Color::White,
                foreground: Color::Black,
                primary: Color::Blue,
                secondary: Color::Cyan,
                accent: Color::Magenta,
                success: Color::Green,
                warning: Color::Rgb(255, 165, 0), // Orange
                error: Color::Red,
                info: Color::Blue,
                muted: Color::Gray,
                border: Color::DarkGray,
                border_focused: Color::Blue,
                selection: Color::LightBlue,
                highlight: Color::Yellow,
            },
            styles: StyleScheme::default(),
        }
    }

    // Style helper methods
    pub fn normal_style(&self) -> Style {
        Style::default()
            .fg(self.colors.foreground)
            .bg(self.colors.background)
    }

    pub fn focused_style(&self) -> Style {
        Style::default()
            .fg(self.colors.foreground)
            .bg(self.colors.background)
            .add_modifier(Modifier::BOLD)
    }

    pub fn selected_item_style(&self) -> Style {
        Style::default()
            .fg(self.colors.background)
            .bg(self.colors.selection)
            .add_modifier(Modifier::BOLD)
    }

    pub fn normal_item_style(&self) -> Style {
        Style::default()
            .fg(self.colors.foreground)
            .bg(self.colors.background)
    }

    pub fn focused_border_style(&self) -> Style {
        Style::default().fg(self.colors.border_focused)
    }

    pub fn normal_border_style(&self) -> Style {
        Style::default().fg(self.colors.border)
    }

    pub fn highlight_style(&self) -> Style {
        Style::default()
            .fg(self.colors.background)
            .bg(self.colors.highlight)
            .add_modifier(Modifier::BOLD)
    }

    pub fn success_style(&self) -> Style {
        Style::default().fg(self.colors.success)
    }

    pub fn warning_style(&self) -> Style {
        Style::default().fg(self.colors.warning)
    }

    pub fn error_style(&self) -> Style {
        Style::default().fg(self.colors.error)
    }

    pub fn info_style(&self) -> Style {
        Style::default().fg(self.colors.info)
    }

    pub fn muted_style(&self) -> Style {
        Style::default().fg(self.colors.muted)
    }

    pub fn chart_line_style(&self) -> Style {
        Style::default().fg(self.colors.primary)
    }

    pub fn axis_style(&self) -> Style {
        Style::default().fg(self.colors.muted)
    }
}

#[derive(Debug)]
pub struct ThemeManager {
    current_theme: Theme,
    available_themes: Vec<Theme>,
    user_theme_dir: PathBuf,
}

impl ThemeManager {
    pub fn load_default() -> Self {
        let mut themes = vec![
            Theme::default_dark(),
            Theme::default_light(),
        ];

        // Load user custom themes
        let user_theme_dir = dirs::config_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join("dora")
            .join("themes");

        if let Ok(custom_themes) = Self::load_custom_themes(&user_theme_dir) {
            themes.extend(custom_themes);
        }

        Self {
            current_theme: themes[0].clone(),
            available_themes: themes,
            user_theme_dir,
        }
    }

    pub fn current_theme(&self) -> &Theme {
        &self.current_theme
    }

    pub fn switch_theme(&mut self, theme_name: &str) -> Result<(), String> {
        if let Some(theme) = self.available_themes.iter().find(|t| t.name == theme_name) {
            self.current_theme = theme.clone();
            Ok(())
        } else {
            Err(format!("Theme '{}' not found", theme_name))
        }
    }

    pub fn available_theme_names(&self) -> Vec<String> {
        self.available_themes.iter().map(|t| t.name.clone()).collect()
    }

    fn load_custom_themes(theme_dir: &Path) -> Result<Vec<Theme>, Box<dyn std::error::Error>> {
        let mut themes = Vec::new();

        if theme_dir.exists() {
            for entry in std::fs::read_dir(theme_dir)? {
                let entry = entry?;
                let path = entry.path();
                
                if path.extension().and_then(|s| s.to_str()) == Some("toml") {
                    let content = std::fs::read_to_string(&path)?;
                    let theme: Theme = toml::from_str(&content)?;
                    themes.push(theme);
                }
            }
        }

        Ok(themes)
    }
}
```

### Why This Approach

**Component-Based Architecture:**
- Reusable components across different views
- Consistent behavior and styling
- Easy maintenance and testing

**Performance Optimized:**
- 60 FPS rendering with frame timing control
- Efficient event handling and state management
- Memory-conscious component lifecycle

**Flexible and Extensible:**
- View stack for complex navigation
- Theme system for customization
- Event-driven architecture for responsiveness

### How to Implement

#### Step 1: Core Application Framework (6 hours)
1. **Implement DoraApp** with main application loop
2. **Add ViewStack** for view management and navigation
3. **Create EventHandler** for terminal event processing
4. **Add performance monitoring** and frame rate control

#### Step 2: Component System (5 hours)
1. **Implement Component trait** and base functionality
2. **Create ComponentRegistry** for component management
3. **Add DataflowListComponent** and MetricsChartComponent
4. **Build component event routing** and handling

#### Step 3: View Foundation (4 hours)
1. **Implement View trait** and BaseView
2. **Create ViewFactory** for view instantiation
3. **Add view lifecycle** management (initialize, update, cleanup)
4. **Build view event handling** system

#### Step 4: Theme and Styling (3 hours)
1. **Implement Theme** system with color schemes
2. **Add ThemeManager** with theme switching
3. **Create default themes** (dark and light)
4. **Add custom theme** loading support

#### Step 5: Integration and Testing (2 hours)
1. **Add comprehensive unit tests** for all components
2. **Test view transitions** and event handling
3. **Validate performance** targets and optimization
4. **Test theme system** and styling consistency

## 🔗 Dependencies
**Depends On:**
- Phase 1 foundation issues (CLI framework, execution context)
- Phase 2 interface selection and complexity analysis

**Enables:**
- All specific TUI view implementations in Phase 3
- Interactive command integrations from Phase 2

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_component_focus_management() {
        let mut component = DataflowListComponent::new();
        
        assert!(!component.is_focused());
        component.set_focus(true);
        assert!(component.is_focused());
    }
    
    #[test]
    fn test_theme_switching() {
        let mut theme_manager = ThemeManager::load_default();
        
        assert!(theme_manager.switch_theme("Light").is_ok());
        assert_eq!(theme_manager.current_theme().name, "Light");
    }
    
    #[test]
    fn test_view_stack_operations() {
        let mut view_stack = ViewStack::new();
        let test_view = TestView::new();
        
        view_stack.push(Box::new(test_view));
        assert_eq!(view_stack.depth(), 1);
        
        view_stack.pop();
        assert_eq!(view_stack.depth(), 0);
    }
}
```

## ✅ Definition of Done
- [ ] DoraApp framework handles TUI lifecycle and event processing
- [ ] Component system provides reusable UI building blocks
- [ ] View system supports complex view composition and navigation
- [ ] Theme system enables consistent styling and customization
- [ ] Performance targets met for rendering and event handling
- [ ] Event routing works correctly between views and components
- [ ] View stack navigation maintains proper context
- [ ] Memory usage stays within target limits
- [ ] Frame rate maintains 60 FPS under normal conditions
- [ ] Comprehensive unit tests validate all core functionality
- [ ] Integration tests confirm TUI launch and basic operations
- [ ] Manual testing validates user experience and responsiveness

This TUI architecture foundation provides a robust, performant, and extensible framework for all interactive views in the hybrid CLI system, ensuring consistency and quality across all TUI implementations.