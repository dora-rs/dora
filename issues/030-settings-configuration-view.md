# Issue #030: Build Settings and Configuration View

## 📋 Summary
Implement a comprehensive settings and configuration interface that provides intuitive management of all Dora system preferences, user customizations, theme settings, and application configurations. This view serves as the central control panel for personalizing and optimizing the Dora development environment according to user preferences and workflow requirements.

## 🎯 Objectives
- Create intuitive settings interface with categorized configuration options
- Implement real-time preview and validation of configuration changes
- Add import/export functionality for settings portability and team sharing
- Provide search and filtering capabilities for efficient settings navigation
- Enable profile management for different development environments and workflows

**Success Metrics:**
- Settings loading completes within 200ms for all configuration categories
- Configuration changes apply immediately with live preview
- Settings search returns results within 50ms for any query
- User customization adoption rate exceeds 75% among regular users
- Configuration export/import success rate above 95%

## 🛠️ Technical Requirements

### What to Build

#### 1. Settings View Implementation
```rust
// src/tui/views/settings.rs
use super::BaseView;
use crate::tui::{
    components::*,
    theme::Theme,
    AppContext, View, ViewEvent, ViewEventResult, ViewType
};
use async_trait::async_trait;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Style},
    widgets::{Block, Borders, Clear, Tabs},
    Frame,
};
use std::collections::HashMap;
use tokio::time::{Duration, Instant};

pub struct SettingsView {
    base: BaseView,
    settings_context: SettingsContext,
    settings_state: SettingsState,
    config_manager: ConfigurationManager,
    validation_engine: SettingsValidationEngine,
    profile_manager: ProfileManager,
    import_export_manager: ImportExportManager,
    last_update: Instant,
}

#[derive(Debug, Clone)]
pub struct SettingsContext {
    pub current_category: SettingsCategory,
    pub search_query: Option<String>,
    pub active_profile: String,
    pub preview_mode: bool,
}

#[derive(Debug, Clone)]
pub enum SettingsCategory {
    General,
    Appearance,
    Performance,
    Debugging,
    Logging,
    Networking,
    Security,
    Advanced,
    Profiles,
    ImportExport,
}

#[derive(Debug)]
pub struct SettingsState {
    pub categories: HashMap<SettingsCategory, CategorySettings>,
    pub current_settings: SettingsConfiguration,
    pub pending_changes: PendingChanges,
    pub validation_results: ValidationResults,
    pub search_results: SearchResults,
    pub profiles: Vec<SettingsProfile>,
}

impl SettingsView {
    pub fn new() -> Self {
        let mut base = BaseView::new();
        
        let settings_context = SettingsContext {
            current_category: SettingsCategory::General,
            search_query: None,
            active_profile: "default".to_string(),
            preview_mode: true,
        };
        
        // Configure layout for settings
        let layout = LayoutConfig::split_horizontal(vec![
            LayoutSection {
                constraint: Constraint::Length(25),
                component_id: ComponentId("settings_navigation".to_string()),
            },
            LayoutSection {
                constraint: Constraint::Min(50),
                component_id: ComponentId("settings_content".to_string()),
            },
            LayoutSection {
                constraint: Constraint::Length(30),
                component_id: ComponentId("settings_preview".to_string()),
            },
        ]);
        
        base.set_layout(layout);
        
        // Add settings components
        base.add_component(
            ComponentId("settings_navigation".to_string()),
            SettingsNavigationComponent::new(settings_context.clone()),
        );
        
        base.add_component(
            ComponentId("settings_content".to_string()),
            SettingsContentComponent::new(settings_context.clone()),
        );
        
        base.add_component(
            ComponentId("settings_preview".to_string()),
            SettingsPreviewComponent::new(settings_context.clone()),
        );
        
        Self {
            base,
            settings_context,
            settings_state: SettingsState::new(),
            config_manager: ConfigurationManager::new(),
            validation_engine: SettingsValidationEngine::new(),
            profile_manager: ProfileManager::new(),
            import_export_manager: ImportExportManager::new(),
            last_update: Instant::now(),
        }
    }
    
    async fn load_settings(&mut self) -> io::Result<()> {
        // Load current configuration
        self.settings_state.current_settings = self.config_manager.load_current_configuration().await?;
        
        // Load all category settings
        for category in SettingsCategory::all() {
            let category_settings = self.config_manager.load_category_settings(&category).await?;
            self.settings_state.categories.insert(category, category_settings);
        }
        
        // Load available profiles
        self.settings_state.profiles = self.profile_manager.load_profiles().await?;
        
        // Validate current settings
        self.validate_current_settings().await?;
        
        Ok(())
    }
    
    async fn validate_current_settings(&mut self) -> io::Result<()> {
        self.settings_state.validation_results = self.validation_engine
            .validate_configuration(&self.settings_state.current_settings)
            .await?;
        
        Ok(())
    }
    
    async fn apply_setting_change(&mut self, setting_path: &str, new_value: SettingValue) -> io::Result<()> {
        // Validate the change
        let validation_result = self.validation_engine
            .validate_setting_change(setting_path, &new_value)
            .await?;
        
        if !validation_result.is_valid {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("Invalid setting value: {}", validation_result.error_message.unwrap_or_default())
            ));
        }
        
        // Add to pending changes
        self.settings_state.pending_changes.add_change(setting_path.to_string(), new_value.clone());
        
        // Apply immediately if in preview mode
        if self.settings_context.preview_mode {
            self.config_manager.apply_setting_change(setting_path, &new_value).await?;
        }
        
        Ok(())
    }
    
    async fn save_settings(&mut self) -> io::Result<()> {
        // Apply all pending changes
        for (setting_path, value) in &self.settings_state.pending_changes.changes {
            self.config_manager.apply_setting_change(setting_path, value).await?;
        }
        
        // Save configuration to disk
        self.config_manager.save_configuration(&self.settings_state.current_settings).await?;
        
        // Clear pending changes
        self.settings_state.pending_changes.clear();
        
        Ok(())
    }
    
    async fn reset_category_to_defaults(&mut self, category: &SettingsCategory) -> io::Result<()> {
        let default_settings = self.config_manager.get_default_category_settings(category).await?;
        
        for (setting_path, default_value) in default_settings {
            self.apply_setting_change(&setting_path, default_value).await?;
        }
        
        Ok(())
    }
    
    async fn search_settings(&mut self, query: &str) -> io::Result<()> {
        self.settings_state.search_results = self.config_manager
            .search_settings(query)
            .await?;
        
        self.settings_context.search_query = Some(query.to_string());
        
        Ok(())
    }
}

#[async_trait]
impl View for SettingsView {
    async fn initialize(&mut self, context: &AppContext) -> io::Result<()> {
        // Load all settings
        self.load_settings().await?;
        
        // Initialize all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        Ok(())
    }
    
    async fn update(&mut self, context: &mut AppContext) -> io::Result<()> {
        // Check for configuration changes
        if self.config_manager.has_external_changes().await? {
            self.load_settings().await?;
        }
        
        // Update all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        self.last_update = Instant::now();
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Render main settings layout
        self.base.render_components(frame, area, theme, context);
        
        // Render settings overlays
        if self.settings_state.pending_changes.has_changes() {
            self.render_pending_changes_indicator(frame, area, theme);
        }
        
        if !self.settings_state.validation_results.is_all_valid() {
            self.render_validation_errors(frame, area, theme);
        }
    }
    
    async fn handle_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult> {
        if let ViewEvent::Key(key_event) = &event {
            match key_event.code {
                crossterm::event::KeyCode::Char('s') if key_event.modifiers.contains(crossterm::event::KeyModifiers::CONTROL) => {
                    // Save settings
                    self.save_settings().await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('r') => {
                    // Reset current category to defaults
                    self.reset_category_to_defaults(&self.settings_context.current_category).await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('/') => {
                    // Start search
                    return Ok(ViewEventResult::PushView(ViewType::SettingsSearchDialog));
                },
                
                crossterm::event::KeyCode::Char('p') => {
                    // Toggle preview mode
                    self.settings_context.preview_mode = !self.settings_context.preview_mode;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('e') => {
                    // Export settings
                    return Ok(ViewEventResult::PushView(ViewType::SettingsExportDialog {
                        categories: vec![self.settings_context.current_category.clone()],
                    }));
                },
                
                crossterm::event::KeyCode::Char('i') => {
                    // Import settings
                    return Ok(ViewEventResult::PushView(ViewType::SettingsImportDialog));
                },
                
                crossterm::event::KeyCode::Char('n') => {
                    // New profile
                    return Ok(ViewEventResult::PushView(ViewType::ProfileCreationDialog));
                },
                
                crossterm::event::KeyCode::F1 => {
                    // Help for current category
                    return Ok(ViewEventResult::PushView(ViewType::HelpBrowser {
                        topic: Some(format!("settings_{}", self.settings_context.current_category.name())),
                    }));
                },
                
                _ => {}
            }
        }
        
        // Forward event to components
        self.base.handle_component_event(event, context).await
    }
    
    async fn handle_resize(&mut self, width: u16, height: u16) -> io::Result<()> {
        self.base.layout_manager.handle_resize(width, height);
        Ok(())
    }
    
    async fn cleanup(&self, context: &AppContext) -> io::Result<()> {
        // Auto-save if there are pending changes
        if self.settings_state.pending_changes.has_changes() {
            // Prompt user about unsaved changes
            // For now, just save automatically
            // self.save_settings().await?;
        }
        
        Ok(())
    }
    
    fn view_type(&self) -> ViewType {
        ViewType::Settings
    }
    
    fn title(&self) -> String {
        format!("Settings - {}", self.settings_context.current_category.display_name())
    }
    
    fn help_text(&self) -> Vec<HelpItem> {
        vec![
            HelpItem::new("Ctrl+S", "Save settings"),
            HelpItem::new("r", "Reset to defaults"),
            HelpItem::new("/", "Search settings"),
            HelpItem::new("p", "Toggle preview"),
            HelpItem::new("e", "Export settings"),
            HelpItem::new("i", "Import settings"),
            HelpItem::new("n", "New profile"),
            HelpItem::new("F1", "Help"),
            HelpItem::new("Esc", "Back"),
        ]
    }
}
```

#### 2. Settings Navigation Component
```rust
// src/tui/components/settings/navigation.rs
pub struct SettingsNavigationComponent {
    settings_context: SettingsContext,
    focused: bool,
    categories: Vec<CategoryInfo>,
    selected_index: usize,
    search_mode: bool,
}

#[derive(Debug, Clone)]
pub struct CategoryInfo {
    pub category: SettingsCategory,
    pub display_name: String,
    pub icon: String,
    pub setting_count: usize,
    pub has_changes: bool,
    pub has_errors: bool,
}

impl SettingsNavigationComponent {
    pub fn new(settings_context: SettingsContext) -> Self {
        Self {
            settings_context,
            focused: false,
            categories: Self::create_category_list(),
            selected_index: 0,
            search_mode: false,
        }
    }
    
    fn create_category_list() -> Vec<CategoryInfo> {
        vec![
            CategoryInfo {
                category: SettingsCategory::General,
                display_name: "General".to_string(),
                icon: "⚙".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
            CategoryInfo {
                category: SettingsCategory::Appearance,
                display_name: "Appearance".to_string(),
                icon: "🎨".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
            CategoryInfo {
                category: SettingsCategory::Performance,
                display_name: "Performance".to_string(),
                icon: "⚡".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
            CategoryInfo {
                category: SettingsCategory::Debugging,
                display_name: "Debugging".to_string(),
                icon: "🐛".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
            CategoryInfo {
                category: SettingsCategory::Logging,
                display_name: "Logging".to_string(),
                icon: "📝".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
            CategoryInfo {
                category: SettingsCategory::Networking,
                display_name: "Networking".to_string(),
                icon: "🌐".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
            CategoryInfo {
                category: SettingsCategory::Security,
                display_name: "Security".to_string(),
                icon: "🔒".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
            CategoryInfo {
                category: SettingsCategory::Advanced,
                display_name: "Advanced".to_string(),
                icon: "🔧".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
            CategoryInfo {
                category: SettingsCategory::Profiles,
                display_name: "Profiles".to_string(),
                icon: "👤".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
            CategoryInfo {
                category: SettingsCategory::ImportExport,
                display_name: "Import/Export".to_string(),
                icon: "📦".to_string(),
                setting_count: 0,
                has_changes: false,
                has_errors: false,
            },
        ]
    }
    
    fn render_category_item(&self, category_info: &CategoryInfo, selected: bool, theme: &Theme) -> ListItem {
        let mut indicators = String::new();
        
        if category_info.has_changes {
            indicators.push_str(" ●");
        }
        
        if category_info.has_errors {
            indicators.push_str(" ⚠");
        }
        
        let content = format!(
            "{} {} ({}){}", 
            category_info.icon,
            category_info.display_name,
            category_info.setting_count,
            indicators
        );
        
        let mut style = if selected {
            theme.selected_item_style()
        } else {
            theme.normal_item_style()
        };
        
        if category_info.has_errors {
            style = style.fg(Color::Red);
        } else if category_info.has_changes {
            style = style.fg(Color::Yellow);
        }
        
        ListItem::new(content).style(style)
    }
    
    fn render_search_box(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        let search_text = if self.search_mode {
            "Search settings..."
        } else {
            "Press / to search"
        };
        
        let search_paragraph = Paragraph::new(search_text)
            .style(if self.search_mode {
                theme.focused_style()
            } else {
                theme.muted_style()
            })
            .block(Block::default().borders(Borders::ALL));
        
        frame.render_widget(search_paragraph, area);
    }
}

#[async_trait]
impl Component for SettingsNavigationComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Update category information from settings state
        if let Some(settings_state) = context.get_component_state::<SettingsState>("settings") {
            for category_info in &mut self.categories {
                if let Some(category_settings) = settings_state.categories.get(&category_info.category) {
                    category_info.setting_count = category_settings.settings.len();
                    category_info.has_changes = settings_state.pending_changes
                        .has_changes_for_category(&category_info.category);
                    category_info.has_errors = settings_state.validation_results
                        .has_errors_for_category(&category_info.category);
                }
            }
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title("Categories")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        // Split area for search box and categories
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),
                Constraint::Min(0),
            ].as_ref())
            .split(inner_area);
        
        self.render_search_box(frame, chunks[0], theme);
        
        // Render category list
        let items: Vec<ListItem> = self.categories.iter()
            .enumerate()
            .map(|(i, category_info)| {
                self.render_category_item(category_info, i == self.selected_index, theme)
            })
            .collect();
        
        let list = List::new(items)
            .highlight_style(theme.highlight_style())
            .highlight_symbol("▶ ");
        
        frame.render_stateful_widget(
            list,
            chunks[1],
            &mut ratatui::widgets::ListState::default().with_selected(Some(self.selected_index))
        );
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
                        if self.selected_index + 1 < self.categories.len() {
                            self.selected_index += 1;
                        }
                    },
                    
                    crossterm::event::KeyCode::Enter => {
                        if let Some(selected_category) = self.categories.get(self.selected_index) {
                            return Ok(ViewEventResult::Custom(CustomEvent::SwitchSettingsCategory {
                                category: selected_category.category.clone(),
                            }));
                        }
                    },
                    
                    crossterm::event::KeyCode::Char('/') => {
                        self.search_mode = true;
                        return Ok(ViewEventResult::Custom(CustomEvent::StartSettingsSearch));
                    },
                    
                    _ => {}
                }
            },
            _ => {}
        }
        
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::SettingsNavigation
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

#### 3. Settings Content Component
```rust
// src/tui/components/settings/content.rs
pub struct SettingsContentComponent {
    settings_context: SettingsContext,
    focused: bool,
    current_settings: Vec<SettingItem>,
    selected_index: usize,
    edit_mode: bool,
    current_editor: Option<SettingEditor>,
}

#[derive(Debug, Clone)]
pub struct SettingItem {
    pub key: String,
    pub display_name: String,
    pub description: String,
    pub value: SettingValue,
    pub setting_type: SettingType,
    pub validation_rules: ValidationRules,
    pub default_value: SettingValue,
    pub has_pending_change: bool,
    pub validation_error: Option<String>,
}

#[derive(Debug, Clone)]
pub enum SettingType {
    Boolean,
    Integer { min: Option<i64>, max: Option<i64> },
    Float { min: Option<f64>, max: Option<f64> },
    String { max_length: Option<usize> },
    Enum { options: Vec<String> },
    Color,
    Path { file_type: PathType },
    Duration,
    List { item_type: Box<SettingType> },
}

impl SettingsContentComponent {
    pub fn new(settings_context: SettingsContext) -> Self {
        Self {
            settings_context,
            focused: true,
            current_settings: Vec::new(),
            selected_index: 0,
            edit_mode: false,
            current_editor: None,
        }
    }
    
    fn render_setting_item(&self, setting: &SettingItem, selected: bool, theme: &Theme) -> ListItem {
        let value_text = self.format_setting_value(&setting.value, &setting.setting_type);
        let mut content = format!("{}: {}", setting.display_name, value_text);
        
        if setting.has_pending_change {
            content.push_str(" *");
        }
        
        let mut style = if selected {
            theme.selected_item_style()
        } else {
            theme.normal_item_style()
        };
        
        if setting.validation_error.is_some() {
            style = style.fg(Color::Red);
        } else if setting.has_pending_change {
            style = style.fg(Color::Yellow);
        }
        
        ListItem::new(content).style(style)
    }
    
    fn format_setting_value(&self, value: &SettingValue, setting_type: &SettingType) -> String {
        match (value, setting_type) {
            (SettingValue::Boolean(b), SettingType::Boolean) => {
                if *b { "Enabled".to_string() } else { "Disabled".to_string() }
            },
            (SettingValue::Integer(i), SettingType::Integer { .. }) => i.to_string(),
            (SettingValue::Float(f), SettingType::Float { .. }) => format!("{:.2}", f),
            (SettingValue::String(s), SettingType::String { .. }) => {
                if s.len() > 30 {
                    format!("{}...", &s[..27])
                } else {
                    s.clone()
                }
            },
            (SettingValue::String(s), SettingType::Enum { .. }) => s.clone(),
            (SettingValue::Color(c), SettingType::Color) => format!("#{:06x}", c),
            (SettingValue::Path(p), SettingType::Path { .. }) => {
                p.file_name()
                    .and_then(|name| name.to_str())
                    .unwrap_or("(invalid path)")
                    .to_string()
            },
            (SettingValue::Duration(d), SettingType::Duration) => {
                format!("{}ms", d.as_millis())
            },
            (SettingValue::List(items), SettingType::List { .. }) => {
                format!("[{} items]", items.len())
            },
            _ => "Invalid".to_string(),
        }
    }
    
    fn render_setting_description(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(selected_setting) = self.current_settings.get(self.selected_index) {
            let description_text = format!(
                "{}\n\nDefault: {}\nType: {}",
                selected_setting.description,
                self.format_setting_value(&selected_setting.default_value, &selected_setting.setting_type),
                selected_setting.setting_type.display_name()
            );
            
            let paragraph = Paragraph::new(description_text)
                .style(theme.info_style())
                .wrap(ratatui::widgets::Wrap { trim: true })
                .block(Block::default().title("Description").borders(Borders::ALL));
            
            frame.render_widget(paragraph, area);
        }
    }
    
    fn render_setting_editor(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(editor) = &self.current_editor {
            editor.render(frame, area, theme);
        }
    }
}

#[async_trait]
impl Component for SettingsContentComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Update settings list based on current category
        if let Some(settings_state) = context.get_component_state::<SettingsState>("settings") {
            if let Some(category_settings) = settings_state.categories.get(&self.settings_context.current_category) {
                self.current_settings = category_settings.settings.clone();
                
                // Update pending changes and validation errors
                for setting in &mut self.current_settings {
                    setting.has_pending_change = settings_state.pending_changes
                        .has_change_for_setting(&setting.key);
                    setting.validation_error = settings_state.validation_results
                        .get_error_for_setting(&setting.key);
                }
            }
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title(format!("{} Settings", self.settings_context.current_category.display_name()))
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        if self.edit_mode && self.current_editor.is_some() {
            // Render editor overlay
            self.render_setting_editor(frame, inner_area, theme);
        } else {
            // Split area for settings list and description
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Min(10),
                    Constraint::Length(8),
                ].as_ref())
                .split(inner_area);
            
            // Render settings list
            let items: Vec<ListItem> = self.current_settings.iter()
                .enumerate()
                .map(|(i, setting)| {
                    self.render_setting_item(setting, i == self.selected_index, theme)
                })
                .collect();
            
            let list = List::new(items)
                .highlight_style(theme.highlight_style())
                .highlight_symbol("▶ ");
            
            frame.render_stateful_widget(
                list,
                chunks[0],
                &mut ratatui::widgets::ListState::default().with_selected(Some(self.selected_index))
            );
            
            // Render description
            self.render_setting_description(frame, chunks[1], theme);
        }
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }
        
        if self.edit_mode {
            // Handle editor events
            if let Some(editor) = &mut self.current_editor {
                return editor.handle_event(event, context).await;
            }
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
                        if self.selected_index + 1 < self.current_settings.len() {
                            self.selected_index += 1;
                        }
                    },
                    
                    crossterm::event::KeyCode::Enter => {
                        // Start editing selected setting
                        if let Some(selected_setting) = self.current_settings.get(self.selected_index) {
                            self.current_editor = Some(SettingEditor::new(selected_setting.clone()));
                            self.edit_mode = true;
                        }
                    },
                    
                    crossterm::event::KeyCode::Char(' ') => {
                        // Quick toggle for boolean settings
                        if let Some(selected_setting) = self.current_settings.get(self.selected_index) {
                            if matches!(selected_setting.setting_type, SettingType::Boolean) {
                                if let SettingValue::Boolean(current_value) = selected_setting.value {
                                    return Ok(ViewEventResult::Custom(CustomEvent::ChangeSettingValue {
                                        setting_key: selected_setting.key.clone(),
                                        new_value: SettingValue::Boolean(!current_value),
                                    }));
                                }
                            }
                        }
                    },
                    
                    crossterm::event::KeyCode::Delete => {
                        // Reset to default value
                        if let Some(selected_setting) = self.current_settings.get(self.selected_index) {
                            return Ok(ViewEventResult::Custom(CustomEvent::ResetSettingToDefault {
                                setting_key: selected_setting.key.clone(),
                            }));
                        }
                    },
                    
                    _ => {}
                }
            },
            _ => {}
        }
        
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::SettingsContent
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

#### 4. Configuration Management System
```rust
// src/tui/components/settings/config_manager.rs
#[derive(Debug)]
pub struct ConfigurationManager {
    config_loader: ConfigLoader,
    config_saver: ConfigSaver,
    schema_validator: SchemaValidator,
    default_configs: HashMap<SettingsCategory, CategorySettings>,
    config_cache: LruCache<String, SettingsConfiguration>,
}

impl ConfigurationManager {
    pub fn new() -> Self {
        Self {
            config_loader: ConfigLoader::new(),
            config_saver: ConfigSaver::new(),
            schema_validator: SchemaValidator::new(),
            default_configs: Self::load_default_configurations(),
            config_cache: LruCache::new(100),
        }
    }
    
    pub async fn load_current_configuration(&mut self) -> io::Result<SettingsConfiguration> {
        // Check cache first
        if let Some(cached_config) = self.config_cache.get("current") {
            return Ok(cached_config.clone());
        }
        
        // Load from file system
        let config = self.config_loader.load_user_configuration().await?;
        
        // Merge with defaults for missing values
        let merged_config = self.merge_with_defaults(config).await?;
        
        // Validate configuration
        self.schema_validator.validate_configuration(&merged_config)?;
        
        // Cache result
        self.config_cache.put("current".to_string(), merged_config.clone());
        
        Ok(merged_config)
    }
    
    pub async fn load_category_settings(&self, category: &SettingsCategory) -> io::Result<CategorySettings> {
        let user_config = self.config_loader.load_user_configuration().await?;
        let default_config = self.default_configs.get(category)
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "Category not found"))?;
        
        // Merge user settings with defaults
        let mut category_settings = default_config.clone();
        
        if let Some(user_category) = user_config.categories.get(category) {
            for (key, value) in &user_category.settings {
                if let Some(setting_item) = category_settings.settings.iter_mut().find(|s| &s.key == key) {
                    setting_item.value = value.clone();
                }
            }
        }
        
        Ok(category_settings)
    }
    
    pub async fn apply_setting_change(&mut self, setting_path: &str, new_value: &SettingValue) -> io::Result<()> {
        // Parse setting path (e.g., "appearance.theme" or "general.auto_save")
        let parts: Vec<&str> = setting_path.split('.').collect();
        if parts.len() != 2 {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "Invalid setting path"));
        }
        
        let category_name = parts[0];
        let setting_key = parts[1];
        
        // Load current configuration
        let mut config = self.load_current_configuration().await?;
        
        // Find and update the setting
        let category = SettingsCategory::from_name(category_name)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "Unknown category"))?;
        
        config.update_setting(&category, setting_key, new_value.clone())?;
        
        // Validate the change
        self.schema_validator.validate_configuration(&config)?;
        
        // Update cache
        self.config_cache.put("current".to_string(), config);
        
        Ok(())
    }
    
    pub async fn save_configuration(&mut self, config: &SettingsConfiguration) -> io::Result<()> {
        // Validate before saving
        self.schema_validator.validate_configuration(config)?;
        
        // Save to file system
        self.config_saver.save_user_configuration(config).await?;
        
        // Update cache
        self.config_cache.put("current".to_string(), config.clone());
        
        Ok(())
    }
    
    pub async fn get_default_category_settings(&self, category: &SettingsCategory) -> io::Result<HashMap<String, SettingValue>> {
        let default_settings = self.default_configs.get(category)
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "Category not found"))?;
        
        let mut result = HashMap::new();
        for setting in &default_settings.settings {
            result.insert(
                format!("{}.{}", category.name(), setting.key),
                setting.default_value.clone()
            );
        }
        
        Ok(result)
    }
    
    pub async fn search_settings(&self, query: &str) -> io::Result<SearchResults> {
        let mut results = SearchResults::new();
        let query_lower = query.to_lowercase();
        
        // Search through all categories
        for (category, category_settings) in &self.default_configs {
            for setting in &category_settings.settings {
                // Search in display name
                if setting.display_name.to_lowercase().contains(&query_lower) {
                    results.add_result(SearchResult {
                        category: category.clone(),
                        setting_key: setting.key.clone(),
                        display_name: setting.display_name.clone(),
                        match_type: MatchType::DisplayName,
                        relevance_score: self.calculate_relevance(&query_lower, &setting.display_name.to_lowercase()),
                    });
                }
                
                // Search in description
                if setting.description.to_lowercase().contains(&query_lower) {
                    results.add_result(SearchResult {
                        category: category.clone(),
                        setting_key: setting.key.clone(),
                        display_name: setting.display_name.clone(),
                        match_type: MatchType::Description,
                        relevance_score: self.calculate_relevance(&query_lower, &setting.description.to_lowercase()),
                    });
                }
                
                // Search in key
                if setting.key.to_lowercase().contains(&query_lower) {
                    results.add_result(SearchResult {
                        category: category.clone(),
                        setting_key: setting.key.clone(),
                        display_name: setting.display_name.clone(),
                        match_type: MatchType::Key,
                        relevance_score: self.calculate_relevance(&query_lower, &setting.key.to_lowercase()),
                    });
                }
            }
        }
        
        // Sort by relevance
        results.sort_by_relevance();
        
        Ok(results)
    }
    
    fn calculate_relevance(&self, query: &str, text: &str) -> f64 {
        if text == query {
            1.0
        } else if text.starts_with(query) {
            0.8
        } else if text.contains(query) {
            0.6
        } else {
            0.3
        }
    }
    
    fn load_default_configurations() -> HashMap<SettingsCategory, CategorySettings> {
        let mut defaults = HashMap::new();
        
        // General settings
        defaults.insert(SettingsCategory::General, CategorySettings {
            settings: vec![
                SettingItem {
                    key: "auto_save".to_string(),
                    display_name: "Auto Save".to_string(),
                    description: "Automatically save configuration changes".to_string(),
                    value: SettingValue::Boolean(true),
                    setting_type: SettingType::Boolean,
                    validation_rules: ValidationRules::default(),
                    default_value: SettingValue::Boolean(true),
                    has_pending_change: false,
                    validation_error: None,
                },
                SettingItem {
                    key: "check_updates".to_string(),
                    display_name: "Check for Updates".to_string(),
                    description: "Automatically check for Dora updates on startup".to_string(),
                    value: SettingValue::Boolean(true),
                    setting_type: SettingType::Boolean,
                    validation_rules: ValidationRules::default(),
                    default_value: SettingValue::Boolean(true),
                    has_pending_change: false,
                    validation_error: None,
                },
                SettingItem {
                    key: "default_dataflow_path".to_string(),
                    display_name: "Default Dataflow Path".to_string(),
                    description: "Default directory for creating new dataflows".to_string(),
                    value: SettingValue::Path(PathBuf::from("./dataflows")),
                    setting_type: SettingType::Path { file_type: PathType::Directory },
                    validation_rules: ValidationRules::default(),
                    default_value: SettingValue::Path(PathBuf::from("./dataflows")),
                    has_pending_change: false,
                    validation_error: None,
                },
            ],
        });
        
        // Appearance settings
        defaults.insert(SettingsCategory::Appearance, CategorySettings {
            settings: vec![
                SettingItem {
                    key: "theme".to_string(),
                    display_name: "Theme".to_string(),
                    description: "Visual theme for the user interface".to_string(),
                    value: SettingValue::String("dark".to_string()),
                    setting_type: SettingType::Enum { 
                        options: vec!["dark".to_string(), "light".to_string(), "auto".to_string()]
                    },
                    validation_rules: ValidationRules::default(),
                    default_value: SettingValue::String("dark".to_string()),
                    has_pending_change: false,
                    validation_error: None,
                },
                SettingItem {
                    key: "font_size".to_string(),
                    display_name: "Font Size".to_string(),
                    description: "Size of text in the interface".to_string(),
                    value: SettingValue::Integer(14),
                    setting_type: SettingType::Integer { min: Some(8), max: Some(24) },
                    validation_rules: ValidationRules::default(),
                    default_value: SettingValue::Integer(14),
                    has_pending_change: false,
                    validation_error: None,
                },
                SettingItem {
                    key: "accent_color".to_string(),
                    display_name: "Accent Color".to_string(),
                    description: "Primary accent color for highlights and selection".to_string(),
                    value: SettingValue::Color(0x007ACC),
                    setting_type: SettingType::Color,
                    validation_rules: ValidationRules::default(),
                    default_value: SettingValue::Color(0x007ACC),
                    has_pending_change: false,
                    validation_error: None,
                },
            ],
        });
        
        // Performance settings
        defaults.insert(SettingsCategory::Performance, CategorySettings {
            settings: vec![
                SettingItem {
                    key: "refresh_rate".to_string(),
                    display_name: "Refresh Rate".to_string(),
                    description: "How often to update the interface (in milliseconds)".to_string(),
                    value: SettingValue::Duration(Duration::from_millis(100)),
                    setting_type: SettingType::Duration,
                    validation_rules: ValidationRules::default(),
                    default_value: SettingValue::Duration(Duration::from_millis(100)),
                    has_pending_change: false,
                    validation_error: None,
                },
                SettingItem {
                    key: "buffer_size".to_string(),
                    display_name: "Buffer Size".to_string(),
                    description: "Maximum number of log entries to keep in memory".to_string(),
                    value: SettingValue::Integer(10000),
                    setting_type: SettingType::Integer { min: Some(1000), max: Some(100000) },
                    validation_rules: ValidationRules::default(),
                    default_value: SettingValue::Integer(10000),
                    has_pending_change: false,
                    validation_error: None,
                },
            ],
        });
        
        // Add other categories...
        
        defaults
    }
}
```

### Why This Approach

**Comprehensive Configuration Management:**
- Centralized settings interface with intuitive categorization
- Real-time validation and preview of configuration changes
- Profile management for different development environments

**User-Friendly Interface:**
- Search and filtering capabilities for efficient settings navigation
- Live preview of changes with immediate feedback
- Import/export functionality for team collaboration

**Robust Architecture:**
- Schema validation ensures configuration integrity
- Efficient caching and lazy loading for performance
- Extensible framework for adding new setting types

### How to Implement

#### Step 1: Settings View Core (5 hours)
1. **Implement SettingsView** with category-based navigation
2. **Add configuration management** and validation system
3. **Create settings state** management and persistence
4. **Add preview mode** and real-time change application

#### Step 2: Navigation and Content Components (4 hours)
1. **Implement SettingsNavigationComponent** with category list
2. **Add SettingsContentComponent** with setting item display
3. **Create setting editors** for different data types
4. **Add search functionality** and result highlighting

#### Step 3: Configuration Management (4 hours)
1. **Implement ConfigurationManager** with loading and saving
2. **Add schema validation** and error handling
3. **Create default configurations** for all categories
4. **Add configuration merging** and migration support

#### Step 4: Advanced Features (3 hours)
1. **Implement profile management** with multiple configurations
2. **Add import/export functionality** with format support
3. **Create settings search** with relevance scoring
4. **Add configuration backup** and restore capabilities

#### Step 5: Integration and Testing (2 hours)
1. **Add comprehensive unit tests** for all settings components
2. **Test configuration validation** and error handling
3. **Validate settings persistence** and loading
4. **Test user interface** responsiveness and usability

## 🔗 Dependencies
**Depends On:**
- Issue #023 (TUI Architecture Foundation) - Base view and component system
- Issue #024 (Dashboard Overview) - Theme integration
- Phase 1 configuration system for backend integration

**Enables:**
- Comprehensive user customization and personalization
- Team configuration sharing and standardization

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_settings_view_initialization() {
        let settings_view = SettingsView::new();
        
        assert_eq!(settings_view.settings_context.current_category, SettingsCategory::General);
        assert!(!settings_view.settings_state.pending_changes.has_changes());
    }
    
    #[test]
    fn test_configuration_validation() {
        let validator = SettingsValidationEngine::new();
        let invalid_config = SettingsConfiguration::invalid_test_config();
        
        let result = validator.validate_configuration(&invalid_config).await.unwrap();
        
        assert!(!result.is_all_valid());
        assert!(!result.errors.is_empty());
    }
    
    #[test]
    fn test_settings_search() {
        let config_manager = ConfigurationManager::new();
        
        let results = config_manager.search_settings("theme").await.unwrap();
        
        assert!(!results.is_empty());
        assert!(results.results.iter().any(|r| r.setting_key == "theme"));
    }
}
```

## ✅ Definition of Done
- [ ] SettingsView provides comprehensive configuration management interface
- [ ] Category navigation enables efficient settings organization and access
- [ ] Setting editors support all configuration data types with validation
- [ ] Real-time preview shows configuration changes immediately
- [ ] Search functionality finds relevant settings quickly and accurately
- [ ] Profile management enables multiple configuration environments
- [ ] Import/export functionality supports configuration sharing
- [ ] Validation system prevents invalid configuration states
- [ ] Performance targets met for all settings operations
- [ ] Configuration persistence works reliably across sessions
- [ ] Comprehensive unit tests validate all settings functionality
- [ ] Integration tests confirm configuration management effectiveness
- [ ] Manual testing validates user experience and workflow efficiency

This comprehensive settings and configuration view provides users with powerful customization capabilities while maintaining system integrity through robust validation and management features.