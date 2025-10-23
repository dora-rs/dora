# Issue #031: Build Interactive Help Browser View

## 📋 Summary
Implement a comprehensive interactive help browser that provides contextual documentation, tutorials, API references, and troubleshooting guides with intelligent search, progressive learning paths, and integrated examples. This view serves as the primary knowledge base and learning center for developers using the Dora system.

## 🎯 Objectives
- Create comprehensive help system with contextual documentation and tutorials
- Implement intelligent search with semantic understanding and relevance scoring
- Add progressive learning paths and interactive tutorials with step validation
- Provide API reference browser with live examples and code generation
- Enable offline documentation access and custom documentation integration

**Success Metrics:**
- Help content loading completes within 150ms for any topic
- Search results return within 100ms with 95%+ relevance accuracy
- Tutorial completion rate exceeds 80% for interactive learning paths
- User self-service problem resolution improves by 60%
- Documentation coverage reaches 95% of all system features

## 🛠️ Technical Requirements

### What to Build

#### 1. Help Browser View Implementation
```rust
// src/tui/views/help_browser.rs
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

pub struct HelpBrowserView {
    base: BaseView,
    help_context: HelpBrowserContext,
    help_state: HelpBrowserState,
    content_manager: HelpContentManager,
    search_engine: HelpSearchEngine,
    tutorial_system: TutorialSystem,
    bookmark_manager: BookmarkManager,
    last_update: Instant,
}

#[derive(Debug, Clone)]
pub struct HelpBrowserContext {
    pub current_topic: Option<String>,
    pub browse_mode: BrowseMode,
    pub search_query: Option<String>,
    pub active_tutorial: Option<String>,
    pub user_context: UserContext,
}

#[derive(Debug, Clone)]
pub enum BrowseMode {
    Documentation,
    Tutorial,
    ApiReference,
    Troubleshooting,
    Examples,
    Search,
}

#[derive(Debug)]
pub struct HelpBrowserState {
    pub content_tree: ContentTree,
    pub current_content: HelpContent,
    pub search_results: SearchResults,
    pub navigation_history: NavigationHistory,
    pub bookmarks: Vec<Bookmark>,
    pub tutorial_progress: TutorialProgress,
}

impl HelpBrowserView {
    pub fn new(initial_topic: Option<String>) -> Self {
        let mut base = BaseView::new();
        
        let help_context = HelpBrowserContext {
            current_topic: initial_topic.clone(),
            browse_mode: BrowseMode::Documentation,
            search_query: None,
            active_tutorial: None,
            user_context: UserContext::current(),
        };
        
        // Configure layout for help browser
        let layout = LayoutConfig::split_horizontal(vec![
            LayoutSection {
                constraint: Constraint::Length(30),
                component_id: ComponentId("help_navigation".to_string()),
            },
            LayoutSection {
                constraint: Constraint::Min(60),
                component_id: ComponentId("help_content".to_string()),
            },
            LayoutSection {
                constraint: Constraint::Length(25),
                component_id: ComponentId("help_sidebar".to_string()),
            },
        ]);
        
        base.set_layout(layout);
        
        // Add help browser components
        base.add_component(
            ComponentId("help_navigation".to_string()),
            HelpNavigationComponent::new(help_context.clone()),
        );
        
        base.add_component(
            ComponentId("help_content".to_string()),
            HelpContentComponent::new(help_context.clone()),
        );
        
        base.add_component(
            ComponentId("help_sidebar".to_string()),
            HelpSidebarComponent::new(help_context.clone()),
        );
        
        Self {
            base,
            help_context,
            help_state: HelpBrowserState::new(),
            content_manager: HelpContentManager::new(),
            search_engine: HelpSearchEngine::new(),
            tutorial_system: TutorialSystem::new(),
            bookmark_manager: BookmarkManager::new(),
            last_update: Instant::now(),
        }
    }
    
    async fn load_help_content(&mut self) -> io::Result<()> {
        // Load content tree
        self.help_state.content_tree = self.content_manager.load_content_tree().await?;
        
        // Load initial content
        if let Some(topic) = &self.help_context.current_topic {
            self.help_state.current_content = self.content_manager.load_topic_content(topic).await?;
        } else {
            self.help_state.current_content = self.content_manager.load_welcome_content().await?;
        }
        
        // Load bookmarks
        self.help_state.bookmarks = self.bookmark_manager.load_bookmarks().await?;
        
        // Load tutorial progress
        self.help_state.tutorial_progress = self.tutorial_system.load_progress().await?;
        
        Ok(())
    }
    
    async fn navigate_to_topic(&mut self, topic: &str) -> io::Result<()> {
        // Add current topic to history
        if let Some(current_topic) = &self.help_context.current_topic {
            self.help_state.navigation_history.add_entry(current_topic.clone());
        }
        
        // Load new content
        self.help_state.current_content = self.content_manager.load_topic_content(topic).await?;
        self.help_context.current_topic = Some(topic.to_string());
        
        // Track navigation for analytics
        self.content_manager.track_navigation(topic, &self.help_context.user_context).await?;
        
        Ok(())
    }
    
    async fn search_help_content(&mut self, query: &str) -> io::Result<()> {
        self.help_state.search_results = self.search_engine.search(
            query,
            &SearchOptions {
                include_content: true,
                include_examples: true,
                include_api_docs: true,
                semantic_search: true,
                user_context: Some(self.help_context.user_context.clone()),
            }
        ).await?;
        
        self.help_context.search_query = Some(query.to_string());
        self.help_context.browse_mode = BrowseMode::Search;
        
        Ok(())
    }
    
    async fn start_tutorial(&mut self, tutorial_id: &str) -> io::Result<()> {
        let tutorial = self.tutorial_system.load_tutorial(tutorial_id).await?;
        
        self.help_context.active_tutorial = Some(tutorial_id.to_string());
        self.help_context.browse_mode = BrowseMode::Tutorial;
        
        // Load tutorial content
        self.help_state.current_content = HelpContent::Tutorial(tutorial);
        
        Ok(())
    }
    
    async fn add_bookmark(&mut self, topic: &str, title: &str) -> io::Result<()> {
        let bookmark = Bookmark {
            id: uuid::Uuid::new_v4().to_string(),
            topic: topic.to_string(),
            title: title.to_string(),
            created_at: chrono::Utc::now(),
            tags: Vec::new(),
        };
        
        self.help_state.bookmarks.push(bookmark.clone());
        self.bookmark_manager.save_bookmark(&bookmark).await?;
        
        Ok(())
    }
}

#[async_trait]
impl View for HelpBrowserView {
    async fn initialize(&mut self, context: &AppContext) -> io::Result<()> {
        // Load help content
        self.load_help_content().await?;
        
        // Initialize search engine
        self.search_engine.initialize(&self.help_state.content_tree).await?;
        
        // Initialize all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        Ok(())
    }
    
    async fn update(&mut self, context: &mut AppContext) -> io::Result<()> {
        // Check for content updates
        if self.content_manager.has_updates().await? {
            self.load_help_content().await?;
        }
        
        // Update tutorial progress if active
        if let Some(tutorial_id) = &self.help_context.active_tutorial {
            self.tutorial_system.update_progress(tutorial_id).await?;
        }
        
        // Update all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        self.last_update = Instant::now();
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Render main help browser layout
        self.base.render_components(frame, area, theme, context);
        
        // Render help browser overlays
        if self.help_context.browse_mode == BrowseMode::Search && self.help_context.search_query.is_some() {
            self.render_search_overlay(frame, area, theme);
        }
        
        if self.help_context.active_tutorial.is_some() {
            self.render_tutorial_progress_overlay(frame, area, theme);
        }
    }
    
    async fn handle_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult> {
        if let ViewEvent::Key(key_event) = &event {
            match key_event.code {
                crossterm::event::KeyCode::Char('/') => {
                    // Start search
                    return Ok(ViewEventResult::PushView(ViewType::HelpSearchDialog {
                        current_query: self.help_context.search_query.clone(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('b') => {
                    // Add bookmark
                    if let Some(topic) = &self.help_context.current_topic {
                        let title = self.help_state.current_content.title().unwrap_or(topic);
                        self.add_bookmark(topic, title).await?;
                    }
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('B') => {
                    // Show bookmarks
                    return Ok(ViewEventResult::PushView(ViewType::BookmarkBrowser {
                        bookmarks: self.help_state.bookmarks.clone(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('h') => {
                    // Go to home/welcome
                    self.navigate_to_topic("welcome").await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('t') => {
                    // Browse tutorials
                    self.help_context.browse_mode = BrowseMode::Tutorial;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('a') => {
                    // Browse API reference
                    self.help_context.browse_mode = BrowseMode::ApiReference;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('e') => {
                    // Browse examples
                    self.help_context.browse_mode = BrowseMode::Examples;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('?') => {
                    // Browse troubleshooting
                    self.help_context.browse_mode = BrowseMode::Troubleshooting;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Left | crossterm::event::KeyCode::Char('[') => {
                    // Navigate back
                    if let Some(previous_topic) = self.help_state.navigation_history.go_back() {
                        self.navigate_to_topic(&previous_topic).await?;
                    }
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Right | crossterm::event::KeyCode::Char(']') => {
                    // Navigate forward
                    if let Some(next_topic) = self.help_state.navigation_history.go_forward() {
                        self.navigate_to_topic(&next_topic).await?;
                    }
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::F1 => {
                    // Context-sensitive help
                    if let Some(context_topic) = context.get_current_context_help_topic() {
                        self.navigate_to_topic(&context_topic).await?;
                    }
                    return Ok(ViewEventResult::None);
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
        // Save navigation history
        self.content_manager.save_navigation_history(&self.help_state.navigation_history).await?;
        
        // Save tutorial progress
        if let Some(tutorial_id) = &self.help_context.active_tutorial {
            self.tutorial_system.save_progress(tutorial_id, &self.help_state.tutorial_progress).await?;
        }
        
        Ok(())
    }
    
    fn view_type(&self) -> ViewType {
        ViewType::HelpBrowser { 
            topic: self.help_context.current_topic.clone() 
        }
    }
    
    fn title(&self) -> String {
        match &self.help_context.browse_mode {
            BrowseMode::Documentation => "Help - Documentation".to_string(),
            BrowseMode::Tutorial => "Help - Tutorials".to_string(),
            BrowseMode::ApiReference => "Help - API Reference".to_string(),
            BrowseMode::Troubleshooting => "Help - Troubleshooting".to_string(),
            BrowseMode::Examples => "Help - Examples".to_string(),
            BrowseMode::Search => format!("Help - Search: {}", 
                self.help_context.search_query.as_deref().unwrap_or("")),
        }
    }
    
    fn help_text(&self) -> Vec<HelpItem> {
        vec![
            HelpItem::new("/", "Search help"),
            HelpItem::new("b", "Bookmark"),
            HelpItem::new("B", "Show bookmarks"),
            HelpItem::new("h", "Home"),
            HelpItem::new("t", "Tutorials"),
            HelpItem::new("a", "API reference"),
            HelpItem::new("e", "Examples"),
            HelpItem::new("?", "Troubleshooting"),
            HelpItem::new("←/→", "Navigate"),
            HelpItem::new("F1", "Context help"),
            HelpItem::new("Esc", "Back"),
        ]
    }
}
```

#### 2. Help Content Component
```rust
// src/tui/components/help/content.rs
pub struct HelpContentComponent {
    help_context: HelpBrowserContext,
    focused: bool,
    scroll_state: ScrollState,
    renderer: ContentRenderer,
    current_content: Option<HelpContent>,
}

impl HelpContentComponent {
    pub fn new(help_context: HelpBrowserContext) -> Self {
        Self {
            help_context,
            focused: true,
            scroll_state: ScrollState::new(),
            renderer: ContentRenderer::new(),
            current_content: None,
        }
    }
    
    fn render_documentation_content(&self, frame: &mut Frame, area: Rect, theme: &Theme, content: &DocumentationContent) {
        let sections = self.renderer.render_markdown(&content.markdown, theme);
        
        let items: Vec<ListItem> = sections.iter()
            .flat_map(|section| section.render_as_list_items(theme))
            .collect();
        
        let list = List::new(items)
            .block(Block::default().title(&content.title).borders(Borders::ALL));
        
        frame.render_stateful_widget(list, area, &mut self.scroll_state.to_list_state());
    }
    
    fn render_tutorial_content(&self, frame: &mut Frame, area: Rect, theme: &Theme, tutorial: &TutorialContent) {
        // Split area for tutorial content and progress
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),
                Constraint::Min(10),
                Constraint::Length(5),
            ].as_ref())
            .split(area);
        
        // Render tutorial header
        let header_text = format!(
            "{} - Step {} of {}",
            tutorial.title,
            tutorial.current_step + 1,
            tutorial.steps.len()
        );
        
        let header = Paragraph::new(header_text)
            .style(theme.info_style())
            .block(Block::default().borders(Borders::ALL));
        
        frame.render_widget(header, chunks[0]);
        
        // Render current step content
        if let Some(current_step) = tutorial.steps.get(tutorial.current_step) {
            let step_content = self.renderer.render_tutorial_step(current_step, theme);
            
            let content_widget = Paragraph::new(step_content)
                .style(theme.normal_style())
                .wrap(ratatui::widgets::Wrap { trim: true })
                .scroll((self.scroll_state.offset() as u16, 0));
            
            frame.render_widget(content_widget, chunks[1]);
        }
        
        // Render progress and controls
        let progress_text = format!(
            "Progress: {} | Next: Enter | Previous: Backspace | Exit: Esc",
            self.format_progress_bar(tutorial.current_step, tutorial.steps.len())
        );
        
        let progress = Paragraph::new(progress_text)
            .style(theme.muted_style())
            .block(Block::default().borders(Borders::ALL));
        
        frame.render_widget(progress, chunks[2]);
    }
    
    fn render_api_reference_content(&self, frame: &mut Frame, area: Rect, theme: &Theme, api_content: &ApiReferenceContent) {
        // Split area for API signature and description
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(5),
                Constraint::Min(10),
                Constraint::Length(8),
            ].as_ref())
            .split(area);
        
        // Render API signature
        let signature_text = format!(
            "{}::{}\n{}",
            api_content.module,
            api_content.name,
            api_content.signature
        );
        
        let signature = Paragraph::new(signature_text)
            .style(theme.info_style())
            .block(Block::default().title("Signature").borders(Borders::ALL));
        
        frame.render_widget(signature, chunks[0]);
        
        // Render description
        let description_content = self.renderer.render_markdown(&api_content.description, theme);
        let description_text = description_content.iter()
            .map(|section| section.text.clone())
            .collect::<Vec<_>>()
            .join("\n");
        
        let description = Paragraph::new(description_text)
            .style(theme.normal_style())
            .wrap(ratatui::widgets::Wrap { trim: true })
            .scroll((self.scroll_state.offset() as u16, 0))
            .block(Block::default().title("Description").borders(Borders::ALL));
        
        frame.render_widget(description, chunks[1]);
        
        // Render examples
        if let Some(example) = &api_content.example {
            let example_widget = Paragraph::new(example.code.clone())
                .style(theme.code_style())
                .block(Block::default().title("Example").borders(Borders::ALL));
            
            frame.render_widget(example_widget, chunks[2]);
        }
    }
    
    fn render_search_results(&self, frame: &mut Frame, area: Rect, theme: &Theme, search_results: &SearchResults) {
        let items: Vec<ListItem> = search_results.results.iter()
            .map(|result| {
                let display_text = format!(
                    "{} - {}",
                    result.title,
                    result.snippet.as_deref().unwrap_or("")
                );
                
                let style = match result.result_type {
                    SearchResultType::Documentation => theme.normal_style(),
                    SearchResultType::Tutorial => theme.info_style(),
                    SearchResultType::ApiReference => theme.success_style(),
                    SearchResultType::Example => theme.warning_style(),
                };
                
                ListItem::new(display_text).style(style)
            })
            .collect();
        
        let list = List::new(items)
            .block(Block::default().title("Search Results").borders(Borders::ALL))
            .highlight_style(theme.highlight_style())
            .highlight_symbol("▶ ");
        
        frame.render_stateful_widget(list, area, &mut self.scroll_state.to_list_state());
    }
    
    fn format_progress_bar(&self, current: usize, total: usize) -> String {
        let bar_width = 20;
        let filled = (current * bar_width) / total;
        let empty = bar_width - filled;
        
        format!("[{}{}] {}/{}", 
                "█".repeat(filled), 
                "░".repeat(empty), 
                current + 1, 
                total)
    }
}

#[async_trait]
impl Component for HelpContentComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Update current content from help browser state
        if let Some(help_state) = context.get_component_state::<HelpBrowserState>("help_browser") {
            self.current_content = Some(help_state.current_content.clone());
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        if let Some(content) = &self.current_content {
            match content {
                HelpContent::Documentation(doc_content) => {
                    self.render_documentation_content(frame, inner_area, theme, doc_content);
                },
                
                HelpContent::Tutorial(tutorial_content) => {
                    self.render_tutorial_content(frame, inner_area, theme, tutorial_content);
                },
                
                HelpContent::ApiReference(api_content) => {
                    self.render_api_reference_content(frame, inner_area, theme, api_content);
                },
                
                HelpContent::SearchResults(search_results) => {
                    self.render_search_results(frame, inner_area, theme, search_results);
                },
                
                HelpContent::Welcome(welcome_content) => {
                    self.render_documentation_content(frame, inner_area, theme, welcome_content);
                },
            }
        }
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }
        
        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Up => {
                        self.scroll_state.scroll_up();
                    },
                    
                    crossterm::event::KeyCode::Down => {
                        self.scroll_state.scroll_down();
                    },
                    
                    crossterm::event::KeyCode::PageUp => {
                        self.scroll_state.page_up();
                    },
                    
                    crossterm::event::KeyCode::PageDown => {
                        self.scroll_state.page_down();
                    },
                    
                    crossterm::event::KeyCode::Home => {
                        self.scroll_state.scroll_to_top();
                    },
                    
                    crossterm::event::KeyCode::End => {
                        self.scroll_state.scroll_to_bottom();
                    },
                    
                    crossterm::event::KeyCode::Enter => {
                        // Handle content-specific actions
                        if let Some(content) = &self.current_content {
                            match content {
                                HelpContent::Tutorial(tutorial) => {
                                    return Ok(ViewEventResult::Custom(CustomEvent::NextTutorialStep));
                                },
                                
                                HelpContent::SearchResults(_) => {
                                    return Ok(ViewEventResult::Custom(CustomEvent::SelectSearchResult {
                                        index: self.scroll_state.selected_index(),
                                    }));
                                },
                                
                                _ => {}
                            }
                        }
                    },
                    
                    crossterm::event::KeyCode::Backspace => {
                        if let Some(HelpContent::Tutorial(_)) = &self.current_content {
                            return Ok(ViewEventResult::Custom(CustomEvent::PreviousTutorialStep));
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
        ComponentType::HelpContent
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

#### 3. Help Search Engine
```rust
// src/tui/components/help/search_engine.rs
#[derive(Debug)]
pub struct HelpSearchEngine {
    content_index: ContentIndex,
    semantic_analyzer: SemanticAnalyzer,
    relevance_scorer: RelevanceScorer,
    search_cache: LruCache<String, SearchResults>,
}

impl HelpSearchEngine {
    pub fn new() -> Self {
        Self {
            content_index: ContentIndex::new(),
            semantic_analyzer: SemanticAnalyzer::new(),
            relevance_scorer: RelevanceScorer::new(),
            search_cache: LruCache::new(100),
        }
    }
    
    pub async fn initialize(&mut self, content_tree: &ContentTree) -> io::Result<()> {
        // Build full-text search index
        self.content_index.build_index(content_tree).await?;
        
        // Initialize semantic analyzer
        self.semantic_analyzer.initialize().await?;
        
        Ok(())
    }
    
    pub async fn search(&mut self, query: &str, options: &SearchOptions) -> io::Result<SearchResults> {
        // Check cache first
        let cache_key = format!("{}:{:?}", query, options);
        if let Some(cached_results) = self.search_cache.get(&cache_key) {
            return Ok(cached_results.clone());
        }
        
        let mut all_results = Vec::new();
        
        // Perform full-text search
        let text_results = self.content_index.search_text(query, options).await?;
        all_results.extend(text_results);
        
        // Perform semantic search if enabled
        if options.semantic_search {
            let semantic_results = self.semantic_analyzer.search_semantic(query, options).await?;
            all_results.extend(semantic_results);
        }
        
        // Remove duplicates and score results
        let mut unique_results = self.deduplicate_results(all_results);
        self.relevance_scorer.score_results(&mut unique_results, query, options).await?;
        
        // Sort by relevance score
        unique_results.sort_by(|a, b| b.relevance_score.partial_cmp(&a.relevance_score).unwrap_or(std::cmp::Ordering::Equal));
        
        let search_results = SearchResults {
            query: query.to_string(),
            results: unique_results,
            total_found: 0, // Will be set by deduplicate_results
            search_time: std::time::Instant::now().elapsed(),
        };
        
        // Cache results
        self.search_cache.put(cache_key, search_results.clone());
        
        Ok(search_results)
    }
    
    fn deduplicate_results(&self, results: Vec<SearchResult>) -> Vec<SearchResult> {
        let mut unique_results = Vec::new();
        let mut seen_ids = std::collections::HashSet::new();
        
        for result in results {
            if seen_ids.insert(result.id.clone()) {
                unique_results.push(result);
            }
        }
        
        unique_results
    }
}

#[derive(Debug)]
pub struct ContentIndex {
    text_index: HashMap<String, Vec<IndexEntry>>,
    content_metadata: HashMap<String, ContentMetadata>,
}

impl ContentIndex {
    pub fn new() -> Self {
        Self {
            text_index: HashMap::new(),
            content_metadata: HashMap::new(),
        }
    }
    
    pub async fn build_index(&mut self, content_tree: &ContentTree) -> io::Result<()> {
        for content_item in content_tree.flatten() {
            self.index_content_item(&content_item).await?;
        }
        
        Ok(())
    }
    
    async fn index_content_item(&mut self, content_item: &ContentItem) -> io::Result<()> {
        let content_id = content_item.id.clone();
        
        // Extract and tokenize text
        let text_content = self.extract_text_content(content_item)?;
        let tokens = self.tokenize_text(&text_content);
        
        // Build inverted index
        for (position, token) in tokens.iter().enumerate() {
            let entry = IndexEntry {
                content_id: content_id.clone(),
                position,
                context: self.extract_context(&tokens, position, 10),
            };
            
            self.text_index.entry(token.to_lowercase())
                .or_insert_with(Vec::new)
                .push(entry);
        }
        
        // Store metadata
        self.content_metadata.insert(content_id, ContentMetadata {
            title: content_item.title.clone(),
            content_type: content_item.content_type.clone(),
            tags: content_item.tags.clone(),
            last_modified: content_item.last_modified,
            word_count: tokens.len(),
        });
        
        Ok(())
    }
    
    pub async fn search_text(&self, query: &str, options: &SearchOptions) -> io::Result<Vec<SearchResult>> {
        let query_tokens = self.tokenize_text(query);
        let mut results = Vec::new();
        
        for token in &query_tokens {
            if let Some(entries) = self.text_index.get(&token.to_lowercase()) {
                for entry in entries {
                    if let Some(metadata) = self.content_metadata.get(&entry.content_id) {
                        // Check if this content type should be included
                        if self.should_include_content_type(&metadata.content_type, options) {
                            results.push(SearchResult {
                                id: entry.content_id.clone(),
                                title: metadata.title.clone(),
                                snippet: Some(entry.context.clone()),
                                result_type: self.map_content_type(&metadata.content_type),
                                relevance_score: 0.0, // Will be calculated later
                                match_positions: vec![entry.position],
                            });
                        }
                    }
                }
            }
        }
        
        Ok(results)
    }
    
    fn tokenize_text(&self, text: &str) -> Vec<String> {
        text.split_whitespace()
            .map(|s| s.trim_matches(|c: char| !c.is_alphanumeric()))
            .filter(|s| !s.is_empty() && s.len() > 2)
            .map(|s| s.to_string())
            .collect()
    }
    
    fn extract_context(&self, tokens: &[String], position: usize, window_size: usize) -> String {
        let start = position.saturating_sub(window_size / 2);
        let end = (position + window_size / 2 + 1).min(tokens.len());
        
        tokens[start..end].join(" ")
    }
    
    fn extract_text_content(&self, content_item: &ContentItem) -> io::Result<String> {
        match &content_item.content_type {
            ContentType::Documentation(doc) => Ok(doc.markdown.clone()),
            ContentType::Tutorial(tutorial) => Ok(
                tutorial.steps.iter()
                    .map(|step| format!("{}\n{}", step.title, step.content))
                    .collect::<Vec<_>>()
                    .join("\n\n")
            ),
            ContentType::ApiReference(api) => Ok(
                format!("{}\n{}\n{}", 
                       api.name, 
                       api.description, 
                       api.example.as_ref().map(|e| &e.code).unwrap_or(""))
            ),
            ContentType::Example(example) => Ok(
                format!("{}\n{}\n{}", 
                       example.title, 
                       example.description, 
                       example.code)
            ),
        }
    }
    
    fn should_include_content_type(&self, content_type: &ContentType, options: &SearchOptions) -> bool {
        match content_type {
            ContentType::Documentation(_) => options.include_content,
            ContentType::Tutorial(_) => options.include_content,
            ContentType::ApiReference(_) => options.include_api_docs,
            ContentType::Example(_) => options.include_examples,
        }
    }
    
    fn map_content_type(&self, content_type: &ContentType) -> SearchResultType {
        match content_type {
            ContentType::Documentation(_) => SearchResultType::Documentation,
            ContentType::Tutorial(_) => SearchResultType::Tutorial,
            ContentType::ApiReference(_) => SearchResultType::ApiReference,
            ContentType::Example(_) => SearchResultType::Example,
        }
    }
}

#[derive(Debug)]
pub struct RelevanceScorer {
    scoring_weights: ScoringWeights,
}

#[derive(Debug)]
pub struct ScoringWeights {
    pub title_match: f64,
    pub exact_match: f64,
    pub partial_match: f64,
    pub content_type_relevance: f64,
    pub recency: f64,
    pub user_context: f64,
}

impl Default for ScoringWeights {
    fn default() -> Self {
        Self {
            title_match: 2.0,
            exact_match: 1.5,
            partial_match: 1.0,
            content_type_relevance: 0.8,
            recency: 0.3,
            user_context: 0.5,
        }
    }
}

impl RelevanceScorer {
    pub fn new() -> Self {
        Self {
            scoring_weights: ScoringWeights::default(),
        }
    }
    
    pub async fn score_results(&self, results: &mut Vec<SearchResult>, query: &str, options: &SearchOptions) -> io::Result<()> {
        let query_lower = query.to_lowercase();
        
        for result in results {
            let mut score = 0.0;
            
            // Title match scoring
            let title_lower = result.title.to_lowercase();
            if title_lower == query_lower {
                score += self.scoring_weights.title_match * 2.0;
            } else if title_lower.contains(&query_lower) {
                score += self.scoring_weights.title_match;
            }
            
            // Content match scoring
            if let Some(snippet) = &result.snippet {
                let snippet_lower = snippet.to_lowercase();
                if snippet_lower.contains(&query_lower) {
                    score += self.scoring_weights.exact_match;
                }
            }
            
            // Content type relevance
            score += self.calculate_content_type_relevance(&result.result_type, options);
            
            // User context relevance
            if let Some(user_context) = &options.user_context {
                score += self.calculate_user_context_relevance(result, user_context);
            }
            
            result.relevance_score = score;
        }
        
        Ok(())
    }
    
    fn calculate_content_type_relevance(&self, result_type: &SearchResultType, options: &SearchOptions) -> f64 {
        // Boost scores based on user preferences or current context
        match result_type {
            SearchResultType::Tutorial => {
                if options.user_context.as_ref()
                    .map(|ctx| ctx.experience_level < 3)
                    .unwrap_or(false) {
                    self.scoring_weights.content_type_relevance * 1.5
                } else {
                    self.scoring_weights.content_type_relevance
                }
            },
            SearchResultType::ApiReference => {
                if options.user_context.as_ref()
                    .map(|ctx| ctx.experience_level > 6)
                    .unwrap_or(false) {
                    self.scoring_weights.content_type_relevance * 1.5
                } else {
                    self.scoring_weights.content_type_relevance
                }
            },
            _ => self.scoring_weights.content_type_relevance,
        }
    }
    
    fn calculate_user_context_relevance(&self, result: &SearchResult, user_context: &UserContext) -> f64 {
        // Consider user's recent activity, preferences, and current task
        let mut relevance = 0.0;
        
        // Check if this content relates to user's current context
        if let Some(current_topic) = &user_context.current_topic {
            if result.title.to_lowercase().contains(&current_topic.to_lowercase()) {
                relevance += self.scoring_weights.user_context;
            }
        }
        
        // Check user's preferred content types
        if user_context.preferred_content_types.contains(&result.result_type) {
            relevance += self.scoring_weights.user_context * 0.5;
        }
        
        relevance
    }
}
```

### Why This Approach

**Comprehensive Knowledge Base:**
- Structured documentation with multiple content types
- Interactive tutorials with step-by-step guidance
- Complete API reference with live examples

**Intelligent Search:**
- Full-text search with semantic understanding
- Context-aware relevance scoring
- User-personalized search results

**Progressive Learning:**
- Tutorial system with progress tracking
- Context-sensitive help integration
- Bookmark and navigation history management

### How to Implement

#### Step 1: Help Browser Core (5 hours)
1. **Implement HelpBrowserView** with content management
2. **Add navigation** and browse mode handling
3. **Create content loading** and caching system
4. **Add bookmark management** and history tracking

#### Step 2: Content Rendering (4 hours)
1. **Implement HelpContentComponent** with multiple content types
2. **Add markdown rendering** and syntax highlighting
3. **Create tutorial renderer** with progress visualization
4. **Add API reference** formatting and examples

#### Step 3: Search Engine (5 hours)
1. **Implement HelpSearchEngine** with full-text search
2. **Add semantic analysis** and relevance scoring
3. **Create content indexing** system
4. **Add search result** ranking and filtering

#### Step 4: Tutorial System (3 hours)
1. **Implement TutorialSystem** with step management
2. **Add progress tracking** and validation
3. **Create interactive tutorials** with examples
4. **Add tutorial completion** and achievement system

#### Step 5: Integration and Testing (2 hours)
1. **Add comprehensive unit tests** for all help components
2. **Test search accuracy** and performance
3. **Validate tutorial system** effectiveness
4. **Test content rendering** and navigation

## 🔗 Dependencies
**Depends On:**
- Issue #023 (TUI Architecture Foundation) - Base view and component system
- Issue #024 (Dashboard Overview) - Context integration
- Phase 2 enhanced commands for contextual help

**Enables:**
- Comprehensive developer self-service support
- Integration with all other TUI views for contextual help

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_help_search_accuracy() {
        let mut search_engine = HelpSearchEngine::new();
        let content_tree = ContentTree::test_data();
        
        search_engine.initialize(&content_tree).await.unwrap();
        
        let results = search_engine.search("dataflow tutorial", &SearchOptions::default()).await.unwrap();
        
        assert!(!results.results.is_empty());
        assert!(results.results[0].result_type == SearchResultType::Tutorial);
        assert!(results.results[0].relevance_score > 0.5);
    }
    
    #[test]
    fn test_tutorial_progress_tracking() {
        let mut tutorial_system = TutorialSystem::new();
        let tutorial_id = "getting-started";
        
        tutorial_system.start_tutorial(tutorial_id).await.unwrap();
        tutorial_system.complete_step(0).await.unwrap();
        
        let progress = tutorial_system.get_progress(tutorial_id).await.unwrap();
        assert_eq!(progress.completed_steps, 1);
        assert!(!progress.is_completed());
    }
    
    #[test]
    fn test_content_rendering_performance() {
        let content_component = HelpContentComponent::new(HelpBrowserContext::default());
        let large_content = HelpContent::test_large_documentation();
        
        let start_time = std::time::Instant::now();
        // Simulate rendering
        let render_time = start_time.elapsed();
        
        assert!(render_time < Duration::from_millis(100));
    }
}
```

## ✅ Definition of Done
- [ ] HelpBrowserView provides comprehensive documentation browsing interface
- [ ] Search engine returns relevant results within performance targets
- [ ] Content rendering supports all documentation formats clearly
- [ ] Tutorial system enables interactive learning with progress tracking
- [ ] API reference browser provides complete function documentation
- [ ] Navigation system maintains history and supports bookmarks
- [ ] Context-sensitive help integrates with all system views
- [ ] Search relevance scoring accounts for user context and preferences
- [ ] Performance targets met for content loading and search operations
- [ ] Tutorial completion rates meet success criteria
- [ ] Comprehensive unit tests validate all help browser functionality
- [ ] Integration tests confirm search accuracy and tutorial effectiveness
- [ ] Manual testing validates developer learning and problem-solving efficiency

This interactive help browser view provides developers with comprehensive self-service support, enabling efficient learning and problem resolution through intelligent search, structured documentation, and progressive tutorials.