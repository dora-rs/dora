/// Help Browser types and state management (Issue #31 - Phase 1)
/// Provides type definitions for help content browsing with mock data support.
use std::time::Instant;

/// Help content category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HelpCategory {
    GettingStarted,
    Commands,
    TuiGuide,
    Troubleshooting,
    Faq,
}

impl HelpCategory {
    /// Get all available categories
    pub fn all() -> Vec<Self> {
        vec![
            Self::GettingStarted,
            Self::Commands,
            Self::TuiGuide,
            Self::Troubleshooting,
            Self::Faq,
        ]
    }

    /// Get display name for the category
    pub fn name(&self) -> &str {
        match self {
            Self::GettingStarted => "Getting Started",
            Self::Commands => "Commands",
            Self::TuiGuide => "TUI Guide",
            Self::Troubleshooting => "Troubleshooting",
            Self::Faq => "FAQ",
        }
    }

    /// Get icon for the category
    pub fn icon(&self) -> &str {
        match self {
            Self::GettingStarted => "🚀",
            Self::Commands => "⌨",
            Self::TuiGuide => "📖",
            Self::Troubleshooting => "🔧",
            Self::Faq => "❓",
        }
    }

    /// Navigate to next category
    pub fn next(&self) -> Self {
        match self {
            Self::GettingStarted => Self::Commands,
            Self::Commands => Self::TuiGuide,
            Self::TuiGuide => Self::Troubleshooting,
            Self::Troubleshooting => Self::Faq,
            Self::Faq => Self::GettingStarted,
        }
    }

    /// Navigate to previous category
    pub fn prev(&self) -> Self {
        match self {
            Self::GettingStarted => Self::Faq,
            Self::Commands => Self::GettingStarted,
            Self::TuiGuide => Self::Commands,
            Self::Troubleshooting => Self::TuiGuide,
            Self::Faq => Self::Troubleshooting,
        }
    }
}

/// Help topic within a category
#[derive(Debug, Clone, PartialEq)]
pub struct HelpTopic {
    pub id: String,
    pub title: String,
    pub category: HelpCategory,
    pub summary: String,
}

impl HelpTopic {
    /// Create a new help topic
    pub fn new(id: String, title: String, category: HelpCategory, summary: String) -> Self {
        Self {
            id,
            title,
            category,
            summary,
        }
    }

    /// Format topic for display
    pub fn format_display(&self) -> String {
        format!("{} - {}", self.title, self.summary)
    }

    /// Check if topic belongs to a category
    pub fn belongs_to(&self, category: HelpCategory) -> bool {
        self.category == category
    }
}

/// Help content for a topic
#[derive(Debug, Clone, PartialEq)]
pub struct HelpContent {
    pub topic_id: String,
    pub title: String,
    pub content: Vec<String>,
    pub related_topics: Vec<String>,
}

impl HelpContent {
    /// Create new help content
    pub fn new(
        topic_id: String,
        title: String,
        content: Vec<String>,
        related_topics: Vec<String>,
    ) -> Self {
        Self {
            topic_id,
            title,
            content,
            related_topics,
        }
    }

    /// Get content line count
    pub fn line_count(&self) -> usize {
        self.content.len()
    }

    /// Check if content has related topics
    pub fn has_related_topics(&self) -> bool {
        !self.related_topics.is_empty()
    }
}

/// Help browser state
pub struct HelpBrowserState {
    pub current_category: HelpCategory,
    pub topics: Vec<HelpTopic>,
    pub current_topic_index: usize,
    pub current_content: Option<HelpContent>,
    pub scroll_offset: usize,
    pub last_refresh: Instant,
}

impl Default for HelpBrowserState {
    fn default() -> Self {
        Self::new()
    }
}

impl HelpBrowserState {
    /// Create new help browser state with mock data
    pub fn new() -> Self {
        let topics = Self::create_all_topics();
        let current_content = if !topics.is_empty() {
            Some(Self::create_content_for_topic(&topics[0]))
        } else {
            None
        };

        Self {
            current_category: HelpCategory::GettingStarted,
            topics,
            current_topic_index: 0,
            current_content,
            scroll_offset: 0,
            last_refresh: Instant::now(),
        }
    }

    /// Switch to a different category
    pub fn switch_category(&mut self, category: HelpCategory) {
        self.current_category = category;
        self.current_topic_index = 0;
        self.scroll_offset = 0;
        self.load_current_topic_content();
    }

    /// Navigate to next category
    pub fn next_category(&mut self) {
        self.current_category = self.current_category.next();
        self.current_topic_index = 0;
        self.scroll_offset = 0;
        self.load_current_topic_content();
    }

    /// Navigate to previous category
    pub fn prev_category(&mut self) {
        self.current_category = self.current_category.prev();
        self.current_topic_index = 0;
        self.scroll_offset = 0;
        self.load_current_topic_content();
    }

    /// Get current category topics
    pub fn current_category_topics(&self) -> Vec<&HelpTopic> {
        self.topics
            .iter()
            .filter(|t| t.category == self.current_category)
            .collect()
    }

    /// Alias for current_category_topics
    pub fn current_topics(&self) -> Vec<&HelpTopic> {
        self.current_category_topics()
    }

    /// Navigate down in topics list
    pub fn navigate_down(&mut self) {
        let count = self.current_category_topics().len();
        if count > 0 && self.current_topic_index + 1 < count {
            self.current_topic_index += 1;
            self.scroll_offset = 0;
            self.load_current_topic_content();
        }
    }

    /// Navigate up in topics list
    pub fn navigate_up(&mut self) {
        if self.current_topic_index > 0 {
            self.current_topic_index -= 1;
            self.scroll_offset = 0;
            self.load_current_topic_content();
        }
    }

    /// Select and load the current topic
    pub fn select_current_topic(&mut self) {
        self.load_current_topic_content();
    }

    /// Scroll content down
    pub fn scroll_down(&mut self) {
        if let Some(content) = &self.current_content {
            if self.scroll_offset + 1 < content.line_count() {
                self.scroll_offset += 1;
            }
        }
    }

    /// Scroll content up
    pub fn scroll_up(&mut self) {
        if self.scroll_offset > 0 {
            self.scroll_offset -= 1;
        }
    }

    /// Page down
    pub fn page_down(&mut self) {
        if let Some(content) = &self.current_content {
            self.scroll_offset =
                (self.scroll_offset + 10).min(content.line_count().saturating_sub(1));
        }
    }

    /// Page up
    pub fn page_up(&mut self) {
        self.scroll_offset = self.scroll_offset.saturating_sub(10);
    }

    /// Scroll to top of content
    pub fn scroll_to_top(&mut self) {
        self.scroll_offset = 0;
    }

    /// Scroll to bottom of content
    pub fn scroll_to_bottom(&mut self) {
        if let Some(content) = &self.current_content {
            self.scroll_offset = content.line_count().saturating_sub(1);
        }
    }

    /// Reset scroll position to top
    pub fn reset_scroll(&mut self) {
        self.scroll_offset = 0;
    }

    /// Get selected topic
    pub fn get_selected_topic(&self) -> Option<&HelpTopic> {
        let topics = self.current_category_topics();
        topics.get(self.current_topic_index).copied()
    }

    /// Load content for current topic
    fn load_current_topic_content(&mut self) {
        if let Some(topic) = self.get_selected_topic() {
            self.current_content = Some(Self::create_content_for_topic(topic));
        } else {
            self.current_content = None;
        }
    }

    /// Mark as refreshed
    pub fn mark_refreshed(&mut self) {
        self.last_refresh = Instant::now();
    }

    /// Get current category topic count
    pub fn current_category_count(&self) -> usize {
        self.current_category_topics().len()
    }

    // Mock data creation methods

    fn create_all_topics() -> Vec<HelpTopic> {
        let mut topics = Vec::new();

        // Getting Started topics
        topics.extend(vec![
            HelpTopic::new(
                "quick-start".to_string(),
                "Quick Start Guide".to_string(),
                HelpCategory::GettingStarted,
                "Get started with Dora in 5 minutes".to_string(),
            ),
            HelpTopic::new(
                "installation".to_string(),
                "Installation".to_string(),
                HelpCategory::GettingStarted,
                "How to install Dora".to_string(),
            ),
            HelpTopic::new(
                "first-dataflow".to_string(),
                "Your First Dataflow".to_string(),
                HelpCategory::GettingStarted,
                "Create and run your first dataflow".to_string(),
            ),
        ]);

        // Commands topics
        topics.extend(vec![
            HelpTopic::new(
                "cmd-start".to_string(),
                "Start Command".to_string(),
                HelpCategory::Commands,
                "Start a dataflow".to_string(),
            ),
            HelpTopic::new(
                "cmd-stop".to_string(),
                "Stop Command".to_string(),
                HelpCategory::Commands,
                "Stop a running dataflow".to_string(),
            ),
            HelpTopic::new(
                "cmd-ps".to_string(),
                "PS Command".to_string(),
                HelpCategory::Commands,
                "List running dataflows".to_string(),
            ),
            HelpTopic::new(
                "cmd-build".to_string(),
                "Build Command".to_string(),
                HelpCategory::Commands,
                "Build dataflow dependencies".to_string(),
            ),
        ]);

        // TUI Guide topics
        topics.extend(vec![
            HelpTopic::new(
                "tui-navigation".to_string(),
                "Navigation".to_string(),
                HelpCategory::TuiGuide,
                "How to navigate the TUI".to_string(),
            ),
            HelpTopic::new(
                "tui-dashboard".to_string(),
                "Dashboard".to_string(),
                HelpCategory::TuiGuide,
                "Using the dashboard view".to_string(),
            ),
            HelpTopic::new(
                "tui-logs".to_string(),
                "Log Viewer".to_string(),
                HelpCategory::TuiGuide,
                "Viewing and filtering logs".to_string(),
            ),
        ]);

        // Troubleshooting topics
        topics.extend(vec![
            HelpTopic::new(
                "ts-connection".to_string(),
                "Connection Issues".to_string(),
                HelpCategory::Troubleshooting,
                "Resolve connection problems".to_string(),
            ),
            HelpTopic::new(
                "ts-performance".to_string(),
                "Performance Issues".to_string(),
                HelpCategory::Troubleshooting,
                "Diagnose performance problems".to_string(),
            ),
        ]);

        // FAQ topics
        topics.extend(vec![
            HelpTopic::new(
                "faq-what-is-dora".to_string(),
                "What is Dora?".to_string(),
                HelpCategory::Faq,
                "Introduction to Dora".to_string(),
            ),
            HelpTopic::new(
                "faq-requirements".to_string(),
                "System Requirements".to_string(),
                HelpCategory::Faq,
                "What you need to run Dora".to_string(),
            ),
            HelpTopic::new(
                "faq-difference".to_string(),
                "Dora vs Other Tools".to_string(),
                HelpCategory::Faq,
                "How Dora compares".to_string(),
            ),
        ]);

        topics
    }

    /// Create mock content for a topic ID (for testing)
    pub fn create_mock_content(topic_id: &str) -> HelpContent {
        // Create a temporary topic for this ID
        let temp_topic = HelpTopic::new(
            topic_id.to_string(),
            "Mock Topic".to_string(),
            HelpCategory::GettingStarted,
            "Mock summary".to_string(),
        );
        Self::create_content_for_topic(&temp_topic)
    }

    fn create_content_for_topic(topic: &HelpTopic) -> HelpContent {
        match topic.id.as_str() {
            "quick-start" => HelpContent::new(
                topic.id.clone(),
                topic.title.clone(),
                vec![
                    "Welcome to Dora!".to_string(),
                    "".to_string(),
                    "Dora is a dataflow-oriented robotic framework that helps you build"
                        .to_string(),
                    "and manage complex robotic systems.".to_string(),
                    "".to_string(),
                    "Quick Start Steps:".to_string(),
                    "1. Install Dora CLI".to_string(),
                    "2. Create a new dataflow: dora new my-dataflow".to_string(),
                    "3. Start your dataflow: dora start dataflow.yml".to_string(),
                    "4. Monitor with TUI: dora ui".to_string(),
                    "".to_string(),
                    "Key Features:".to_string(),
                    "- Dataflow-based architecture".to_string(),
                    "- Multiple language support (Rust, Python, C/C++)".to_string(),
                    "- Zero-copy communication with Apache Arrow".to_string(),
                    "- Built-in monitoring and debugging tools".to_string(),
                    "- Distributed system support".to_string(),
                    "".to_string(),
                    "Getting Help:".to_string(),
                    "- Browse this help system for documentation".to_string(),
                    "- Check the FAQ section for common questions".to_string(),
                    "- Visit the troubleshooting guide for issues".to_string(),
                    "".to_string(),
                    "For more information, see the Installation and First Dataflow topics."
                        .to_string(),
                ],
                vec!["installation".to_string(), "first-dataflow".to_string()],
            ),
            "installation" => HelpContent::new(
                topic.id.clone(),
                topic.title.clone(),
                vec![
                    "Installing Dora".to_string(),
                    "".to_string(),
                    "Prerequisites:".to_string(),
                    "- Rust 1.70 or later".to_string(),
                    "- Git".to_string(),
                    "".to_string(),
                    "Installation Steps:".to_string(),
                    "1. Clone the repository:".to_string(),
                    "   git clone https://github.com/dora-rs/dora".to_string(),
                    "".to_string(),
                    "2. Install the CLI:".to_string(),
                    "   cargo install --path binaries/cli".to_string(),
                    "".to_string(),
                    "3. Verify installation:".to_string(),
                    "   dora --version".to_string(),
                ],
                vec!["quick-start".to_string(), "faq-requirements".to_string()],
            ),
            _ => HelpContent::new(
                topic.id.clone(),
                topic.title.clone(),
                vec![
                    format!("{}", topic.title),
                    "".to_string(),
                    format!("{}", topic.summary),
                    "".to_string(),
                    "This is mock help content for demonstration purposes.".to_string(),
                    "In a full implementation, this would contain detailed".to_string(),
                    "documentation, examples, and interactive tutorials.".to_string(),
                    "".to_string(),
                    "Key Points:".to_string(),
                    "- Feature overview".to_string(),
                    "- Usage examples".to_string(),
                    "- Common patterns".to_string(),
                    "- Best practices".to_string(),
                    "".to_string(),
                    "See related topics for more information.".to_string(),
                ],
                vec![],
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_help_category_all() {
        let categories = HelpCategory::all();
        assert_eq!(categories.len(), 5);
        assert_eq!(categories[0], HelpCategory::GettingStarted);
        assert_eq!(categories[4], HelpCategory::Faq);
    }

    #[test]
    fn test_help_category_names() {
        assert_eq!(HelpCategory::GettingStarted.name(), "Getting Started");
        assert_eq!(HelpCategory::Commands.name(), "Commands");
        assert_eq!(HelpCategory::TuiGuide.name(), "TUI Guide");
    }

    #[test]
    fn test_help_category_navigation() {
        let mut cat = HelpCategory::GettingStarted;
        cat = cat.next();
        assert_eq!(cat, HelpCategory::Commands);
        cat = cat.prev();
        assert_eq!(cat, HelpCategory::GettingStarted);
    }

    #[test]
    fn test_help_topic_creation() {
        let topic = HelpTopic::new(
            "test".to_string(),
            "Test Topic".to_string(),
            HelpCategory::GettingStarted,
            "A test topic".to_string(),
        );

        assert_eq!(topic.id, "test");
        assert_eq!(topic.title, "Test Topic");
        assert_eq!(topic.category, HelpCategory::GettingStarted);
    }

    #[test]
    fn test_help_content_creation() {
        let content = HelpContent::new(
            "test".to_string(),
            "Test".to_string(),
            vec!["Line 1".to_string(), "Line 2".to_string()],
            vec![],
        );

        assert_eq!(content.topic_id, "test");
        assert_eq!(content.line_count(), 2);
    }

    #[test]
    fn test_help_browser_state_creation() {
        let state = HelpBrowserState::new();
        assert_eq!(state.current_category, HelpCategory::GettingStarted);
        assert!(state.topics.len() > 0);
        assert!(state.current_content.is_some());
    }

    #[test]
    fn test_help_browser_state_category_switch() {
        let mut state = HelpBrowserState::new();
        state.switch_category(HelpCategory::Commands);
        assert_eq!(state.current_category, HelpCategory::Commands);
        assert_eq!(state.current_topic_index, 0);
    }

    #[test]
    fn test_help_browser_state_navigation() {
        let mut state = HelpBrowserState::new();
        state.next_category();
        assert_eq!(state.current_category, HelpCategory::Commands);
        state.prev_category();
        assert_eq!(state.current_category, HelpCategory::GettingStarted);
    }

    #[test]
    fn test_help_browser_state_topic_navigation() {
        let mut state = HelpBrowserState::new();
        state.navigate_down();
        assert_eq!(state.current_topic_index, 1);
        state.navigate_up();
        assert_eq!(state.current_topic_index, 0);
    }

    #[test]
    fn test_help_browser_state_scroll() {
        let mut state = HelpBrowserState::new();
        state.scroll_down();
        assert_eq!(state.scroll_offset, 1);
        state.scroll_up();
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn test_help_browser_state_current_category_topics() {
        let state = HelpBrowserState::new();
        let topics = state.current_category_topics();
        assert!(topics.len() > 0);
        for topic in topics {
            assert_eq!(topic.category, state.current_category);
        }
    }

    #[test]
    fn test_help_browser_state_get_selected_topic() {
        let state = HelpBrowserState::new();
        let topic = state.get_selected_topic();
        assert!(topic.is_some());
    }

    #[test]
    fn test_help_browser_state_mark_refreshed() {
        let mut state = HelpBrowserState::new();
        let before = state.last_refresh;
        std::thread::sleep(std::time::Duration::from_millis(10));
        state.mark_refreshed();
        assert!(state.last_refresh > before);
    }
}
