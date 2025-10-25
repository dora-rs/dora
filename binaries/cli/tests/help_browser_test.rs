/// Tests for Help Browser View (Issue #31)
#[cfg(test)]
mod help_browser_tests {
    use dora_cli::tui::views::{
        HelpCategory, HelpTopic, HelpContent, HelpBrowserState,
        HelpBrowserView, View,
    };
    use dora_cli::tui::theme::ThemeConfig;

    // HelpCategory tests
    #[test]
    fn test_help_category_all() {
        let categories = HelpCategory::all();
        assert_eq!(categories.len(), 5);
        assert_eq!(categories[0], HelpCategory::GettingStarted);
        assert_eq!(categories[1], HelpCategory::Commands);
        assert_eq!(categories[2], HelpCategory::TuiGuide);
        assert_eq!(categories[3], HelpCategory::Troubleshooting);
        assert_eq!(categories[4], HelpCategory::Faq);
    }

    #[test]
    fn test_help_category_names() {
        assert_eq!(HelpCategory::GettingStarted.name(), "Getting Started");
        assert_eq!(HelpCategory::Commands.name(), "Commands");
        assert_eq!(HelpCategory::TuiGuide.name(), "TUI Guide");
        assert_eq!(HelpCategory::Troubleshooting.name(), "Troubleshooting");
        assert_eq!(HelpCategory::Faq.name(), "FAQ");
    }

    #[test]
    fn test_help_category_icons() {
        assert_eq!(HelpCategory::GettingStarted.icon(), "🚀");
        assert_eq!(HelpCategory::Commands.icon(), "⌨");
        assert_eq!(HelpCategory::TuiGuide.icon(), "📖");
        assert_eq!(HelpCategory::Troubleshooting.icon(), "🔧");
        assert_eq!(HelpCategory::Faq.icon(), "❓");
    }

    #[test]
    fn test_help_category_navigation() {
        let mut cat = HelpCategory::GettingStarted;
        cat = cat.next();
        assert_eq!(cat, HelpCategory::Commands);
        cat = cat.next();
        assert_eq!(cat, HelpCategory::TuiGuide);

        cat = cat.prev();
        assert_eq!(cat, HelpCategory::Commands);
        cat = cat.prev();
        assert_eq!(cat, HelpCategory::GettingStarted);

        // Test wrap-around
        cat = cat.prev();
        assert_eq!(cat, HelpCategory::Faq);
    }

    // HelpTopic tests
    #[test]
    fn test_help_topic_creation() {
        let topic = HelpTopic::new(
            "test".to_string(),
            "Test Topic".to_string(),
            HelpCategory::GettingStarted,
            "Test summary".to_string(),
        );

        assert_eq!(topic.id, "test");
        assert_eq!(topic.title, "Test Topic");
        assert_eq!(topic.category, HelpCategory::GettingStarted);
        assert_eq!(topic.summary, "Test summary");
    }

    #[test]
    fn test_help_topic_belongs_to_category() {
        let topic = HelpTopic::new(
            "test".to_string(),
            "Test".to_string(),
            HelpCategory::Commands,
            "Test".to_string(),
        );

        assert!(topic.belongs_to(HelpCategory::Commands));
        assert!(!topic.belongs_to(HelpCategory::GettingStarted));
    }

    // HelpContent tests
    #[test]
    fn test_help_content_creation() {
        let content = HelpContent::new(
            "test".to_string(),
            "Test Content".to_string(),
            vec!["Line 1".to_string(), "Line 2".to_string()],
            vec!["related1".to_string()],
        );

        assert_eq!(content.topic_id, "test");
        assert_eq!(content.title, "Test Content");
        assert_eq!(content.content.len(), 2);
        assert_eq!(content.related_topics.len(), 1);
    }

    #[test]
    fn test_help_content_line_count() {
        let content = HelpContent::new(
            "test".to_string(),
            "Test".to_string(),
            vec!["Line 1".to_string(), "Line 2".to_string(), "Line 3".to_string()],
            vec![],
        );

        assert_eq!(content.line_count(), 3);
    }

    #[test]
    fn test_help_content_has_related_topics() {
        let content_with_related = HelpContent::new(
            "test".to_string(),
            "Test".to_string(),
            vec![],
            vec!["related1".to_string()],
        );

        let content_without_related = HelpContent::new(
            "test".to_string(),
            "Test".to_string(),
            vec![],
            vec![],
        );

        assert!(content_with_related.has_related_topics());
        assert!(!content_without_related.has_related_topics());
    }

    // HelpBrowserState tests
    #[test]
    fn test_help_browser_state_new() {
        let state = HelpBrowserState::new();

        assert_eq!(state.current_category, HelpCategory::GettingStarted);
        assert_eq!(state.current_topic_index, 0);
        assert_eq!(state.scroll_offset, 0);
        assert!(state.topics.len() > 0);
        // Content is loaded for the first topic by default
        assert!(state.current_content.is_some());
    }

    #[test]
    fn test_help_browser_state_has_mock_topics() {
        let state = HelpBrowserState::new();

        // Check we have topics in all categories
        let getting_started_topics: Vec<_> = state.topics
            .iter()
            .filter(|t| t.category == HelpCategory::GettingStarted)
            .collect();
        assert!(getting_started_topics.len() > 0);

        let commands_topics: Vec<_> = state.topics
            .iter()
            .filter(|t| t.category == HelpCategory::Commands)
            .collect();
        assert!(commands_topics.len() > 0);

        let tui_topics: Vec<_> = state.topics
            .iter()
            .filter(|t| t.category == HelpCategory::TuiGuide)
            .collect();
        assert!(tui_topics.len() > 0);
    }

    #[test]
    fn test_help_browser_state_switch_category() {
        let mut state = HelpBrowserState::new();
        state.current_topic_index = 2;

        state.switch_category(HelpCategory::Commands);

        assert_eq!(state.current_category, HelpCategory::Commands);
        assert_eq!(state.current_topic_index, 0); // Reset on category switch
    }

    #[test]
    fn test_help_browser_state_next_prev_category() {
        let mut state = HelpBrowserState::new();

        state.next_category();
        assert_eq!(state.current_category, HelpCategory::Commands);

        state.next_category();
        assert_eq!(state.current_category, HelpCategory::TuiGuide);

        state.prev_category();
        assert_eq!(state.current_category, HelpCategory::Commands);

        state.prev_category();
        assert_eq!(state.current_category, HelpCategory::GettingStarted);
    }

    #[test]
    fn test_help_browser_state_current_topics() {
        let state = HelpBrowserState::new();
        let topics = state.current_topics();

        // Should only get topics for current category
        assert!(topics.len() > 0);
        for topic in topics {
            assert_eq!(topic.category, state.current_category);
        }
    }

    #[test]
    fn test_help_browser_state_navigate_up_down() {
        let mut state = HelpBrowserState::new();

        assert_eq!(state.current_topic_index, 0);

        state.navigate_down();
        assert_eq!(state.current_topic_index, 1);

        state.navigate_down();
        assert_eq!(state.current_topic_index, 2);

        state.navigate_up();
        assert_eq!(state.current_topic_index, 1);

        state.navigate_up();
        assert_eq!(state.current_topic_index, 0);

        // Should not go below 0
        state.navigate_up();
        assert_eq!(state.current_topic_index, 0);
    }

    #[test]
    fn test_help_browser_state_navigate_bounds() {
        let mut state = HelpBrowserState::new();
        let max_index = state.current_topics().len().saturating_sub(1);

        // Navigate to the end
        for _ in 0..=max_index {
            state.navigate_down();
        }

        // Should stop at max_index
        assert_eq!(state.current_topic_index, max_index);

        // Try to go further - should stay at max_index
        state.navigate_down();
        assert_eq!(state.current_topic_index, max_index);
    }

    #[test]
    fn test_help_browser_state_select_current_topic() {
        let mut state = HelpBrowserState::new();

        state.select_current_topic();

        assert!(state.current_content.is_some());
        let content = state.current_content.as_ref().unwrap();
        assert!(content.content.len() > 0);
    }

    #[test]
    fn test_help_browser_state_scroll_up_down() {
        let mut state = HelpBrowserState::new();
        state.select_current_topic();

        assert_eq!(state.scroll_offset, 0);

        state.scroll_down();
        assert_eq!(state.scroll_offset, 1);

        state.scroll_down();
        assert_eq!(state.scroll_offset, 2);

        state.scroll_up();
        assert_eq!(state.scroll_offset, 1);

        state.scroll_up();
        assert_eq!(state.scroll_offset, 0);

        // Should not go below 0
        state.scroll_up();
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn test_help_browser_state_page_up_down() {
        let mut state = HelpBrowserState::new();
        state.select_current_topic();

        assert_eq!(state.scroll_offset, 0);

        state.page_down();
        assert_eq!(state.scroll_offset, 10);

        state.page_down();
        assert_eq!(state.scroll_offset, 20);

        state.page_up();
        assert_eq!(state.scroll_offset, 10);

        state.page_up();
        assert_eq!(state.scroll_offset, 0);

        // Should not go below 0
        state.page_up();
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn test_help_browser_state_scroll_to_top_bottom() {
        let mut state = HelpBrowserState::new();
        state.select_current_topic();

        state.scroll_offset = 50;

        state.scroll_to_top();
        assert_eq!(state.scroll_offset, 0);

        state.scroll_to_bottom();
        if let Some(content) = &state.current_content {
            assert!(state.scroll_offset > 0);
            assert!(state.scroll_offset <= content.content.len());
        }
    }

    #[test]
    fn test_help_browser_state_reset_scroll() {
        let mut state = HelpBrowserState::new();
        state.scroll_offset = 50;

        state.reset_scroll();
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn test_help_browser_state_mark_refreshed() {
        let mut state = HelpBrowserState::new();
        let before = state.last_refresh;

        std::thread::sleep(std::time::Duration::from_millis(10));
        state.mark_refreshed();

        assert!(state.last_refresh > before);
    }

    #[test]
    fn test_help_browser_state_current_category_count() {
        let state = HelpBrowserState::new();
        let count = state.current_category_count();

        assert!(count > 0);
        assert_eq!(count, state.current_topics().len());
    }

    // HelpBrowserView tests
    #[test]
    fn test_help_browser_view_creation() {
        let theme = ThemeConfig::default();
        let view = HelpBrowserView::new(&theme);

        assert_eq!(view.title(), "Help Browser");
        assert_eq!(view.state.current_category, HelpCategory::GettingStarted);
    }

    #[test]
    fn test_help_browser_view_has_mock_data() {
        let theme = ThemeConfig::default();
        let view = HelpBrowserView::new(&theme);

        assert!(view.state.topics.len() > 0);
        assert!(view.state.topics.len() >= 15); // At least 15 topics
    }

    #[test]
    fn test_help_browser_view_auto_refresh() {
        let theme = ThemeConfig::default();
        let view = HelpBrowserView::new(&theme);

        assert!(view.auto_refresh().is_some());
        assert_eq!(view.auto_refresh().unwrap(), std::time::Duration::from_millis(1000));
    }

    #[test]
    fn test_help_browser_view_help_text() {
        let theme = ThemeConfig::default();
        let view = HelpBrowserView::new(&theme);

        let help = view.help_text();
        assert!(help.len() >= 7);
        assert!(help.iter().any(|(key, _)| *key == "Tab"));
        assert!(help.iter().any(|(key, _)| *key == "Enter"));
        assert!(help.iter().any(|(key, _)| *key == "j/k"));
    }

    #[test]
    fn test_complete_help_browser_workflow() {
        let mut state = HelpBrowserState::new();

        // Start in Getting Started
        assert_eq!(state.current_category, HelpCategory::GettingStarted);

        // Navigate to Commands
        state.next_category();
        assert_eq!(state.current_category, HelpCategory::Commands);

        // Select second topic
        state.navigate_down();
        assert_eq!(state.current_topic_index, 1);

        // Load topic content
        state.select_current_topic();
        assert!(state.current_content.is_some());

        // Scroll content
        state.scroll_down();
        assert_eq!(state.scroll_offset, 1);

        // Navigate to TUI Guide
        state.next_category();
        assert_eq!(state.current_category, HelpCategory::TuiGuide);
        assert_eq!(state.current_topic_index, 0); // Reset on category switch

        // Load topic
        state.select_current_topic();
        assert!(state.current_content.is_some());

        // Page down
        state.page_down();
        assert_eq!(state.scroll_offset, 10);

        // Go back to Commands
        state.prev_category();
        assert_eq!(state.current_category, HelpCategory::Commands);

        // Scroll to top
        state.scroll_to_top();
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn test_all_categories_have_topics() {
        let state = HelpBrowserState::new();

        for category in HelpCategory::all() {
            let topics: Vec<_> = state.topics
                .iter()
                .filter(|t| t.category == category)
                .collect();
            assert!(
                topics.len() > 0,
                "Category {:?} should have topics",
                category
            );
        }
    }

    #[test]
    fn test_topic_content_generation() {
        let state = HelpBrowserState::new();

        // Get first topic from Getting Started
        let topics = state.current_topics();
        assert!(topics.len() > 0);

        let first_topic = &topics[0];
        let content = HelpBrowserState::create_mock_content(&first_topic.id);

        assert_eq!(content.topic_id, first_topic.id);
        assert!(content.content.len() > 0);
    }

    #[test]
    fn test_category_switching_resets_selection() {
        let mut state = HelpBrowserState::new();

        // Select a topic
        state.navigate_down();
        state.navigate_down();
        assert_eq!(state.current_topic_index, 2);

        // Switch category
        state.next_category();

        // Selection should be reset
        assert_eq!(state.current_topic_index, 0);
    }

    #[test]
    fn test_scroll_bounds_with_content() {
        let mut state = HelpBrowserState::new();
        state.select_current_topic();

        if let Some(content) = &state.current_content {
            let max_scroll = content.content.len().saturating_sub(1);

            // Try to scroll past the end
            for _ in 0..1000 {
                state.scroll_down();
            }

            // Should not exceed max
            assert!(state.scroll_offset <= max_scroll);
        }
    }

    #[test]
    fn test_help_browser_state_all_categories_accessible() {
        let mut state = HelpBrowserState::new();

        // Verify we can navigate to all categories
        for expected_category in HelpCategory::all() {
            state.switch_category(expected_category);
            assert_eq!(state.current_category, expected_category);
            assert!(state.current_topics().len() > 0);
        }
    }

    #[test]
    fn test_help_content_has_mock_data() {
        let content = HelpBrowserState::create_mock_content("installation");

        assert_eq!(content.topic_id, "installation");
        assert!(content.title.len() > 0);
        assert!(content.content.len() > 5); // Should have multiple lines
        assert!(content.related_topics.len() > 0);
    }
}
