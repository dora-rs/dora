// Comprehensive tests for Collaborative Features (Issue #34)

use dora_cli::tui::{
    app::AppState,
    theme::ThemeConfig,
    views::{
        collab_features::CollabFeaturesView,
        collab_features_types::*,
        View, ViewAction,
    },
};
use crossterm::event::{KeyCode, KeyEvent};
use ratatui::{backend::TestBackend, Terminal};

// ============================================================================
// Helper Functions
// ============================================================================

fn create_test_view() -> CollabFeaturesView {
    CollabFeaturesView::new(&ThemeConfig::default())
}

fn create_test_terminal() -> Terminal<TestBackend> {
    let backend = TestBackend::new(80, 24);
    Terminal::new(backend).unwrap()
}

// ============================================================================
// CollabSection Tests
// ============================================================================

#[test]
fn test_collab_section_all() {
    let sections = CollabSection::all();
    assert_eq!(sections.len(), 4);
    assert_eq!(sections[0], CollabSection::TeamWorkspace);
    assert_eq!(sections[1], CollabSection::LiveSessions);
    assert_eq!(sections[2], CollabSection::CodeReviews);
    assert_eq!(sections[3], CollabSection::KnowledgeBase);
}

#[test]
fn test_collab_section_names() {
    assert_eq!(CollabSection::TeamWorkspace.name(), "Team Workspace");
    assert_eq!(CollabSection::LiveSessions.name(), "Live Sessions");
    assert_eq!(CollabSection::CodeReviews.name(), "Code Reviews");
    assert_eq!(CollabSection::KnowledgeBase.name(), "Knowledge Base");
}

#[test]
fn test_collab_section_descriptions() {
    assert!(!CollabSection::TeamWorkspace.description().is_empty());
    assert!(!CollabSection::LiveSessions.description().is_empty());
    assert!(!CollabSection::CodeReviews.description().is_empty());
    assert!(!CollabSection::KnowledgeBase.description().is_empty());
}

#[test]
fn test_collab_section_icons() {
    assert_eq!(CollabSection::TeamWorkspace.icon(), "👥");
    assert_eq!(CollabSection::LiveSessions.icon(), "🔴");
    assert_eq!(CollabSection::CodeReviews.icon(), "📝");
    assert_eq!(CollabSection::KnowledgeBase.icon(), "📚");
}

// ============================================================================
// TeamRole Tests
// ============================================================================

#[test]
fn test_team_role_labels() {
    assert_eq!(TeamRole::Owner.label(), "Owner");
    assert_eq!(TeamRole::Admin.label(), "Admin");
    assert_eq!(TeamRole::Developer.label(), "Developer");
    assert_eq!(TeamRole::Viewer.label(), "Viewer");
    assert_eq!(TeamRole::Guest.label(), "Guest");
}

#[test]
fn test_team_role_color_indicators() {
    assert_eq!(TeamRole::Owner.color_indicator(), "🟣");
    assert_eq!(TeamRole::Admin.color_indicator(), "🔵");
    assert_eq!(TeamRole::Developer.color_indicator(), "🟢");
    assert_eq!(TeamRole::Viewer.color_indicator(), "🟡");
    assert_eq!(TeamRole::Guest.color_indicator(), "⚪");
}

// ============================================================================
// PresenceStatus Tests
// ============================================================================

#[test]
fn test_presence_status_indicators() {
    assert_eq!(PresenceStatus::Online.indicator(), "🟢");
    assert_eq!(PresenceStatus::Away.indicator(), "🟡");
    assert_eq!(PresenceStatus::Busy.indicator(), "🔴");
    assert_eq!(PresenceStatus::Offline.indicator(), "⚫");
}

#[test]
fn test_presence_status_labels() {
    assert_eq!(PresenceStatus::Online.label(), "Online");
    assert_eq!(PresenceStatus::Away.label(), "Away");
    assert_eq!(PresenceStatus::Busy.label(), "Busy");
    assert_eq!(PresenceStatus::Offline.label(), "Offline");
}

// ============================================================================
// SessionType Tests
// ============================================================================

#[test]
fn test_session_type_labels() {
    assert_eq!(SessionType::CodeReview.label(), "Code Review");
    assert_eq!(SessionType::PairProgramming.label(), "Pair Programming");
    assert_eq!(SessionType::TeamDebug.label(), "Team Debug");
    assert_eq!(SessionType::LiveDemo.label(), "Live Demo");
    assert_eq!(SessionType::PlanningSession.label(), "Planning");
}

#[test]
fn test_session_type_icons() {
    assert_eq!(SessionType::CodeReview.icon(), "📝");
    assert_eq!(SessionType::PairProgramming.icon(), "👥");
    assert_eq!(SessionType::TeamDebug.icon(), "🐛");
    assert_eq!(SessionType::LiveDemo.icon(), "🎥");
    assert_eq!(SessionType::PlanningSession.icon(), "📋");
}

// ============================================================================
// ReviewStatus Tests
// ============================================================================

#[test]
fn test_review_status_labels() {
    assert_eq!(ReviewStatus::Draft.label(), "Draft");
    assert_eq!(ReviewStatus::PendingReview.label(), "Pending");
    assert_eq!(ReviewStatus::InReview.label(), "In Review");
    assert_eq!(ReviewStatus::ChangesRequested.label(), "Changes Requested");
    assert_eq!(ReviewStatus::Approved.label(), "Approved");
    assert_eq!(ReviewStatus::Merged.label(), "Merged");
    assert_eq!(ReviewStatus::Closed.label(), "Closed");
}

#[test]
fn test_review_status_color_indicators() {
    assert_eq!(ReviewStatus::Draft.color_indicator(), "⚪");
    assert_eq!(ReviewStatus::PendingReview.color_indicator(), "🟡");
    assert_eq!(ReviewStatus::InReview.color_indicator(), "🔵");
    assert_eq!(ReviewStatus::ChangesRequested.color_indicator(), "🟠");
    assert_eq!(ReviewStatus::Approved.color_indicator(), "🟢");
    assert_eq!(ReviewStatus::Merged.color_indicator(), "🟣");
    assert_eq!(ReviewStatus::Closed.color_indicator(), "⚫");
}

// ============================================================================
// DifficultyLevel Tests
// ============================================================================

#[test]
fn test_difficulty_level_labels() {
    assert_eq!(DifficultyLevel::Beginner.label(), "Beginner");
    assert_eq!(DifficultyLevel::Intermediate.label(), "Intermediate");
    assert_eq!(DifficultyLevel::Advanced.label(), "Advanced");
    assert_eq!(DifficultyLevel::Expert.label(), "Expert");
}

#[test]
fn test_difficulty_level_icons() {
    assert_eq!(DifficultyLevel::Beginner.icon(), "🌱");
    assert_eq!(DifficultyLevel::Intermediate.icon(), "🌿");
    assert_eq!(DifficultyLevel::Advanced.icon(), "🌳");
    assert_eq!(DifficultyLevel::Expert.icon(), "🎓");
}

// ============================================================================
// KnowledgeCategory Tests
// ============================================================================

#[test]
fn test_knowledge_category_labels() {
    assert_eq!(KnowledgeCategory::GettingStarted.label(), "Getting Started");
    assert_eq!(KnowledgeCategory::BestPractices.label(), "Best Practices");
    assert_eq!(KnowledgeCategory::Troubleshooting.label(), "Troubleshooting");
    assert_eq!(KnowledgeCategory::AdvancedTopics.label(), "Advanced Topics");
    assert_eq!(KnowledgeCategory::Tutorials.label(), "Tutorials");
    assert_eq!(KnowledgeCategory::ReferenceGuide.label(), "Reference Guide");
}

#[test]
fn test_knowledge_category_icons() {
    assert_eq!(KnowledgeCategory::GettingStarted.icon(), "🚀");
    assert_eq!(KnowledgeCategory::BestPractices.icon(), "⭐");
    assert_eq!(KnowledgeCategory::Troubleshooting.icon(), "🔧");
    assert_eq!(KnowledgeCategory::AdvancedTopics.icon(), "🎓");
    assert_eq!(KnowledgeCategory::Tutorials.icon(), "📖");
    assert_eq!(KnowledgeCategory::ReferenceGuide.icon(), "📚");
}

// ============================================================================
// Mock Data Tests
// ============================================================================

#[test]
fn test_mock_team_members() {
    let members = create_mock_team_members();
    assert_eq!(members.len(), 6);

    // Verify all roles are represented
    assert!(members.iter().any(|m| m.role == TeamRole::Owner));
    assert!(members.iter().any(|m| m.role == TeamRole::Admin));
    assert!(members.iter().any(|m| m.role == TeamRole::Developer));
    assert!(members.iter().any(|m| m.role == TeamRole::Viewer));
    assert!(members.iter().any(|m| m.role == TeamRole::Guest));

    // Verify all have usernames and emails
    assert!(members.iter().all(|m| !m.username.is_empty()));
    assert!(members.iter().all(|m| !m.email.is_empty()));
}

#[test]
fn test_mock_sessions() {
    let sessions = create_mock_sessions();
    assert_eq!(sessions.len(), 4);

    // Verify all session types are represented
    assert!(sessions.iter().any(|s| s.session_type == SessionType::CodeReview));
    assert!(sessions.iter().any(|s| s.session_type == SessionType::PairProgramming));
    assert!(sessions.iter().any(|s| s.session_type == SessionType::TeamDebug));
    assert!(sessions.iter().any(|s| s.session_type == SessionType::LiveDemo));

    // Verify all have hosts and session IDs
    assert!(sessions.iter().all(|s| !s.host.is_empty()));
    assert!(sessions.iter().all(|s| !s.session_id.is_empty()));
}

#[test]
fn test_mock_reviews() {
    let reviews = create_mock_reviews();
    assert_eq!(reviews.len(), 5);

    // Verify different review statuses
    assert!(reviews.iter().any(|r| r.status == ReviewStatus::InReview));
    assert!(reviews.iter().any(|r| r.status == ReviewStatus::Approved));
    assert!(reviews.iter().any(|r| r.status == ReviewStatus::Merged));
    assert!(reviews.iter().any(|r| r.status == ReviewStatus::ChangesRequested));
    assert!(reviews.iter().any(|r| r.status == ReviewStatus::PendingReview));

    // Verify all have required fields
    assert!(reviews.iter().all(|r| !r.title.is_empty()));
    assert!(reviews.iter().all(|r| !r.author.is_empty()));
    assert!(reviews.iter().all(|r| !r.review_id.is_empty()));
}

#[test]
fn test_mock_articles() {
    let articles = create_mock_articles();
    assert_eq!(articles.len(), 6);

    // Verify different categories
    assert!(articles.iter().any(|a| a.category == KnowledgeCategory::GettingStarted));
    assert!(articles.iter().any(|a| a.category == KnowledgeCategory::BestPractices));
    assert!(articles.iter().any(|a| a.category == KnowledgeCategory::Troubleshooting));
    assert!(articles.iter().any(|a| a.category == KnowledgeCategory::AdvancedTopics));
    assert!(articles.iter().any(|a| a.category == KnowledgeCategory::Tutorials));

    // Verify different difficulty levels
    assert!(articles.iter().any(|a| a.difficulty == DifficultyLevel::Beginner));
    assert!(articles.iter().any(|a| a.difficulty == DifficultyLevel::Intermediate));
    assert!(articles.iter().any(|a| a.difficulty == DifficultyLevel::Advanced));

    // Verify all have required fields
    assert!(articles.iter().all(|a| !a.title.is_empty()));
    assert!(articles.iter().all(|a| !a.author.is_empty()));
    assert!(articles.iter().all(|a| a.read_time_minutes > 0));
}

// ============================================================================
// CollabFeaturesState Tests
// ============================================================================

#[test]
fn test_collab_features_state_new() {
    let state = CollabFeaturesState::new();
    assert_eq!(state.current_section, CollabSection::TeamWorkspace);
    assert_eq!(state.selected_item, 0);
}

#[test]
fn test_collab_features_state_default() {
    let state = CollabFeaturesState::default();
    assert_eq!(state.current_section, CollabSection::TeamWorkspace);
    assert_eq!(state.selected_item, 0);
}

#[test]
fn test_state_next_section() {
    let mut state = CollabFeaturesState::new();

    assert_eq!(state.current_section, CollabSection::TeamWorkspace);
    state.next_section();
    assert_eq!(state.current_section, CollabSection::LiveSessions);
    assert_eq!(state.selected_item, 0);

    state.next_section();
    assert_eq!(state.current_section, CollabSection::CodeReviews);
    assert_eq!(state.selected_item, 0);

    state.next_section();
    assert_eq!(state.current_section, CollabSection::KnowledgeBase);
    assert_eq!(state.selected_item, 0);

    // Wrap around
    state.next_section();
    assert_eq!(state.current_section, CollabSection::TeamWorkspace);
    assert_eq!(state.selected_item, 0);
}

#[test]
fn test_state_previous_section() {
    let mut state = CollabFeaturesState::new();

    assert_eq!(state.current_section, CollabSection::TeamWorkspace);

    // Wrap around to last section
    state.previous_section();
    assert_eq!(state.current_section, CollabSection::KnowledgeBase);
    assert_eq!(state.selected_item, 0);

    state.previous_section();
    assert_eq!(state.current_section, CollabSection::CodeReviews);
    assert_eq!(state.selected_item, 0);

    state.previous_section();
    assert_eq!(state.current_section, CollabSection::LiveSessions);
    assert_eq!(state.selected_item, 0);

    state.previous_section();
    assert_eq!(state.current_section, CollabSection::TeamWorkspace);
    assert_eq!(state.selected_item, 0);
}

#[test]
fn test_state_select_next() {
    let mut state = CollabFeaturesState::new();
    let item_count = state.get_item_count();

    assert_eq!(state.selected_item, 0);

    state.select_next();
    assert_eq!(state.selected_item, 1);

    state.select_next();
    assert_eq!(state.selected_item, 2);

    // Move to last item
    for _ in 3..item_count {
        state.select_next();
    }
    assert_eq!(state.selected_item, item_count - 1);

    // Wrap around to first item
    state.select_next();
    assert_eq!(state.selected_item, 0);
}

#[test]
fn test_state_select_previous() {
    let mut state = CollabFeaturesState::new();
    let item_count = state.get_item_count();

    assert_eq!(state.selected_item, 0);

    // Wrap around to last item
    state.select_previous();
    assert_eq!(state.selected_item, item_count - 1);

    state.select_previous();
    assert_eq!(state.selected_item, item_count - 2);
}

#[test]
fn test_state_get_item_count() {
    let mut state = CollabFeaturesState::new();

    // Team workspace has 6 members
    assert_eq!(state.get_item_count(), 6);

    state.next_section();
    // Live sessions has 4 sessions
    assert_eq!(state.get_item_count(), 4);

    state.next_section();
    // Code reviews has 5 reviews
    assert_eq!(state.get_item_count(), 5);

    state.next_section();
    // Knowledge base has 6 articles
    assert_eq!(state.get_item_count(), 6);
}

#[test]
fn test_state_refresh() {
    let mut state = CollabFeaturesState::new();
    let initial_time = state.last_refresh;

    std::thread::sleep(std::time::Duration::from_millis(10));

    state.refresh();
    assert!(state.last_refresh > initial_time);
}

#[test]
fn test_state_data_updates_on_section_change() {
    let mut state = CollabFeaturesState::new();

    // Initial section should be TeamWorkspace
    match &state.data {
        CollabData::TeamWorkspace(_) => {},
        _ => panic!("Expected TeamWorkspace data"),
    }

    state.next_section();
    match &state.data {
        CollabData::LiveSessions(_) => {},
        _ => panic!("Expected LiveSessions data"),
    }

    state.next_section();
    match &state.data {
        CollabData::CodeReviews(_) => {},
        _ => panic!("Expected CodeReviews data"),
    }

    state.next_section();
    match &state.data {
        CollabData::KnowledgeBase(_) => {},
        _ => panic!("Expected KnowledgeBase data"),
    }
}

// ============================================================================
// CollabFeaturesView Tests
// ============================================================================

#[test]
fn test_view_creation() {
    let view = create_test_view();
    assert_eq!(view.title(), "Collaborative Features");
    assert_eq!(view.state.current_section, CollabSection::TeamWorkspace);
}

#[test]
fn test_view_auto_refresh() {
    let view = create_test_view();
    assert!(view.auto_refresh().is_some());
}

#[test]
fn test_view_help_text() {
    let view = create_test_view();
    let help = view.help_text();
    assert!(!help.is_empty());
    assert!(help.iter().any(|(key, _)| *key == "Tab"));
    assert!(help.iter().any(|(key, _)| *key == "r"));
    assert!(help.iter().any(|(key, _)| *key == "q"));
}

#[tokio::test]
async fn test_view_update() {
    let mut view = create_test_view();
    let mut app_state = AppState::default();
    let result = view.update(&mut app_state).await;
    assert!(result.is_ok());
}

#[tokio::test]
async fn test_handle_quit_key() {
    let mut view = create_test_view();
    let mut app_state = AppState::default();
    let key = KeyEvent::from(KeyCode::Char('q'));
    let action = view.handle_key(key, &mut app_state).await.unwrap();
    assert!(matches!(action, ViewAction::PopView));
}

#[tokio::test]
async fn test_handle_ctrl_c_key() {
    let mut view = create_test_view();
    let mut app_state = AppState::default();
    let key = KeyEvent::new(
        KeyCode::Char('c'),
        crossterm::event::KeyModifiers::CONTROL,
    );
    let action = view.handle_key(key, &mut app_state).await.unwrap();
    assert!(matches!(action, ViewAction::Quit));
}

#[tokio::test]
async fn test_handle_tab_key() {
    let mut view = create_test_view();
    let mut app_state = AppState::default();

    assert_eq!(view.state.current_section, CollabSection::TeamWorkspace);

    let key = KeyEvent::from(KeyCode::Tab);
    let _ = view.handle_key(key, &mut app_state).await;

    assert_eq!(view.state.current_section, CollabSection::LiveSessions);
}

#[tokio::test]
async fn test_handle_backtab_key() {
    let mut view = create_test_view();
    let mut app_state = AppState::default();

    assert_eq!(view.state.current_section, CollabSection::TeamWorkspace);

    let key = KeyEvent::from(KeyCode::BackTab);
    let _ = view.handle_key(key, &mut app_state).await;

    assert_eq!(view.state.current_section, CollabSection::KnowledgeBase);
}

#[tokio::test]
async fn test_handle_up_down_keys() {
    let mut view = create_test_view();
    let mut app_state = AppState::default();

    assert_eq!(view.state.selected_item, 0);

    let key_down = KeyEvent::from(KeyCode::Down);
    let _ = view.handle_key(key_down, &mut app_state).await;
    assert_eq!(view.state.selected_item, 1);

    let key_up = KeyEvent::from(KeyCode::Up);
    let _ = view.handle_key(key_up, &mut app_state).await;
    assert_eq!(view.state.selected_item, 0);
}

#[tokio::test]
async fn test_handle_j_k_keys() {
    let mut view = create_test_view();
    let mut app_state = AppState::default();

    assert_eq!(view.state.selected_item, 0);

    let key_j = KeyEvent::from(KeyCode::Char('j'));
    let _ = view.handle_key(key_j, &mut app_state).await;
    assert_eq!(view.state.selected_item, 1);

    let key_k = KeyEvent::from(KeyCode::Char('k'));
    let _ = view.handle_key(key_k, &mut app_state).await;
    assert_eq!(view.state.selected_item, 0);
}

#[tokio::test]
async fn test_handle_refresh_keys() {
    let mut view = create_test_view();
    let mut app_state = AppState::default();

    let initial_time = view.state.last_refresh;
    std::thread::sleep(std::time::Duration::from_millis(10));

    let key = KeyEvent::from(KeyCode::Char('r'));
    let _ = view.handle_key(key, &mut app_state).await;
    assert!(view.state.last_refresh > initial_time);

    let initial_time = view.state.last_refresh;
    std::thread::sleep(std::time::Duration::from_millis(10));

    let key = KeyEvent::from(KeyCode::Char('R'));
    let _ = view.handle_key(key, &mut app_state).await;
    assert!(view.state.last_refresh > initial_time);
}

#[tokio::test]
async fn test_handle_help_key() {
    let mut view = create_test_view();
    let mut app_state = AppState::default();
    let key = KeyEvent::from(KeyCode::Char('?'));
    let action = view.handle_key(key, &mut app_state).await.unwrap();
    assert!(matches!(action, ViewAction::ShowHelp));
}

#[test]
fn test_view_renders_without_error() {
    let mut view = create_test_view();
    let mut terminal = create_test_terminal();
    let app_state = AppState::default();

    terminal
        .draw(|f| {
            let area = ratatui::layout::Rect::new(0, 0, 80, 24);
            view.render(f, area, &app_state);
        })
        .unwrap();
}

#[test]
fn test_all_sections_render() {
    let mut view = create_test_view();
    let mut terminal = create_test_terminal();
    let app_state = AppState::default();

    for section in CollabSection::all() {
        view.state.current_section = section;
        view.state.data = match section {
            CollabSection::TeamWorkspace => CollabData::TeamWorkspace(create_mock_team_members()),
            CollabSection::LiveSessions => CollabData::LiveSessions(create_mock_sessions()),
            CollabSection::CodeReviews => CollabData::CodeReviews(create_mock_reviews()),
            CollabSection::KnowledgeBase => CollabData::KnowledgeBase(create_mock_articles()),
        };

        terminal
            .draw(|f| {
                let area = ratatui::layout::Rect::new(0, 0, 80, 24);
                view.render(f, area, &app_state);
            })
            .unwrap();
    }
}
