// Collaborative Development Features Types - Phase 1: Simplified Implementation with Mock Data
// TODO(Issue #34 Phase 2): Add real workspace management, live collaboration engine, and VCS integration

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::time::Instant;

/// Represents the different collaborative feature sections
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CollabSection {
    TeamWorkspace,
    LiveSessions,
    CodeReviews,
    KnowledgeBase,
}

impl CollabSection {
    /// Get all available sections
    pub fn all() -> Vec<Self> {
        vec![
            Self::TeamWorkspace,
            Self::LiveSessions,
            Self::CodeReviews,
            Self::KnowledgeBase,
        ]
    }

    /// Get the name of the section
    pub fn name(&self) -> &'static str {
        match self {
            Self::TeamWorkspace => "Team Workspace",
            Self::LiveSessions => "Live Sessions",
            Self::CodeReviews => "Code Reviews",
            Self::KnowledgeBase => "Knowledge Base",
        }
    }

    /// Get the description of the section
    pub fn description(&self) -> &'static str {
        match self {
            Self::TeamWorkspace => "View team members, roles, and workspace information",
            Self::LiveSessions => "Active collaboration sessions with team members",
            Self::CodeReviews => "Code reviews with automated analysis and team feedback",
            Self::KnowledgeBase => "Team documentation, best practices, and learning resources",
        }
    }

    /// Get the icon for the section
    pub fn icon(&self) -> &'static str {
        match self {
            Self::TeamWorkspace => "👥",
            Self::LiveSessions => "🔴",
            Self::CodeReviews => "📝",
            Self::KnowledgeBase => "📚",
        }
    }
}

/// Team member role in the workspace
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TeamRole {
    Owner,
    Admin,
    Developer,
    Viewer,
    Guest,
}

impl TeamRole {
    /// Get the label for the role
    pub fn label(&self) -> &'static str {
        match self {
            Self::Owner => "Owner",
            Self::Admin => "Admin",
            Self::Developer => "Developer",
            Self::Viewer => "Viewer",
            Self::Guest => "Guest",
        }
    }

    /// Get the color indicator for the role
    pub fn color_indicator(&self) -> &'static str {
        match self {
            Self::Owner => "🟣",
            Self::Admin => "🔵",
            Self::Developer => "🟢",
            Self::Viewer => "🟡",
            Self::Guest => "⚪",
        }
    }
}

/// Presence status for team members
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PresenceStatus {
    Online,
    Away,
    Busy,
    Offline,
}

impl PresenceStatus {
    /// Get the indicator for the presence status
    pub fn indicator(&self) -> &'static str {
        match self {
            Self::Online => "🟢",
            Self::Away => "🟡",
            Self::Busy => "🔴",
            Self::Offline => "⚫",
        }
    }

    /// Get the label for the presence status
    pub fn label(&self) -> &'static str {
        match self {
            Self::Online => "Online",
            Self::Away => "Away",
            Self::Busy => "Busy",
            Self::Offline => "Offline",
        }
    }
}

/// Represents a team member
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TeamMember {
    pub username: String,
    pub email: String,
    pub role: TeamRole,
    pub presence: PresenceStatus,
    pub joined_at: DateTime<Utc>,
    pub last_active: DateTime<Utc>,
    pub contributions: usize,
}

/// Type of collaboration session
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SessionType {
    CodeReview,
    PairProgramming,
    TeamDebug,
    LiveDemo,
    PlanningSession,
}

impl SessionType {
    /// Get the label for the session type
    pub fn label(&self) -> &'static str {
        match self {
            Self::CodeReview => "Code Review",
            Self::PairProgramming => "Pair Programming",
            Self::TeamDebug => "Team Debug",
            Self::LiveDemo => "Live Demo",
            Self::PlanningSession => "Planning",
        }
    }

    /// Get the icon for the session type
    pub fn icon(&self) -> &'static str {
        match self {
            Self::CodeReview => "📝",
            Self::PairProgramming => "👥",
            Self::TeamDebug => "🐛",
            Self::LiveDemo => "🎥",
            Self::PlanningSession => "📋",
        }
    }
}

/// Represents an active collaboration session
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CollabSession {
    pub session_id: String,
    pub session_type: SessionType,
    pub host: String,
    pub participants: Vec<String>,
    pub started_at: DateTime<Utc>,
    pub activity_count: usize,
    pub is_active: bool,
}

/// Code review status
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewStatus {
    Draft,
    PendingReview,
    InReview,
    ChangesRequested,
    Approved,
    Merged,
    Closed,
}

impl ReviewStatus {
    /// Get the label for the review status
    pub fn label(&self) -> &'static str {
        match self {
            Self::Draft => "Draft",
            Self::PendingReview => "Pending",
            Self::InReview => "In Review",
            Self::ChangesRequested => "Changes Requested",
            Self::Approved => "Approved",
            Self::Merged => "Merged",
            Self::Closed => "Closed",
        }
    }

    /// Get the color indicator for the status
    pub fn color_indicator(&self) -> &'static str {
        match self {
            Self::Draft => "⚪",
            Self::PendingReview => "🟡",
            Self::InReview => "🔵",
            Self::ChangesRequested => "🟠",
            Self::Approved => "🟢",
            Self::Merged => "🟣",
            Self::Closed => "⚫",
        }
    }
}

/// Represents a code review
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CodeReview {
    pub review_id: String,
    pub title: String,
    pub author: String,
    pub reviewers: Vec<String>,
    pub status: ReviewStatus,
    pub files_changed: usize,
    pub comments: usize,
    pub approvals: usize,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
}

/// Knowledge article difficulty level
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DifficultyLevel {
    Beginner,
    Intermediate,
    Advanced,
    Expert,
}

impl DifficultyLevel {
    /// Get the label for the difficulty level
    pub fn label(&self) -> &'static str {
        match self {
            Self::Beginner => "Beginner",
            Self::Intermediate => "Intermediate",
            Self::Advanced => "Advanced",
            Self::Expert => "Expert",
        }
    }

    /// Get the icon for the difficulty level
    pub fn icon(&self) -> &'static str {
        match self {
            Self::Beginner => "🌱",
            Self::Intermediate => "🌿",
            Self::Advanced => "🌳",
            Self::Expert => "🎓",
        }
    }
}

/// Knowledge article category
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum KnowledgeCategory {
    GettingStarted,
    BestPractices,
    Troubleshooting,
    AdvancedTopics,
    Tutorials,
    ReferenceGuide,
}

impl KnowledgeCategory {
    /// Get the label for the category
    pub fn label(&self) -> &'static str {
        match self {
            Self::GettingStarted => "Getting Started",
            Self::BestPractices => "Best Practices",
            Self::Troubleshooting => "Troubleshooting",
            Self::AdvancedTopics => "Advanced Topics",
            Self::Tutorials => "Tutorials",
            Self::ReferenceGuide => "Reference Guide",
        }
    }

    /// Get the icon for the category
    pub fn icon(&self) -> &'static str {
        match self {
            Self::GettingStarted => "🚀",
            Self::BestPractices => "⭐",
            Self::Troubleshooting => "🔧",
            Self::AdvancedTopics => "🎓",
            Self::Tutorials => "📖",
            Self::ReferenceGuide => "📚",
        }
    }
}

/// Represents a knowledge base article
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KnowledgeArticle {
    pub article_id: String,
    pub title: String,
    pub author: String,
    pub category: KnowledgeCategory,
    pub difficulty: DifficultyLevel,
    pub read_time_minutes: usize,
    pub views: usize,
    pub likes: usize,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
}

/// Collaborative features data container
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CollabData {
    TeamWorkspace(Vec<TeamMember>),
    LiveSessions(Vec<CollabSession>),
    CodeReviews(Vec<CodeReview>),
    KnowledgeBase(Vec<KnowledgeArticle>),
}

/// State management for collaborative features view
#[derive(Debug, Clone)]
pub struct CollabFeaturesState {
    pub current_section: CollabSection,
    pub selected_item: usize,
    pub data: CollabData,
    pub last_refresh: Instant,
}

impl CollabFeaturesState {
    /// Create a new collaborative features state
    pub fn new() -> Self {
        let current_section = CollabSection::TeamWorkspace;
        Self {
            current_section,
            selected_item: 0,
            data: Self::create_initial_data(&current_section),
            last_refresh: Instant::now(),
        }
    }

    /// Create initial mock data for a section
    fn create_initial_data(section: &CollabSection) -> CollabData {
        match section {
            CollabSection::TeamWorkspace => CollabData::TeamWorkspace(create_mock_team_members()),
            CollabSection::LiveSessions => CollabData::LiveSessions(create_mock_sessions()),
            CollabSection::CodeReviews => CollabData::CodeReviews(create_mock_reviews()),
            CollabSection::KnowledgeBase => CollabData::KnowledgeBase(create_mock_articles()),
        }
    }

    /// Navigate to the next section
    pub fn next_section(&mut self) {
        let sections = CollabSection::all();
        let current_index = sections
            .iter()
            .position(|s| s == &self.current_section)
            .unwrap_or(0);
        let next_index = (current_index + 1) % sections.len();
        self.current_section = sections[next_index];
        self.selected_item = 0;
        self.data = Self::create_initial_data(&self.current_section);
    }

    /// Navigate to the previous section
    pub fn previous_section(&mut self) {
        let sections = CollabSection::all();
        let current_index = sections
            .iter()
            .position(|s| s == &self.current_section)
            .unwrap_or(0);
        let prev_index = if current_index == 0 {
            sections.len() - 1
        } else {
            current_index - 1
        };
        self.current_section = sections[prev_index];
        self.selected_item = 0;
        self.data = Self::create_initial_data(&self.current_section);
    }

    /// Select the next item in the current section
    pub fn select_next(&mut self) {
        let item_count = self.get_item_count();
        if item_count > 0 {
            self.selected_item = (self.selected_item + 1) % item_count;
        }
    }

    /// Select the previous item in the current section
    pub fn select_previous(&mut self) {
        let item_count = self.get_item_count();
        if item_count > 0 {
            if self.selected_item == 0 {
                self.selected_item = item_count - 1;
            } else {
                self.selected_item -= 1;
            }
        }
    }

    /// Get the number of items in the current section
    pub fn get_item_count(&self) -> usize {
        match &self.data {
            CollabData::TeamWorkspace(members) => members.len(),
            CollabData::LiveSessions(sessions) => sessions.len(),
            CollabData::CodeReviews(reviews) => reviews.len(),
            CollabData::KnowledgeBase(articles) => articles.len(),
        }
    }

    /// Refresh the data for the current section
    pub fn refresh(&mut self) {
        self.data = Self::create_initial_data(&self.current_section);
        self.last_refresh = Instant::now();
    }
}

impl Default for CollabFeaturesState {
    fn default() -> Self {
        Self::new()
    }
}

/// Create mock team members for demonstration
pub fn create_mock_team_members() -> Vec<TeamMember> {
    let now = Utc::now();
    vec![
        TeamMember {
            username: "alice_dev".to_string(),
            email: "alice@example.com".to_string(),
            role: TeamRole::Owner,
            presence: PresenceStatus::Online,
            joined_at: now - chrono::Duration::days(180),
            last_active: now,
            contributions: 342,
        },
        TeamMember {
            username: "bob_admin".to_string(),
            email: "bob@example.com".to_string(),
            role: TeamRole::Admin,
            presence: PresenceStatus::Online,
            joined_at: now - chrono::Duration::days(150),
            last_active: now - chrono::Duration::minutes(5),
            contributions: 256,
        },
        TeamMember {
            username: "charlie_code".to_string(),
            email: "charlie@example.com".to_string(),
            role: TeamRole::Developer,
            presence: PresenceStatus::Busy,
            joined_at: now - chrono::Duration::days(90),
            last_active: now - chrono::Duration::minutes(2),
            contributions: 187,
        },
        TeamMember {
            username: "diana_dev".to_string(),
            email: "diana@example.com".to_string(),
            role: TeamRole::Developer,
            presence: PresenceStatus::Online,
            joined_at: now - chrono::Duration::days(60),
            last_active: now - chrono::Duration::minutes(1),
            contributions: 143,
        },
        TeamMember {
            username: "eve_review".to_string(),
            email: "eve@example.com".to_string(),
            role: TeamRole::Viewer,
            presence: PresenceStatus::Away,
            joined_at: now - chrono::Duration::days(30),
            last_active: now - chrono::Duration::hours(2),
            contributions: 42,
        },
        TeamMember {
            username: "frank_guest".to_string(),
            email: "frank@example.com".to_string(),
            role: TeamRole::Guest,
            presence: PresenceStatus::Offline,
            joined_at: now - chrono::Duration::days(7),
            last_active: now - chrono::Duration::days(1),
            contributions: 8,
        },
    ]
}

/// Create mock collaboration sessions for demonstration
pub fn create_mock_sessions() -> Vec<CollabSession> {
    let now = Utc::now();
    vec![
        CollabSession {
            session_id: "session-001".to_string(),
            session_type: SessionType::CodeReview,
            host: "alice_dev".to_string(),
            participants: vec!["bob_admin".to_string(), "charlie_code".to_string()],
            started_at: now - chrono::Duration::minutes(45),
            activity_count: 23,
            is_active: true,
        },
        CollabSession {
            session_id: "session-002".to_string(),
            session_type: SessionType::PairProgramming,
            host: "charlie_code".to_string(),
            participants: vec!["diana_dev".to_string()],
            started_at: now - chrono::Duration::minutes(120),
            activity_count: 67,
            is_active: true,
        },
        CollabSession {
            session_id: "session-003".to_string(),
            session_type: SessionType::TeamDebug,
            host: "bob_admin".to_string(),
            participants: vec![
                "alice_dev".to_string(),
                "charlie_code".to_string(),
                "diana_dev".to_string(),
            ],
            started_at: now - chrono::Duration::minutes(30),
            activity_count: 15,
            is_active: true,
        },
        CollabSession {
            session_id: "session-004".to_string(),
            session_type: SessionType::LiveDemo,
            host: "alice_dev".to_string(),
            participants: vec![
                "bob_admin".to_string(),
                "eve_review".to_string(),
                "frank_guest".to_string(),
            ],
            started_at: now - chrono::Duration::hours(2),
            activity_count: 12,
            is_active: false,
        },
    ]
}

/// Create mock code reviews for demonstration
pub fn create_mock_reviews() -> Vec<CodeReview> {
    let now = Utc::now();
    vec![
        CodeReview {
            review_id: "PR-123".to_string(),
            title: "Add real-time collaboration engine".to_string(),
            author: "charlie_code".to_string(),
            reviewers: vec!["alice_dev".to_string(), "bob_admin".to_string()],
            status: ReviewStatus::InReview,
            files_changed: 8,
            comments: 12,
            approvals: 1,
            created_at: now - chrono::Duration::hours(6),
            updated_at: now - chrono::Duration::minutes(15),
        },
        CodeReview {
            review_id: "PR-122".to_string(),
            title: "Implement knowledge base search".to_string(),
            author: "diana_dev".to_string(),
            reviewers: vec!["alice_dev".to_string()],
            status: ReviewStatus::Approved,
            files_changed: 5,
            comments: 8,
            approvals: 2,
            created_at: now - chrono::Duration::days(1),
            updated_at: now - chrono::Duration::hours(2),
        },
        CodeReview {
            review_id: "PR-121".to_string(),
            title: "Fix workspace permission bug".to_string(),
            author: "bob_admin".to_string(),
            reviewers: vec!["alice_dev".to_string(), "charlie_code".to_string()],
            status: ReviewStatus::Merged,
            files_changed: 3,
            comments: 5,
            approvals: 2,
            created_at: now - chrono::Duration::days(2),
            updated_at: now - chrono::Duration::hours(12),
        },
        CodeReview {
            review_id: "PR-120".to_string(),
            title: "Add team member invitation system".to_string(),
            author: "charlie_code".to_string(),
            reviewers: vec!["alice_dev".to_string(), "diana_dev".to_string()],
            status: ReviewStatus::ChangesRequested,
            files_changed: 12,
            comments: 18,
            approvals: 0,
            created_at: now - chrono::Duration::hours(8),
            updated_at: now - chrono::Duration::minutes(30),
        },
        CodeReview {
            review_id: "PR-119".to_string(),
            title: "Update documentation for API changes".to_string(),
            author: "eve_review".to_string(),
            reviewers: vec!["alice_dev".to_string()],
            status: ReviewStatus::PendingReview,
            files_changed: 6,
            comments: 2,
            approvals: 0,
            created_at: now - chrono::Duration::hours(3),
            updated_at: now - chrono::Duration::hours(3),
        },
    ]
}

/// Create mock knowledge articles for demonstration
pub fn create_mock_articles() -> Vec<KnowledgeArticle> {
    let now = Utc::now();
    vec![
        KnowledgeArticle {
            article_id: "KB-001".to_string(),
            title: "Getting Started with Dora Collaborative Features".to_string(),
            author: "alice_dev".to_string(),
            category: KnowledgeCategory::GettingStarted,
            difficulty: DifficultyLevel::Beginner,
            read_time_minutes: 5,
            views: 342,
            likes: 45,
            created_at: now - chrono::Duration::days(30),
            updated_at: now - chrono::Duration::days(5),
        },
        KnowledgeArticle {
            article_id: "KB-002".to_string(),
            title: "Code Review Best Practices".to_string(),
            author: "bob_admin".to_string(),
            category: KnowledgeCategory::BestPractices,
            difficulty: DifficultyLevel::Intermediate,
            read_time_minutes: 8,
            views: 289,
            likes: 38,
            created_at: now - chrono::Duration::days(25),
            updated_at: now - chrono::Duration::days(10),
        },
        KnowledgeArticle {
            article_id: "KB-003".to_string(),
            title: "Troubleshooting Real-Time Sync Issues".to_string(),
            author: "charlie_code".to_string(),
            category: KnowledgeCategory::Troubleshooting,
            difficulty: DifficultyLevel::Intermediate,
            read_time_minutes: 12,
            views: 156,
            likes: 22,
            created_at: now - chrono::Duration::days(20),
            updated_at: now - chrono::Duration::days(3),
        },
        KnowledgeArticle {
            article_id: "KB-004".to_string(),
            title: "Advanced Conflict Resolution Strategies".to_string(),
            author: "alice_dev".to_string(),
            category: KnowledgeCategory::AdvancedTopics,
            difficulty: DifficultyLevel::Advanced,
            read_time_minutes: 15,
            views: 98,
            likes: 18,
            created_at: now - chrono::Duration::days(15),
            updated_at: now - chrono::Duration::days(2),
        },
        KnowledgeArticle {
            article_id: "KB-005".to_string(),
            title: "Building a Dataflow Pipeline Tutorial".to_string(),
            author: "diana_dev".to_string(),
            category: KnowledgeCategory::Tutorials,
            difficulty: DifficultyLevel::Beginner,
            read_time_minutes: 20,
            views: 412,
            likes: 67,
            created_at: now - chrono::Duration::days(45),
            updated_at: now - chrono::Duration::days(15),
        },
        KnowledgeArticle {
            article_id: "KB-006".to_string(),
            title: "Workspace Management API Reference".to_string(),
            author: "bob_admin".to_string(),
            category: KnowledgeCategory::ReferenceGuide,
            difficulty: DifficultyLevel::Expert,
            read_time_minutes: 25,
            views: 67,
            likes: 12,
            created_at: now - chrono::Duration::days(10),
            updated_at: now - chrono::Duration::days(1),
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_collab_section_all() {
        let sections = CollabSection::all();
        assert_eq!(sections.len(), 4);
        assert!(sections.contains(&CollabSection::TeamWorkspace));
        assert!(sections.contains(&CollabSection::LiveSessions));
        assert!(sections.contains(&CollabSection::CodeReviews));
        assert!(sections.contains(&CollabSection::KnowledgeBase));
    }

    #[test]
    fn test_collab_section_names() {
        assert_eq!(CollabSection::TeamWorkspace.name(), "Team Workspace");
        assert_eq!(CollabSection::LiveSessions.name(), "Live Sessions");
        assert_eq!(CollabSection::CodeReviews.name(), "Code Reviews");
        assert_eq!(CollabSection::KnowledgeBase.name(), "Knowledge Base");
    }

    #[test]
    fn test_team_role_labels() {
        assert_eq!(TeamRole::Owner.label(), "Owner");
        assert_eq!(TeamRole::Admin.label(), "Admin");
        assert_eq!(TeamRole::Developer.label(), "Developer");
        assert_eq!(TeamRole::Viewer.label(), "Viewer");
        assert_eq!(TeamRole::Guest.label(), "Guest");
    }

    #[test]
    fn test_presence_status_indicators() {
        assert_eq!(PresenceStatus::Online.indicator(), "🟢");
        assert_eq!(PresenceStatus::Away.indicator(), "🟡");
        assert_eq!(PresenceStatus::Busy.indicator(), "🔴");
        assert_eq!(PresenceStatus::Offline.indicator(), "⚫");
    }

    #[test]
    fn test_session_type_labels() {
        assert_eq!(SessionType::CodeReview.label(), "Code Review");
        assert_eq!(SessionType::PairProgramming.label(), "Pair Programming");
        assert_eq!(SessionType::TeamDebug.label(), "Team Debug");
        assert_eq!(SessionType::LiveDemo.label(), "Live Demo");
        assert_eq!(SessionType::PlanningSession.label(), "Planning");
    }

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
    fn test_difficulty_level_labels() {
        assert_eq!(DifficultyLevel::Beginner.label(), "Beginner");
        assert_eq!(DifficultyLevel::Intermediate.label(), "Intermediate");
        assert_eq!(DifficultyLevel::Advanced.label(), "Advanced");
        assert_eq!(DifficultyLevel::Expert.label(), "Expert");
    }

    #[test]
    fn test_knowledge_category_labels() {
        assert_eq!(KnowledgeCategory::GettingStarted.label(), "Getting Started");
        assert_eq!(KnowledgeCategory::BestPractices.label(), "Best Practices");
        assert_eq!(
            KnowledgeCategory::Troubleshooting.label(),
            "Troubleshooting"
        );
        assert_eq!(KnowledgeCategory::AdvancedTopics.label(), "Advanced Topics");
        assert_eq!(KnowledgeCategory::Tutorials.label(), "Tutorials");
        assert_eq!(KnowledgeCategory::ReferenceGuide.label(), "Reference Guide");
    }

    #[test]
    fn test_collab_features_state_new() {
        let state = CollabFeaturesState::new();
        assert_eq!(state.current_section, CollabSection::TeamWorkspace);
        assert_eq!(state.selected_item, 0);
    }

    #[test]
    fn test_collab_features_state_next_section() {
        let mut state = CollabFeaturesState::new();
        assert_eq!(state.current_section, CollabSection::TeamWorkspace);

        state.next_section();
        assert_eq!(state.current_section, CollabSection::LiveSessions);
        assert_eq!(state.selected_item, 0);

        state.next_section();
        assert_eq!(state.current_section, CollabSection::CodeReviews);

        state.next_section();
        assert_eq!(state.current_section, CollabSection::KnowledgeBase);

        state.next_section();
        assert_eq!(state.current_section, CollabSection::TeamWorkspace);
    }

    #[test]
    fn test_collab_features_state_previous_section() {
        let mut state = CollabFeaturesState::new();
        assert_eq!(state.current_section, CollabSection::TeamWorkspace);

        state.previous_section();
        assert_eq!(state.current_section, CollabSection::KnowledgeBase);
        assert_eq!(state.selected_item, 0);

        state.previous_section();
        assert_eq!(state.current_section, CollabSection::CodeReviews);

        state.previous_section();
        assert_eq!(state.current_section, CollabSection::LiveSessions);

        state.previous_section();
        assert_eq!(state.current_section, CollabSection::TeamWorkspace);
    }

    #[test]
    fn test_collab_features_state_select_next() {
        let mut state = CollabFeaturesState::new();
        let item_count = state.get_item_count();

        state.select_next();
        assert_eq!(state.selected_item, 1);

        state.select_next();
        assert_eq!(state.selected_item, 2);

        // Move to last item
        for _ in 3..item_count {
            state.select_next();
        }
        assert_eq!(state.selected_item, item_count - 1);

        // Should wrap around
        state.select_next();
        assert_eq!(state.selected_item, 0);
    }

    #[test]
    fn test_collab_features_state_select_previous() {
        let mut state = CollabFeaturesState::new();
        let item_count = state.get_item_count();

        // At first item, should wrap to last
        state.select_previous();
        assert_eq!(state.selected_item, item_count - 1);

        state.select_previous();
        assert_eq!(state.selected_item, item_count - 2);
    }

    #[test]
    fn test_mock_team_members() {
        let members = create_mock_team_members();
        assert_eq!(members.len(), 6);
        assert!(members.iter().any(|m| m.role == TeamRole::Owner));
        assert!(members.iter().any(|m| m.role == TeamRole::Admin));
        assert!(members.iter().any(|m| m.role == TeamRole::Developer));
        assert!(members.iter().any(|m| m.role == TeamRole::Viewer));
        assert!(members.iter().any(|m| m.role == TeamRole::Guest));
    }

    #[test]
    fn test_mock_sessions() {
        let sessions = create_mock_sessions();
        assert_eq!(sessions.len(), 4);
        assert!(
            sessions
                .iter()
                .any(|s| s.session_type == SessionType::CodeReview)
        );
        assert!(
            sessions
                .iter()
                .any(|s| s.session_type == SessionType::PairProgramming)
        );
        assert!(
            sessions
                .iter()
                .any(|s| s.session_type == SessionType::TeamDebug)
        );
        assert!(
            sessions
                .iter()
                .any(|s| s.session_type == SessionType::LiveDemo)
        );
    }

    #[test]
    fn test_mock_reviews() {
        let reviews = create_mock_reviews();
        assert_eq!(reviews.len(), 5);
        assert!(reviews.iter().any(|r| r.status == ReviewStatus::InReview));
        assert!(reviews.iter().any(|r| r.status == ReviewStatus::Approved));
        assert!(reviews.iter().any(|r| r.status == ReviewStatus::Merged));
        assert!(
            reviews
                .iter()
                .any(|r| r.status == ReviewStatus::ChangesRequested)
        );
    }

    #[test]
    fn test_mock_articles() {
        let articles = create_mock_articles();
        assert_eq!(articles.len(), 6);
        assert!(
            articles
                .iter()
                .any(|a| a.category == KnowledgeCategory::GettingStarted)
        );
        assert!(
            articles
                .iter()
                .any(|a| a.category == KnowledgeCategory::BestPractices)
        );
        assert!(
            articles
                .iter()
                .any(|a| a.difficulty == DifficultyLevel::Beginner)
        );
        assert!(
            articles
                .iter()
                .any(|a| a.difficulty == DifficultyLevel::Advanced)
        );
    }

    #[test]
    fn test_state_get_item_count() {
        let mut state = CollabFeaturesState::new();

        // Team workspace should have 6 members
        assert_eq!(state.get_item_count(), 6);

        state.next_section();
        // Live sessions should have 4 sessions
        assert_eq!(state.get_item_count(), 4);

        state.next_section();
        // Code reviews should have 5 reviews
        assert_eq!(state.get_item_count(), 5);

        state.next_section();
        // Knowledge base should have 6 articles
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
}
