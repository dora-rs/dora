// Help Content Manager for Issue #21
// Intelligent help content delivery and personalization

use super::types::*;
use chrono::{Duration, Utc};
use eyre::Result;
use std::collections::HashMap;

// ============================================================================
// Help Content Manager
// ============================================================================

/// Manages help content delivery with personalization
#[derive(Debug)]
pub struct HelpContentManager {
    content_index: HelpIndex,
    search_engine: HelpSearchEngine,
    personalization_engine: HelpPersonalizationEngine,
}

impl HelpContentManager {
    pub fn new() -> Self {
        Self {
            content_index: HelpIndex::new(),
            search_engine: HelpSearchEngine::new(),
            personalization_engine: HelpPersonalizationEngine::new(),
        }
    }

    /// Get help content for a specific topic
    pub async fn get_topic_content(
        &self,
        topic: &str,
        user_expertise: UserExpertiseLevel,
        detail_level: DetailLevel,
    ) -> Result<HelpContent> {
        // Resolve topic from index
        let base_content = self.content_index.get_topic(topic).await?
            .unwrap_or_else(|| self.generate_default_topic_content(topic));

        // Apply user expertise filtering
        let filtered_content = self.filter_content_by_expertise(&base_content, user_expertise);

        // Apply detail level filtering
        let detailed_content = self.apply_detail_level(&filtered_content, detail_level);

        // Add personalized suggestions
        let personalized_content = self.personalization_engine
            .enhance_content(&detailed_content, user_expertise).await?;

        Ok(personalized_content)
    }

    /// Search help content
    pub async fn search_content(
        &self,
        query: &str,
        user_expertise: UserExpertiseLevel,
        detail_level: DetailLevel,
    ) -> Result<HelpContent> {
        // Perform semantic search
        let search_results = self.search_engine.search(query, user_expertise).await?;

        if search_results.is_empty() {
            return Ok(HelpContent::not_found(query));
        }

        // Combine relevant results
        let combined_content = self.combine_search_results(search_results, detail_level);

        Ok(combined_content)
    }

    /// Get contextual help based on system state
    pub async fn get_contextual_help(
        &self,
        system_context: &Option<SystemContext>,
        user_expertise: UserExpertiseLevel,
    ) -> Result<HelpContent> {
        let mut contextual_content = HelpContent::new("Contextual Help".to_string());
        contextual_content.title = "Context-Aware Help".to_string();
        contextual_content.summary = "Help tailored to your current system state".to_string();

        if let Some(context) = system_context {
            // Analyze current dataflows
            if !context.active_dataflows.is_empty() {
                let dataflow_help = self.generate_dataflow_contextual_help(&context.active_dataflows).await?;
                contextual_content.sections.extend(dataflow_help);
            }

            // Analyze recent errors
            if !context.recent_errors.is_empty() {
                let error_help = self.generate_error_contextual_help(&context.recent_errors).await?;
                contextual_content.sections.extend(error_help);
            }

            // Analyze system performance
            if let Some(performance_issues) = &context.performance_issues {
                let performance_help = self.generate_performance_help(performance_issues).await?;
                contextual_content.sections.extend(performance_help);
            }

            // Suggest next actions
            let suggestions = self.generate_contextual_suggestions(context, user_expertise).await?;
            contextual_content.sections.push(HelpSection {
                title: "Suggested Actions".to_string(),
                content: suggestions,
                section_type: SectionType::Usage,
                visibility_level: DetailLevel::Normal,
                interactive_elements: Vec::new(),
            });
        } else {
            contextual_content = self.get_overview_content(user_expertise).await?;
        }

        Ok(contextual_content)
    }

    /// Get overview of all help content
    pub async fn get_overview_content(&self, user_expertise: UserExpertiseLevel) -> Result<HelpContent> {
        let mut overview = HelpContent::new("overview".to_string());
        overview.title = "Dora CLI Help Overview".to_string();
        overview.summary = "Comprehensive guide to Dora CLI commands and features".to_string();

        // Core commands section
        overview.sections.push(HelpSection {
            title: "Core Commands".to_string(),
            content: "Essential commands for dataflow management:\n\
                      • dora ps - List active dataflows\n\
                      • dora start - Start a dataflow\n\
                      • dora stop - Stop a dataflow\n\
                      • dora logs - View dataflow logs\n\
                      • dora build - Build dataflow components".to_string(),
            section_type: SectionType::Overview,
            visibility_level: DetailLevel::Normal,
            interactive_elements: Vec::new(),
        });

        // Enhanced commands section
        if matches!(user_expertise, UserExpertiseLevel::Intermediate | UserExpertiseLevel::Expert) {
            overview.sections.push(HelpSection {
                title: "Enhanced Commands".to_string(),
                content: "Advanced commands with smart features:\n\
                          • dora inspect - Deep resource inspection\n\
                          • dora debug - Interactive debugging\n\
                          • dora analyze - Multi-modal analysis\n\
                          • dora monitor - Real-time monitoring".to_string(),
                section_type: SectionType::Advanced,
                visibility_level: DetailLevel::Detailed,
                interactive_elements: Vec::new(),
            });
        }

        // Getting started examples
        overview.examples.push(HelpExample {
            title: "Start your first dataflow".to_string(),
            description: "Launch a basic dataflow from a YAML file".to_string(),
            command: "dora start dataflow.yaml".to_string(),
            expected_output: Some("Dataflow started successfully".to_string()),
            explanation: "This command initializes and starts all nodes defined in the dataflow configuration".to_string(),
            difficulty: DifficultyLevel::Beginner,
            prerequisites: vec!["Valid dataflow YAML file".to_string()],
            runnable: true,
        });

        Ok(overview)
    }

    /// Get list of all available topics
    pub async fn get_topic_list(&self, user_expertise: UserExpertiseLevel) -> Result<HelpContent> {
        let mut topics_content = HelpContent::new("topics".to_string());
        topics_content.title = "Available Help Topics".to_string();
        topics_content.summary = "Browse all help topics organized by category".to_string();

        let topics_list = "Available topics:\n\
                           • commands - All CLI commands\n\
                           • dataflow - Dataflow management\n\
                           • configuration - System configuration\n\
                           • debugging - Debugging and troubleshooting\n\
                           • examples - Usage examples\n\
                           • tutorials - Interactive tutorials".to_string();

        topics_content.sections.push(HelpSection {
            title: "Topics".to_string(),
            content: topics_list,
            section_type: SectionType::Overview,
            visibility_level: DetailLevel::Normal,
            interactive_elements: Vec::new(),
        });

        Ok(topics_content)
    }

    // ========================================================================
    // Private Helper Methods
    // ========================================================================

    fn generate_default_topic_content(&self, topic: &str) -> HelpContent {
        let mut content = HelpContent::new(topic.to_string());
        content.title = format!("Help: {}", topic);
        content.summary = format!("Information about {}", topic);

        content.sections.push(HelpSection {
            title: "Description".to_string(),
            content: format!("Help content for {} is being prepared.", topic),
            section_type: SectionType::Overview,
            visibility_level: DetailLevel::Normal,
            interactive_elements: Vec::new(),
        });

        content
    }

    fn filter_content_by_expertise(&self, content: &HelpContent, expertise: UserExpertiseLevel) -> HelpContent {
        let mut filtered = content.clone();

        // Filter sections based on expertise
        filtered.sections.retain(|section| {
            match (expertise, &section.section_type) {
                (UserExpertiseLevel::Beginner, SectionType::Advanced) => false,
                (UserExpertiseLevel::Beginner, SectionType::Tutorial) => true,
                _ => true,
            }
        });

        // Filter examples by difficulty
        filtered.examples.retain(|example| {
            match (expertise, example.difficulty) {
                (UserExpertiseLevel::Beginner, DifficultyLevel::Advanced) => false,
                (UserExpertiseLevel::Beginner, DifficultyLevel::Expert) => false,
                _ => true,
            }
        });

        filtered
    }

    fn apply_detail_level(&self, content: &HelpContent, detail_level: DetailLevel) -> HelpContent {
        let mut detailed = content.clone();

        // Filter sections by visibility level
        detailed.sections.retain(|section| {
            match detail_level {
                DetailLevel::Quick => matches!(section.visibility_level, DetailLevel::Quick | DetailLevel::Normal),
                DetailLevel::Normal => !matches!(section.visibility_level, DetailLevel::Expert),
                DetailLevel::Detailed => true,
                DetailLevel::Expert => true,
            }
        });

        detailed
    }

    fn combine_search_results(&self, results: Vec<HelpSearchResult>, detail_level: DetailLevel) -> HelpContent {
        let mut combined = HelpContent::new("search-results".to_string());
        combined.title = "Search Results".to_string();
        combined.summary = format!("Found {} matching topics", results.len());

        for result in results.iter().take(10) {
            combined.sections.push(HelpSection {
                title: result.title.clone(),
                content: format!("{}\n\nRelevance: {:.0}%", result.snippet, result.relevance_score * 100.0),
                section_type: SectionType::Overview,
                visibility_level: detail_level,
                interactive_elements: vec![
                    InteractiveElement::QuickLink {
                        text: "View full topic".to_string(),
                        target: result.topic.clone(),
                    }
                ],
            });
        }

        combined
    }

    async fn generate_dataflow_contextual_help(&self, dataflows: &[DataflowInfo]) -> Result<Vec<HelpSection>> {
        let mut sections = Vec::new();

        sections.push(HelpSection {
            title: "Active Dataflows".to_string(),
            content: format!(
                "You currently have {} active dataflow(s). Here are some common operations:",
                dataflows.len()
            ),
            section_type: SectionType::Overview,
            visibility_level: DetailLevel::Normal,
            interactive_elements: Vec::new(),
        });

        // Add examples for common dataflow operations
        let first_dataflow = dataflows.first().map(|d| d.name.as_str()).unwrap_or("DATAFLOW_NAME");
        let examples_content = format!(
            "• View dataflow status: `dora ps`\n\
             • Inspect a specific dataflow: `dora inspect {}`\n\
             • View logs: `dora logs {}`\n\
             • Stop a dataflow: `dora stop {}`",
            first_dataflow, first_dataflow, first_dataflow
        );

        sections.push(HelpSection {
            title: "Common Operations".to_string(),
            content: examples_content,
            section_type: SectionType::Examples,
            visibility_level: DetailLevel::Normal,
            interactive_elements: vec![
                InteractiveElement::RunnableCommand {
                    command: "dora ps".to_string(),
                    description: "Show current dataflow status".to_string(),
                },
            ],
        });

        Ok(sections)
    }

    async fn generate_error_contextual_help(&self, errors: &[ErrorInfo]) -> Result<Vec<HelpSection>> {
        let mut sections = Vec::new();

        let error_summary = format!(
            "Recent errors detected ({}). Common troubleshooting steps:",
            errors.len()
        );

        sections.push(HelpSection {
            title: "Troubleshooting Recent Errors".to_string(),
            content: error_summary,
            section_type: SectionType::Troubleshooting,
            visibility_level: DetailLevel::Normal,
            interactive_elements: Vec::new(),
        });

        // Group errors by type for targeted help
        let grouped_errors = self.group_errors_by_type(errors);

        for (error_type, error_group) in grouped_errors {
            let troubleshooting_content = self.get_error_type_help(&error_type, &error_group).await?;
            sections.push(troubleshooting_content);
        }

        Ok(sections)
    }

    fn group_errors_by_type(&self, errors: &[ErrorInfo]) -> HashMap<String, Vec<ErrorInfo>> {
        let mut grouped = HashMap::new();
        for error in errors {
            grouped.entry(error.error_type.clone())
                .or_insert_with(Vec::new)
                .push(error.clone());
        }
        grouped
    }

    async fn get_error_type_help(&self, error_type: &str, errors: &[ErrorInfo]) -> Result<HelpSection> {
        let content = format!(
            "Error type: {}\nOccurrences: {}\n\nSuggested actions:\n\
             • Check logs with `dora logs --errors-only`\n\
             • Inspect system state with `dora inspect`\n\
             • Review configuration files",
            error_type,
            errors.len()
        );

        Ok(HelpSection {
            title: format!("{} Errors", error_type),
            content,
            section_type: SectionType::Troubleshooting,
            visibility_level: DetailLevel::Normal,
            interactive_elements: Vec::new(),
        })
    }

    async fn generate_performance_help(&self, issues: &PerformanceIssues) -> Result<Vec<HelpSection>> {
        let mut sections = Vec::new();

        let mut recommendations = Vec::new();
        if issues.high_cpu_usage {
            recommendations.push("• Consider reducing node workload or optimizing algorithms");
        }
        if issues.high_memory_usage {
            recommendations.push("• Review memory allocations and consider streaming data processing");
        }
        if issues.slow_response_time {
            recommendations.push("• Profile dataflow performance with `dora analyze --performance`");
        }

        sections.push(HelpSection {
            title: "Performance Optimization".to_string(),
            content: format!(
                "Performance issues detected: {}\n\nRecommendations:\n{}",
                issues.description,
                recommendations.join("\n")
            ),
            section_type: SectionType::Troubleshooting,
            visibility_level: DetailLevel::Normal,
            interactive_elements: Vec::new(),
        });

        Ok(sections)
    }

    async fn generate_contextual_suggestions(
        &self,
        context: &SystemContext,
        user_expertise: UserExpertiseLevel,
    ) -> Result<String> {
        let mut suggestions = Vec::new();

        if !context.active_dataflows.is_empty() {
            suggestions.push("• Use `dora inspect` to view detailed dataflow information");
            suggestions.push("• Monitor performance with `dora monitor`");
        }

        if !context.recent_errors.is_empty() {
            suggestions.push("• Debug issues with `dora debug`");
            suggestions.push("• Analyze error patterns with `dora analyze --errors`");
        }

        if matches!(user_expertise, UserExpertiseLevel::Beginner) {
            suggestions.push("• Try interactive tutorials with `dora help --tutorial`");
        }

        Ok(suggestions.join("\n"))
    }
}

// ============================================================================
// Help Search Engine
// ============================================================================

#[derive(Debug)]
pub struct HelpSearchEngine {
    // Mock search engine - would use real search index
}

impl HelpSearchEngine {
    pub fn new() -> Self {
        Self {}
    }

    pub async fn search(
        &self,
        query: &str,
        _user_expertise: UserExpertiseLevel,
    ) -> Result<Vec<HelpSearchResult>> {
        // Mock search results
        let mut results = Vec::new();

        // Simple keyword matching (would use real search in production)
        let keywords = ["start", "stop", "logs", "inspect", "debug", "analyze"];

        for keyword in keywords {
            if query.to_lowercase().contains(keyword) {
                results.push(HelpSearchResult {
                    topic: keyword.to_string(),
                    title: format!("{} Command", keyword),
                    relevance_score: 0.8,
                    matched_keywords: vec![keyword.to_string()],
                    snippet: format!("Help for {} command...", keyword),
                });
            }
        }

        Ok(results)
    }
}

// ============================================================================
// Help Personalization Engine
// ============================================================================

#[derive(Debug)]
pub struct HelpPersonalizationEngine {
    // Mock personalization - would use ML in production
}

impl HelpPersonalizationEngine {
    pub fn new() -> Self {
        Self {}
    }

    pub async fn enhance_content(
        &self,
        content: &HelpContent,
        user_expertise: UserExpertiseLevel,
    ) -> Result<HelpContent> {
        let mut enhanced = content.clone();

        // Add personalized recommendations
        match user_expertise {
            UserExpertiseLevel::Beginner => {
                enhanced.related_topics.insert(0, "tutorials".to_string());
                enhanced.related_topics.insert(1, "getting-started".to_string());
            }
            UserExpertiseLevel::Intermediate => {
                enhanced.related_topics.insert(0, "advanced-usage".to_string());
            }
            UserExpertiseLevel::Expert => {
                enhanced.related_topics.insert(0, "api-reference".to_string());
                enhanced.related_topics.insert(1, "optimization".to_string());
            }
        }

        Ok(enhanced)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_get_topic_content() {
        let manager = HelpContentManager::new();
        let content = manager.get_topic_content(
            "start",
            UserExpertiseLevel::Beginner,
            DetailLevel::Normal,
        ).await;

        assert!(content.is_ok());
        let content = content.unwrap();
        assert_eq!(content.topic, "start");
    }

    #[tokio::test]
    async fn test_search_content() {
        let manager = HelpContentManager::new();
        let results = manager.search_content(
            "start dataflow",
            UserExpertiseLevel::Beginner,
            DetailLevel::Normal,
        ).await;

        assert!(results.is_ok());
    }

    #[tokio::test]
    async fn test_contextual_help_with_dataflows() {
        let manager = HelpContentManager::new();
        let context = SystemContext {
            active_dataflows: vec![DataflowInfo {
                name: "test-flow".to_string(),
                status: "running".to_string(),
                node_count: 3,
                uptime: Some(Duration::hours(1)),
            }],
            recent_errors: Vec::new(),
            performance_issues: None,
            recent_commands: Vec::new(),
            environment_info: EnvironmentInfo {
                is_ci: false,
                shell_type: Some("bash".to_string()),
                terminal_capabilities: "full".to_string(),
            },
        };

        let help = manager.get_contextual_help(&Some(context), UserExpertiseLevel::Beginner).await;
        assert!(help.is_ok());
        let help = help.unwrap();
        assert!(!help.sections.is_empty());
    }
}
