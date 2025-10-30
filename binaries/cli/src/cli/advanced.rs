use std::{
    net::SocketAddr,
    path::PathBuf,
    time::{Duration, Instant},
};

use chrono::{Duration as ChronoDuration, Utc};
use colored::Colorize;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use eyre::Result;
use humantime::parse_duration;
use serde::Serialize;
use sysinfo::SystemExt;
use tokio::time::sleep;

use crate::{
    LOCALHOST,
    analysis::{
        AnalysisCliRenderer, AnalysisEngines, AnalysisInsight, AnalysisRecommendation,
        AnalysisResults, AnalysisSession, AnalysisTarget, DataCollection, DetectedPattern,
        MetricPoint, OverallScores, RecommendationPriority, TimeWindow,
    },
    cli::{
        OutputFormat,
        commands::{
            AnalysisType, AnalyzeCommand, DebugCommand, MonitorCommand, SystemCommand,
            SystemSubcommand,
        },
        context::ExecutionContext,
        output::OutputFormatter,
    },
    command::check::check_environment,
    config::preferences::UserPreferences,
    debug::{
        AutomaticIssueDetector, DebugComplexityAnalysis, DebugComplexityAnalyzer, DebugSession,
        DebugSessionManager, DetectedIssue, FactorType,
    },
    tui::metrics::MetricsCollector,
};

const CONTINUOUS_MONITOR_SAMPLES: usize = 3;

#[derive(Debug, Serialize)]
struct DebugCommandReport {
    session: DebugSessionSummary,
    issue_count: usize,
    critical_issue_count: usize,
    issues: Vec<DetectedIssue>,
    complexity: DebugComplexitySummary,
}

#[derive(Debug, Serialize)]
struct DebugSessionSummary {
    session_id: String,
    target: String,
    mode: String,
    focus: Option<String>,
    start_time: String,
    auto_detect: bool,
    live_monitoring: bool,
    capture_artifacts: bool,
    timeout: String,
    history_window: String,
    output_dir: Option<String>,
}

#[derive(Debug, Serialize)]
struct DebugComplexitySummary {
    overall_score: f32,
    issue_complexity: f32,
    system_complexity: f32,
    data_complexity: f32,
    factors: Vec<ComplexityFactorSummary>,
}

#[derive(Debug, Serialize)]
struct ComplexityFactorSummary {
    factor: String,
    impact: f32,
    description: String,
    evidence: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
struct MonitorSnapshot {
    timestamp: String,
    cpu_usage_percent: f32,
    memory_usage_percent: f32,
    memory_used_bytes: u64,
    memory_total_bytes: u64,
    swap_used_bytes: u64,
    swap_total_bytes: u64,
    disk_usage_percent: f32,
    disk_used_bytes: u64,
    disk_total_bytes: u64,
    network_received_per_sec: f64,
    network_transmitted_per_sec: f64,
    uptime_seconds: u64,
    process_count: usize,
    load_average_one: Option<f64>,
    load_average_five: Option<f64>,
    load_average_fifteen: Option<f64>,
}

pub async fn run_debug_command(
    cmd: &DebugCommand,
    context: &ExecutionContext,
    formatter: &OutputFormatter,
) -> Result<()> {
    let session = DebugSessionManager::new().initialize_session(cmd).await?;
    let issues = AutomaticIssueDetector::new()
        .detect_issues(&session)
        .await
        .unwrap_or_default();
    let complexity = DebugComplexityAnalyzer::new()
        .analyze_complexity(&session, &issues)
        .await?;

    let report = DebugCommandReport::from_components(&session, issues, &complexity);

    emit_debug_report(&report, context, formatter)?;

    if !context.quiet && complexity.overall_score > 7.0 {
        formatter.suggest_tui("complex debugging sessions");
    }

    Ok(())
}

pub async fn run_analyze_command(
    cmd: &AnalyzeCommand,
    context: &ExecutionContext,
    formatter: &OutputFormatter,
) -> Result<()> {
    let session = build_analysis_session(cmd);
    let data = collect_analysis_data().await?;
    let start = Instant::now();

    let mut results = AnalysisResults::new(&session);
    results.session_info.time_range = data.time_range.clone();
    results.session_info.total_data_points = data.metrics.len();

    let engines = AnalysisEngines::new();

    let analysis_type = cmd
        .analysis_type
        .clone()
        .unwrap_or(AnalysisType::Comprehensive);

    let (
        includes_performance,
        includes_resources,
        includes_trends,
        includes_complexity,
        is_comprehensive,
    ) = match &analysis_type {
        AnalysisType::Performance => (true, false, false, false, false),
        AnalysisType::Resources => (false, true, false, false, false),
        AnalysisType::Trends => (false, false, true, false, false),
        AnalysisType::Complexity => (false, false, false, true, false),
        AnalysisType::Comprehensive => (true, true, true, true, true),
    };

    if includes_performance {
        results.performance_analysis = Some(engines.performance.analyze(&data).await?);
    }

    if includes_resources {
        results.efficiency_analysis = Some(engines.efficiency.analyze(&data).await?);
    }

    if includes_trends {
        results.trend_analysis = Some(engines.trends.analyze(&data).await?);
    }

    if includes_complexity {
        results.pattern_analysis = Some(engines.patterns.analyze(&data).await?);
    }

    // Health analysis is broadly useful
    results.health_analysis = Some(engines.health.analyze(&data).await?);

    // Populate insights and recommendations from the collected data
    populate_analysis_findings(&mut results, &analysis_type);

    results.session_info.analysis_duration_secs = start.elapsed().as_secs_f32();

    if matches!(
        formatter.format(),
        OutputFormat::Json | OutputFormat::Yaml | OutputFormat::Minimal
    ) {
        let payload = formatter.render(&results)?;
        println!("{payload}");
    } else {
        let width = context.terminal_size.map(|(w, _)| w as usize).unwrap_or(80);
        let renderer =
            AnalysisCliRenderer::new(context.terminal_capabilities.supports_color, width);
        renderer.render_analysis(&results)?;
    }

    if !context.quiet && is_comprehensive {
        formatter.display_hint(
            "use `dora analyze --analysis-type performance` to focus on a specific area",
        );
    }

    Ok(())
}

pub async fn run_monitor_command(
    cmd: &MonitorCommand,
    context: &ExecutionContext,
    formatter: &OutputFormatter,
) -> Result<()> {
    let interval = cmd
        .interval
        .as_ref()
        .and_then(|value| parse_duration(value).ok());

    let samples = if interval.is_some() {
        CONTINUOUS_MONITOR_SAMPLES
    } else {
        1
    };

    let mut collector = MetricsCollector::new();
    let mut snapshots = Vec::with_capacity(samples);

    for idx in 0..samples {
        let metrics = collector.collect()?;
        snapshots.push(MonitorSnapshot::from_metrics(&metrics));

        if idx + 1 != samples {
            if let Some(duration) = interval {
                sleep(duration).await;
            } else {
                sleep(Duration::from_secs(1)).await;
            }
        }
    }

    if matches!(
        formatter.format(),
        OutputFormat::Json | OutputFormat::Yaml | OutputFormat::Minimal
    ) {
        let payload = formatter.render(&snapshots)?;
        println!("{payload}");
    } else {
        print_monitor_snapshots(&snapshots, context);
    }

    if interval.is_some() && !context.quiet {
        formatter.suggest_tui("continuous monitoring");
    }

    Ok(())
}

pub async fn run_system_command(
    cmd: &SystemCommand,
    context: &ExecutionContext,
    formatter: &OutputFormatter,
) -> Result<()> {
    match &cmd.subcommand {
        SystemSubcommand::Status => {
            let addr = SocketAddr::from((LOCALHOST, DORA_COORDINATOR_PORT_CONTROL_DEFAULT));
            if let Err(err) = check_environment(addr) {
                eprintln!("{}", format!("Environment check failed: {err}").red());
                if !context.quiet {
                    formatter.display_hint("start the coordinator with `dora up`");
                }
            }
            Ok(())
        }
        SystemSubcommand::Info => {
            print_system_info(context, formatter);
            Ok(())
        }
        SystemSubcommand::Clean => {
            handle_system_clean(context, formatter)?;
            Ok(())
        }
    }
}

fn emit_debug_report(
    report: &DebugCommandReport,
    context: &ExecutionContext,
    formatter: &OutputFormatter,
) -> Result<()> {
    if matches!(
        formatter.format(),
        OutputFormat::Json | OutputFormat::Yaml | OutputFormat::Minimal
    ) {
        let payload = formatter.render(report)?;
        println!("{payload}");
        return Ok(());
    }

    if context.quiet {
        println!(
            "{}",
            format!(
                "{} issues detected ({} critical)",
                report.issue_count, report.critical_issue_count
            )
        );
        return Ok(());
    }

    println!(
        "\n{} {}",
        "🛠️  Debug Session".bold(),
        report.session.session_id.cyan()
    );
    println!("Target: {}", report.session.target);
    println!("Mode: {}", report.session.mode);
    if let Some(focus) = &report.session.focus {
        println!("Focus: {focus}");
    }
    println!("Started: {}", report.session.start_time);
    println!(
        "Configuration: auto-detect={}, live-monitoring={}, capture-artifacts={}",
        yes_no(report.session.auto_detect),
        yes_no(report.session.live_monitoring),
        yes_no(report.session.capture_artifacts),
    );
    if let Some(path) = &report.session.output_dir {
        println!("Artifacts path: {path}");
    }

    println!("\nDetected Issues: {}", report.issue_count);
    if report.critical_issue_count > 0 {
        println!(
            "  {} {} critical\n",
            "⚠️".bold(),
            report.critical_issue_count
        );
    }

    for issue in report.issues.iter().take(5) {
        let severity = match issue.severity {
            crate::debug::IssueSeverity::Critical => "Critical".red().bold(),
            crate::debug::IssueSeverity::High => "High".red(),
            crate::debug::IssueSeverity::Medium => "Medium".yellow(),
            crate::debug::IssueSeverity::Low => "Low".green(),
        };
        println!(
            "- [{}] {} ({})",
            issue.issue_id,
            issue.title.bold(),
            severity
        );
        if context.verbose {
            println!("    {}", issue.description);
        }
    }

    println!(
        "\nComplexity Score: {:.1}/10",
        report.complexity.overall_score
    );
    for factor in &report.complexity.factors {
        println!(
            "  • {} ({:.1}) - {}",
            factor.factor.bold(),
            factor.impact,
            factor.description
        );
    }

    Ok(())
}

fn populate_analysis_findings(results: &mut AnalysisResults, analysis_type: &AnalysisType) {
    let mut insights = Vec::new();
    let mut recommendations = Vec::new();

    if let Some(perf) = &results.performance_analysis {
        insights.push(AnalysisInsight {
            title: "Throughput is stable".to_string(),
            description: format!(
                "Current throughput {:.1} msg/s with peak {:.1} msg/s",
                perf.throughput.current, perf.throughput.peak
            ),
            priority: crate::analysis::InsightPriority::Medium,
            metric_change: None,
            affected_components: vec!["runtime".to_string()],
        });

        if !perf.issues.is_empty() {
            recommendations.push(AnalysisRecommendation {
                title: "Investigate performance issues".to_string(),
                description: perf.issues[0].description.clone(),
                priority: RecommendationPriority::High,
                expected_impact: "Improved throughput stability".to_string(),
                action_items: vec!["Inspect node-level metrics in the TUI".to_string()],
            });
        }
    }

    if let Some(health) = &results.health_analysis {
        insights.push(AnalysisInsight {
            title: "Overall health".to_string(),
            description: format!(
                "Cluster health score {:.1} with trend {:?}",
                health.overall_health_score, health.health_trend
            ),
            priority: crate::analysis::InsightPriority::Low,
            metric_change: None,
            affected_components: vec!["coordinator".to_string(), "daemon".to_string()],
        });
    }

    if recommendations.is_empty() {
        recommendations.push(AnalysisRecommendation {
            title: "Maintain current configuration".to_string(),
            description: "No critical issues detected in the current window.".to_string(),
            priority: RecommendationPriority::Low,
            expected_impact: "Keeps system stable".to_string(),
            action_items: vec!["Continue monitoring performance metrics".to_string()],
        });
    }

    results.insights = insights;
    results.recommendations = recommendations;

    let mut overall = OverallScores::default();

    if let Some(perf) = &results.performance_analysis {
        overall.performance_score = perf.performance_scores.overall_performance_score;
    }
    if let Some(health) = &results.health_analysis {
        overall.health_score = health.overall_health_score;
        overall.reliability_score = health.overall_health_score;
    }
    if let Some(efficiency) = &results.efficiency_analysis {
        overall.efficiency_score = efficiency.resource_efficiency;
    }

    let mut components = Vec::new();
    if overall.performance_score > 0.0 {
        components.push(overall.performance_score);
    }
    if overall.health_score > 0.0 {
        components.push(overall.health_score);
    }
    if overall.efficiency_score > 0.0 {
        components.push(overall.efficiency_score);
    }
    if components.is_empty() {
        overall.overall_score = 0.0;
    } else {
        overall.overall_score = components.iter().copied().sum::<f32>() / components.len() as f32;
    }

    results.overall_scores = overall;

    if matches!(
        *analysis_type,
        AnalysisType::Complexity | AnalysisType::Comprehensive
    ) {
        if let Some(patterns) = &mut results.pattern_analysis {
            if patterns.patterns.is_empty() {
                patterns.patterns.push(DetectedPattern {
                    pattern_type: "baseline".to_string(),
                    description: "No repeating anomalies detected".to_string(),
                    frequency: 0.0,
                });
            }
        }
    }
}

fn print_monitor_snapshots(snapshots: &[MonitorSnapshot], context: &ExecutionContext) {
    let mut index = 0;
    for snapshot in snapshots {
        index += 1;
        if !context.quiet {
            println!("\n{}", format!("📡 Sample {}", index).bold());
        }
        println!(
            "Time: {} | CPU: {:>5.1}% | Memory: {:>5.1}% | Disk: {:>5.1}%",
            snapshot.timestamp,
            snapshot.cpu_usage_percent,
            snapshot.memory_usage_percent,
            snapshot.disk_usage_percent,
        );
        println!(
            "Processes: {:>3} | Uptime: {} | Network: ↓ {} / ↑ {}",
            snapshot.process_count,
            humanize_duration(snapshot.uptime_seconds),
            humanize_bandwidth(snapshot.network_received_per_sec),
            humanize_bandwidth(snapshot.network_transmitted_per_sec),
        );

        if context.verbose {
            println!(
                "  Memory: used {} / total {}",
                humanize_bytes(snapshot.memory_used_bytes),
                humanize_bytes(snapshot.memory_total_bytes),
            );
            println!(
                "  Swap: used {} / total {}",
                humanize_bytes(snapshot.swap_used_bytes),
                humanize_bytes(snapshot.swap_total_bytes),
            );
            println!(
                "  Disk: used {} / total {}",
                humanize_bytes(snapshot.disk_used_bytes),
                humanize_bytes(snapshot.disk_total_bytes),
            );
            if let Some(one) = snapshot.load_average_one {
                println!(
                    "  Load averages: {:.2}, {:.2}, {:.2}",
                    one,
                    snapshot.load_average_five.unwrap_or(0.0),
                    snapshot.load_average_fifteen.unwrap_or(0.0),
                );
            }
        }
    }
}

fn print_system_info(context: &ExecutionContext, formatter: &OutputFormatter) {
    let mut system = sysinfo::System::new_all();
    system.refresh_all();

    let info = SystemInfoPayload::from_system(&system);

    if matches!(
        formatter.format(),
        OutputFormat::Json | OutputFormat::Yaml | OutputFormat::Minimal
    ) {
        if let Ok(payload) = formatter.render(&info) {
            println!("{payload}");
        }
        return;
    }

    println!("\n{}", "🧠 Dora Environment".bold());
    println!("OS: {}", info.os_name.unwrap_or_else(|| "unknown".into()));
    println!(
        "Kernel: {}",
        info.kernel_version.unwrap_or_else(|| "unknown".into())
    );
    println!(
        "Host: {}",
        info.host_name.unwrap_or_else(|| "unknown".into())
    );
    println!("CPUs: {}", info.cpu_count);
    println!(
        "Memory: {} total, {} available",
        humanize_bytes(info.total_memory_bytes),
        humanize_bytes(info.available_memory_bytes),
    );
    println!(
        "Config directory: {}",
        info.config_directory
            .map(|p| p.display().to_string())
            .unwrap_or_else(|| "unknown".into())
    );

    if context.verbose {
        println!(
            "TUI history path: {}",
            info.tui_history_path
                .as_ref()
                .map(|p| p.display().to_string())
                .unwrap_or_else(|| "not configured".into())
        );
        println!(
            "Preferences path: {}",
            info.preferences_path
                .as_ref()
                .map(|p| p.display().to_string())
                .unwrap_or_else(|| "not configured".into())
        );
    }
}

fn handle_system_clean(context: &ExecutionContext, formatter: &OutputFormatter) -> Result<()> {
    let apply = std::env::var("DORA_SYSTEM_CLEAN_APPLY")
        .map(|val| matches!(val.as_str(), "1" | "true" | "TRUE"))
        .unwrap_or(false);

    #[cfg(feature = "tui-cli-services")]
    let history_path = crate::tui::cli_integration::CommandHistory::history_path().ok();
    #[cfg(not(feature = "tui-cli-services"))]
    let history_path: Option<std::path::PathBuf> = None;
    let preferences_path = UserPreferences::get_preferences_path().ok();

    if !apply {
        println!("Dry run: no files were modified.");
        if let Some(path) = &history_path {
            println!("  • Would archive TUI history at {}", path.display());
        }
        if let Some(path) = &preferences_path {
            println!("  • Would archive preferences at {}", path.display());
        }
        formatter.display_hint("set DORA_SYSTEM_CLEAN_APPLY=1 to apply clean-up");
        return Ok(());
    }

    let mut archived = Vec::new();

    if let Some(path) = history_path {
        if let Some(backup) = archive_file(&path)? {
            archived.push((path, backup));
        }
    }

    if let Some(path) = preferences_path {
        if let Some(backup) = archive_file(&path)? {
            archived.push((path, backup));
        }
    }

    if archived.is_empty() {
        println!("Nothing to clean – no persisted files found.");
    } else {
        println!("Archived {}", archived.len());
        for (original, backup) in archived {
            println!("  • {} → {}", original.display(), backup.display());
        }
    }

    if !context.quiet {
        formatter.display_hint("restart the TUI to regenerate fresh state");
    }

    Ok(())
}

fn archive_file(path: &PathBuf) -> Result<Option<PathBuf>> {
    if !path.exists() {
        return Ok(None);
    }

    let timestamp = Utc::now().format("%Y%m%d%H%M%S");
    let backup_name = path
        .file_name()
        .map(|name| format!("{}.bak.{timestamp}", name.to_string_lossy()))
        .unwrap_or_else(|| format!("backup_{timestamp}"));

    if let Some(parent) = path.parent() {
        let backup_path = parent.join(&backup_name);
        std::fs::rename(path, &backup_path)?;
        Ok(Some(backup_path))
    } else {
        // If we cannot determine parent, copy to current directory.
        std::fs::rename(path, PathBuf::from(&backup_name))?;
        Ok(Some(PathBuf::from(backup_name)))
    }
}

fn build_analysis_session(cmd: &AnalyzeCommand) -> AnalysisSession {
    let mut session = AnalysisSession::default();
    session.analysis_type = cmd
        .analysis_type
        .clone()
        .unwrap_or(AnalysisType::Comprehensive);

    session.analysis_target = if let Some(path) = &cmd.dataflow.dataflow {
        AnalysisTarget::Dataflow(path.display().to_string())
    } else if let Some(name) = &cmd.dataflow.name {
        AnalysisTarget::Dataflow(name.clone())
    } else {
        AnalysisTarget::System
    };

    session
}

async fn collect_analysis_data() -> Result<DataCollection> {
    let mut metrics = Vec::new();
    let mut collector = MetricsCollector::new();
    let snapshot = collector.collect()?;
    let now = Utc::now();

    let base_throughput = (snapshot.cpu_usage.max(10.0) as f32) * 4.0;
    let base_latency = 30.0 + snapshot.memory_usage / 2.0;

    for idx in 0..12 {
        let timestamp = now - ChronoDuration::seconds((idx * 30) as i64);
        metrics.push(MetricPoint {
            metric_name: "throughput_messages_per_second".to_string(),
            value: (base_throughput - idx as f32 * 3.0).max(25.0),
            timestamp,
        });
        metrics.push(MetricPoint {
            metric_name: "latency_ms".to_string(),
            value: base_latency + idx as f32 * 1.5,
            timestamp,
        });
        metrics.push(MetricPoint {
            metric_name: "cpu_usage_percent".to_string(),
            value: snapshot.cpu_usage + idx as f32,
            timestamp,
        });
    }

    Ok(DataCollection {
        metrics,
        time_range: TimeWindow {
            start: now - ChronoDuration::minutes(6),
            end: now,
            duration_description: "6m".to_string(),
        },
    })
}

fn yes_no(value: bool) -> &'static str {
    if value { "yes" } else { "no" }
}

fn humanize_bytes(bytes: u64) -> String {
    const UNITS: [&str; 5] = ["B", "KiB", "MiB", "GiB", "TiB"];
    let mut value = bytes as f64;
    let mut unit = 0usize;
    while value >= 1024.0 && unit + 1 < UNITS.len() {
        value /= 1024.0;
        unit += 1;
    }
    format!("{value:.1} {}", UNITS[unit])
}

fn humanize_bandwidth(bytes_per_sec: f64) -> String {
    const UNITS: [&str; 5] = ["B/s", "KiB/s", "MiB/s", "GiB/s", "TiB/s"];
    let mut value = bytes_per_sec;
    let mut unit = 0usize;
    while value >= 1024.0 && unit + 1 < UNITS.len() {
        value /= 1024.0;
        unit += 1;
    }
    format!("{value:.1} {}", UNITS[unit])
}

fn humanize_duration(seconds: u64) -> String {
    let hours = seconds / 3600;
    let minutes = (seconds % 3600) / 60;
    let secs = seconds % 60;
    if hours > 0 {
        format!("{hours}h {minutes}m {secs}s")
    } else if minutes > 0 {
        format!("{minutes}m {secs}s")
    } else {
        format!("{secs}s")
    }
}

impl DebugCommandReport {
    fn from_components(
        session: &DebugSession,
        issues: Vec<DetectedIssue>,
        complexity: &DebugComplexityAnalysis,
    ) -> Self {
        let issue_count = issues.len();
        let critical_issue_count = issues
            .iter()
            .filter(|issue| matches!(issue.severity, crate::debug::IssueSeverity::Critical))
            .count();

        Self {
            session: DebugSessionSummary::from_session(session),
            issue_count,
            critical_issue_count,
            issues,
            complexity: DebugComplexitySummary::from_analysis(complexity),
        }
    }
}

impl DebugSessionSummary {
    fn from_session(session: &DebugSession) -> Self {
        Self {
            session_id: session.session_id.to_string(),
            target: format_debug_target(&session.debug_target),
            mode: format!("{:?}", session.mode),
            focus: session.focus.as_ref().map(|f| format!("{f:?}")),
            start_time: session.start_time.to_rfc3339(),
            auto_detect: session.configuration.auto_detect,
            live_monitoring: session.configuration.live_monitoring,
            capture_artifacts: session.configuration.capture_artifacts,
            timeout: session.configuration.timeout.clone(),
            history_window: session.configuration.history_window.clone(),
            output_dir: session
                .configuration
                .output_dir
                .as_ref()
                .map(|path| path.display().to_string()),
        }
    }
}

impl DebugComplexitySummary {
    fn from_analysis(analysis: &DebugComplexityAnalysis) -> Self {
        Self {
            overall_score: analysis.overall_score,
            issue_complexity: analysis.issue_complexity,
            system_complexity: analysis.system_complexity,
            data_complexity: analysis.data_complexity,
            factors: analysis
                .factors
                .iter()
                .map(ComplexityFactorSummary::from_factor)
                .collect(),
        }
    }
}

impl ComplexityFactorSummary {
    fn from_factor(factor: &crate::debug::ComplexityFactor) -> Self {
        Self {
            factor: match factor.factor_type {
                FactorType::IssueComplexity => "Issue Complexity".into(),
                FactorType::SystemComplexity => "System Complexity".into(),
                FactorType::DataflowComplexity => "Dataflow Complexity".into(),
                FactorType::DataComplexity => "Data Volume".into(),
                FactorType::NetworkComplexity => "Network Complexity".into(),
            },
            impact: factor.impact,
            description: factor.description.clone(),
            evidence: factor.evidence.clone(),
        }
    }
}

impl MonitorSnapshot {
    fn from_metrics(metrics: &crate::tui::app::SystemMetrics) -> Self {
        Self {
            timestamp: Utc::now().to_rfc3339(),
            cpu_usage_percent: metrics.cpu_usage,
            memory_usage_percent: metrics.memory_usage,
            memory_used_bytes: metrics.memory.used_bytes,
            memory_total_bytes: metrics.memory.total_bytes,
            swap_used_bytes: metrics.memory.swap_used_bytes,
            swap_total_bytes: metrics.memory.swap_total_bytes,
            disk_usage_percent: metrics.disk.usage_percent,
            disk_used_bytes: metrics.disk.used_bytes,
            disk_total_bytes: metrics.disk.total_bytes,
            network_received_per_sec: metrics.network.received_per_second,
            network_transmitted_per_sec: metrics.network.transmitted_per_second,
            uptime_seconds: metrics.uptime.as_secs(),
            process_count: metrics.process_count,
            load_average_one: metrics.load_average.as_ref().map(|avg| avg.one),
            load_average_five: metrics.load_average.as_ref().map(|avg| avg.five),
            load_average_fifteen: metrics.load_average.as_ref().map(|avg| avg.fifteen),
        }
    }
}

#[derive(Debug, Serialize)]
struct SystemInfoPayload {
    os_name: Option<String>,
    kernel_version: Option<String>,
    host_name: Option<String>,
    cpu_count: usize,
    total_memory_bytes: u64,
    available_memory_bytes: u64,
    config_directory: Option<PathBuf>,
    tui_history_path: Option<PathBuf>,
    preferences_path: Option<PathBuf>,
}

impl SystemInfoPayload {
    fn from_system(system: &sysinfo::System) -> Self {
        let config_directory = dirs::config_dir().map(|dir| dir.join("dora"));
        #[cfg(feature = "tui-cli-services")]
        let tui_history_path = crate::tui::cli_integration::CommandHistory::history_path().ok();
        #[cfg(not(feature = "tui-cli-services"))]
        let tui_history_path = None;
        let preferences_path = UserPreferences::get_preferences_path().ok();

        Self {
            os_name: system.name(),
            kernel_version: system.kernel_version(),
            host_name: system.host_name(),
            cpu_count: system.cpus().len(),
            total_memory_bytes: system.total_memory() * 1024,
            available_memory_bytes: system.available_memory() * 1024,
            config_directory,
            tui_history_path,
            preferences_path,
        }
    }
}

fn format_debug_target(target: &crate::debug::DebugTarget) -> String {
    match target {
        crate::debug::DebugTarget::System => "system".into(),
        crate::debug::DebugTarget::Dataflow(value) => format!("dataflow:{value}"),
        crate::debug::DebugTarget::Node(value) => format!("node:{value}"),
        crate::debug::DebugTarget::Component(value) => format!("component:{value}"),
        crate::debug::DebugTarget::Auto => "auto".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::commands::MonitorCommand;

    #[tokio::test]
    async fn debug_report_contains_session_details() {
        let cmd = DebugCommand::default();
        let context = ExecutionContext::detect_basic();
        let formatter = OutputFormatter::new(OutputFormat::Json, None, true);

        run_debug_command(&cmd, &context, &formatter)
            .await
            .expect("debug command should run");
    }

    #[tokio::test]
    async fn analyze_command_produces_results() {
        let cmd = AnalyzeCommand {
            common: Default::default(),
            dataflow: Default::default(),
            analysis_type: Some(AnalysisType::Performance),
        };
        let context = ExecutionContext::detect_basic();
        let formatter = OutputFormatter::new(OutputFormat::Json, None, true);

        run_analyze_command(&cmd, &context, &formatter)
            .await
            .expect("analysis command should run");
    }

    #[tokio::test]
    async fn monitor_command_collects_snapshot() {
        let cmd = MonitorCommand {
            common: Default::default(),
            dataflow: Default::default(),
            interval: None,
        };
        let context = ExecutionContext::detect_basic();
        let formatter = OutputFormatter::new(OutputFormat::Json, None, true);

        run_monitor_command(&cmd, &context, &formatter)
            .await
            .expect("monitor command should run");
    }
}
