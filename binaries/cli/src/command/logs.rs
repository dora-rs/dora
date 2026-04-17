use std::{
    collections::HashMap,
    io::{Read, Seek, Write},
    path::{Path, PathBuf},
};

use super::{Executable, default_tracing};
use crate::{
    common::{
        CoordinatorOptions, expect_reply, resolve_dataflow_identifier_interactive,
        send_control_request,
    },
    output::{
        LogFormat, LogOutputConfig, parse_jsonl_line, parse_log_filter, parse_log_level_str,
        print_log_message,
    },
    ws_client::WsSession,
};
use chrono::{DateTime, Utc};
use clap::Args;
use dora_message::{cli_to_coordinator::ControlRequest, common::LogMessage, id::NodeId};
use duration_str::parse as parse_duration_str;
use eyre::{Context, Result, bail};
use uuid::Uuid;

#[derive(Debug, Args)]
/// Show logs of a given dataflow and node.
pub struct LogsArgs {
    /// Identifier of the dataflow
    #[clap(value_name = "UUID_OR_NAME")]
    pub dataflow: Option<String>,
    /// Show logs for the given node (omit with --all-nodes)
    #[clap(value_name = "NAME")]
    pub node: Option<NodeId>,
    /// Show logs from all nodes merged by timestamp.
    /// Streams from coordinator by default, falls back to local out/ directory.
    #[clap(long)]
    pub all_nodes: bool,
    /// Number of lines to show from the end of the logs
    #[clap(long)]
    pub tail: Option<usize>,
    /// Follow log output
    #[clap(long, short)]
    pub follow: bool,
    /// Read log files from local out/ directory instead of coordinator
    #[clap(long)]
    pub local: bool,
    /// Only show logs newer than this duration ago (e.g. "5m", "1h")
    #[clap(long, value_name = "DURATION")]
    #[arg(value_parser = parse_duration_str)]
    pub since: Option<std::time::Duration>,
    /// Only show logs older than this duration ago (e.g. "5m", "1h")
    #[clap(long, value_name = "DURATION")]
    #[arg(value_parser = parse_duration_str)]
    pub until: Option<std::time::Duration>,
    /// Minimum log level to display (error, warn, info, debug, trace, stdout)
    #[clap(
        long,
        value_name = "LEVEL",
        default_value = "stdout",
        env = "DORA_LOG_LEVEL"
    )]
    #[arg(value_parser = parse_log_level_str)]
    pub level: dora_core::build::LogLevelOrStdout,
    /// Output format for log messages
    #[clap(long, default_value = "pretty", env = "DORA_LOG_FORMAT")]
    pub log_format: LogFormat,
    /// Per-node log level filter (e.g. "sensor=debug,processor=warn")
    #[clap(long, value_name = "FILTER", env = "DORA_LOG_FILTER")]
    pub log_filter: Option<String>,
    /// Filter logs by text pattern (case-insensitive substring match)
    #[clap(long, value_name = "PATTERN")]
    pub grep: Option<String>,
    #[clap(flatten)]
    pub coordinator: CoordinatorOptions,
}

fn build_log_config(args: &LogsArgs) -> Result<LogOutputConfig> {
    let node_filters = match &args.log_filter {
        Some(filter) => parse_log_filter(filter)
            .map_err(|e| eyre::eyre!("failed to parse --log-filter: {e}"))?,
        None => Default::default(),
    };
    Ok(LogOutputConfig {
        min_level: args.level.clone(),
        format: args.log_format,
        node_filters,
        print_dataflow_id: false,
        print_daemon_name: false,
    })
}

impl Executable for LogsArgs {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        // --local always uses local file path
        if self.local {
            if self.follow {
                return follow_local_logs(&self);
            }
            return read_local_logs(&self);
        }

        // Single node via coordinator (unchanged)
        if let Some(ref _node) = self.node
            && !self.all_nodes
        {
            let node = self.node.clone().unwrap();
            let config = build_log_config(&self)?;
            let session = self.coordinator.connect()?;
            let uuid = resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;
            return logs(
                &session,
                uuid,
                node,
                self.tail,
                self.follow,
                self.grep.as_deref(),
                &self.level,
                self.since,
                self.until,
                &config,
            );
        }

        // All nodes (explicit --all-nodes or no node specified):
        // Try coordinator first, fall back to local
        match self.coordinator.connect() {
            Ok(session) => {
                let uuid =
                    resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;
                let config = build_log_config(&self)?;
                stream_logs_from_coordinator(
                    &session,
                    uuid,
                    &self.level,
                    self.since,
                    self.until,
                    self.grep.as_deref(),
                    &config,
                )
            }
            Err(_) => {
                // Coordinator unavailable, fall back to local
                read_local_logs(&self)
            }
        }
    }
}

fn read_local_logs(args: &LogsArgs) -> Result<()> {
    let out_dir = Path::new("out");
    if !out_dir.exists() {
        bail!(
            "no out/ directory found in current directory\n\n  \
             hint: local logs are stored in ./out/ when running with `dora run`.\n  \
             For remote dataflows, connect to the coordinator with `dora logs -d <DATAFLOW>`"
        );
    }

    // Find the dataflow directory (most recent if not specified)
    let dataflow_dir = find_dataflow_dir(out_dir, args.dataflow.as_deref())?;

    let config = build_log_config(args)?;
    let now = Utc::now();

    let log_files = match &args.node {
        Some(node) if !args.all_nodes => find_node_log_files(&dataflow_dir, node)?,
        _ => find_log_files(&dataflow_dir)?,
    };
    if log_files.is_empty() {
        bail!(
            "no log files found in {}\n\n  \
             hint: the dataflow may not have produced any logs yet, \
             or node logging may be disabled",
            dataflow_dir.display()
        );
    }

    let mut all_messages: Vec<LogMessage> = Vec::new();
    for path in &log_files {
        all_messages.extend(read_log_file(path)?);
    }
    all_messages.sort_by(|a, b| a.timestamp.cmp(&b.timestamp));
    let filtered = apply_time_filters(all_messages, args.since, args.until, now);
    let grepped = apply_grep(filtered, args.grep.as_deref());
    let display = apply_tail(grepped, args.tail);

    for msg in display {
        print_log_message(msg, &config);
    }

    Ok(())
}

fn follow_local_logs(args: &LogsArgs) -> Result<()> {
    let out_dir = Path::new("out");
    if !out_dir.exists() {
        bail!(
            "no out/ directory found in current directory\n\n  \
             hint: local logs are stored in ./out/ when running with `dora run`.\n  \
             For remote dataflows, connect to the coordinator with `dora logs -d <DATAFLOW>`"
        );
    }

    let dataflow_dir = find_dataflow_dir(out_dir, args.dataflow.as_deref())?;
    let config = build_log_config(args)?;
    let now = Utc::now();

    let files = match &args.node {
        Some(node) if !args.all_nodes => find_node_log_files(&dataflow_dir, node)?,
        _ => find_log_files(&dataflow_dir)?,
    };

    if files.is_empty() {
        bail!(
            "no log files found in {}\n\n  \
             hint: the dataflow may not have produced any logs yet, \
             or node logging may be disabled",
            dataflow_dir.display()
        );
    }

    // Print existing content with filters
    let mut all_messages: Vec<LogMessage> = Vec::new();
    for path in &files {
        all_messages.extend(read_log_file(path)?);
    }
    all_messages.sort_by(|a, b| a.timestamp.cmp(&b.timestamp));
    let filtered = apply_time_filters(all_messages, args.since, args.until, now);
    let grepped = apply_grep(filtered, args.grep.as_deref());
    let display = apply_tail(grepped, args.tail);

    for msg in display {
        print_log_message(msg, &config);
    }

    // Track file byte offsets (start after existing content)
    let mut file_positions: HashMap<PathBuf, u64> = HashMap::new();
    for path in &files {
        let len = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
        file_positions.insert(path.clone(), len);
    }

    // Follow loop: poll for new content
    loop {
        std::thread::sleep(std::time::Duration::from_millis(200));

        let mut new_messages = Vec::new();
        for path in &files {
            let pos = file_positions.get(path).copied().unwrap_or(0);
            let current_size = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
            if current_size <= pos {
                continue;
            }
            let mut file = std::fs::File::open(path)?;
            file.seek(std::io::SeekFrom::Start(pos))?;
            let mut buf = String::new();
            file.read_to_string(&mut buf)?;
            for line in buf.lines() {
                if let Some(msg) = parse_jsonl_line(line) {
                    new_messages.push(msg);
                }
            }
            file_positions.insert(path.clone(), current_size);
        }

        new_messages.sort_by(|a, b| a.timestamp.cmp(&b.timestamp));
        for msg in new_messages {
            if matches_grep(&msg, args.grep.as_deref()) {
                print_log_message(msg, &config);
            }
        }
    }
}

fn find_dataflow_dir(out_dir: &Path, dataflow_id: Option<&str>) -> Result<PathBuf> {
    if let Some(id) = dataflow_id {
        let dir = out_dir.join(id);
        // Validate the resolved path stays within out_dir
        let canonical = dunce::canonicalize(&dir).wrap_err_with(|| {
            format!(
                "dataflow directory not found: {}\n\n  \
                     hint: use `dora list` to see running dataflows and their IDs",
                dir.display()
            )
        })?;
        let canonical_base =
            dunce::canonicalize(out_dir).wrap_err("failed to canonicalize out/ directory")?;
        if !canonical.starts_with(&canonical_base) {
            bail!("invalid dataflow identifier: path traversal detected");
        }
        return Ok(canonical);
    }

    // Find the most recent dataflow directory by modification time
    let mut entries: Vec<_> = std::fs::read_dir(out_dir)
        .wrap_err("failed to read out/ directory")?
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().map(|t| t.is_dir()).unwrap_or(false))
        .collect();

    if entries.is_empty() {
        bail!(
            "no dataflow directories found in out/\n\n  \
             hint: run a dataflow first with `dora run <DATAFLOW.yml>`"
        );
    }

    entries.sort_by(|a, b| {
        let a_time = a
            .metadata()
            .and_then(|m| m.modified())
            .unwrap_or(std::time::UNIX_EPOCH);
        let b_time = b
            .metadata()
            .and_then(|m| m.modified())
            .unwrap_or(std::time::UNIX_EPOCH);
        b_time.cmp(&a_time)
    });

    Ok(entries[0].path())
}

fn find_log_files(dataflow_dir: &Path) -> Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    for entry in std::fs::read_dir(dataflow_dir)
        .wrap_err_with(|| format!("failed to read {}", dataflow_dir.display()))?
    {
        let entry = entry?;
        let path = entry.path();
        let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
        if name.starts_with("log_") && (name.ends_with(".jsonl") || name.ends_with(".txt")) {
            files.push(path);
        }
    }
    // Sort so rotated files (older) come before current files
    files.sort_by(|a, b| {
        let a_idx = rotation_index(a);
        let b_idx = rotation_index(b);
        // Higher rotation index = older file, should come first
        b_idx.cmp(&a_idx)
    });
    Ok(files)
}

/// Extract rotation index from a log filename. Current file returns 0, `.1.jsonl` returns 1, etc.
fn rotation_index(path: &Path) -> u32 {
    let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
    // Pattern: log_<node>.<N>.jsonl
    if let Some(rest) = name.strip_prefix("log_")
        && let Some(rest) = rest.strip_suffix(".jsonl")
    {
        // Check if the last segment after the last '.' is a number
        if let Some(dot_pos) = rest.rfind('.')
            && let Ok(idx) = rest[dot_pos + 1..].parse::<u32>()
        {
            return idx;
        }
    }
    0 // current file
}

/// Find all log files for a node (including rotated), oldest first.
fn find_node_log_files(dataflow_dir: &Path, node: &NodeId) -> Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    let node_str = node.to_string();

    for entry in std::fs::read_dir(dataflow_dir)
        .wrap_err_with(|| format!("failed to read {}", dataflow_dir.display()))?
    {
        let entry = entry?;
        let name = entry.file_name().to_str().unwrap_or_default().to_string();
        // Match: log_<node>.jsonl, log_<node>.1.jsonl, log_<node>.txt
        let prefix = format!("log_{node_str}");
        if name.starts_with(&prefix) && (name.ends_with(".jsonl") || name.ends_with(".txt")) {
            files.push(entry.path());
        }
    }

    if files.is_empty() {
        bail!(
            "no log file found for node '{node}' in {}\n\n  \
             hint: check the node name is correct. \
             Use `dora node list` to see available nodes",
            dataflow_dir.display()
        );
    }

    // Sort: rotated (older) first, then current
    files.sort_by(|a, b| {
        let a_idx = rotation_index(a);
        let b_idx = rotation_index(b);
        b_idx.cmp(&a_idx)
    });
    Ok(files)
}

fn read_log_file(path: &Path) -> Result<Vec<LogMessage>> {
    let content = std::fs::read_to_string(path)
        .wrap_err_with(|| format!("failed to read {}", path.display()))?;

    let is_jsonl = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e == "jsonl")
        .unwrap_or(false);

    if is_jsonl {
        Ok(content
            .lines()
            .filter(|line| !line.trim().is_empty())
            .filter_map(parse_jsonl_line)
            .collect())
    } else {
        // Legacy .txt files: try to parse each line as JSON (LogMessage)
        // If that fails, treat as raw text
        let messages: Vec<LogMessage> = content
            .lines()
            .filter(|line| !line.trim().is_empty())
            .filter_map(parse_jsonl_line)
            .collect();

        if messages.is_empty() {
            // Raw text file, just print it directly
            std::io::stdout()
                .write_all(content.as_bytes())
                .wrap_err("failed to write to stdout")?;
            Ok(Vec::new())
        } else {
            Ok(messages)
        }
    }
}

fn apply_time_filters(
    messages: Vec<LogMessage>,
    since: Option<std::time::Duration>,
    until: Option<std::time::Duration>,
    now: DateTime<Utc>,
) -> Vec<LogMessage> {
    let since_threshold =
        since.and_then(|d| chrono::TimeDelta::from_std(d).ok().map(|td| now - td));
    let until_threshold =
        until.and_then(|d| chrono::TimeDelta::from_std(d).ok().map(|td| now - td));

    messages
        .into_iter()
        .filter(|msg| {
            if let Some(threshold) = since_threshold
                && msg.timestamp < threshold
            {
                return false;
            }
            if let Some(threshold) = until_threshold
                && msg.timestamp > threshold
            {
                return false;
            }
            true
        })
        .collect()
}

fn apply_grep(messages: Vec<LogMessage>, pattern: Option<&str>) -> Vec<LogMessage> {
    let Some(pattern) = pattern else {
        return messages;
    };
    messages
        .into_iter()
        .filter(|msg| matches_grep(msg, Some(pattern)))
        .collect()
}

fn apply_tail(messages: Vec<LogMessage>, tail: Option<usize>) -> Vec<LogMessage> {
    match tail {
        Some(n) => messages
            .into_iter()
            .rev()
            .take(n)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .collect(),
        None => messages,
    }
}

fn matches_grep(msg: &LogMessage, pattern: Option<&str>) -> bool {
    let Some(pattern) = pattern else { return true };
    let pattern_lower = pattern.to_lowercase();
    if msg.message.to_lowercase().contains(&pattern_lower) {
        return true;
    }
    if let Some(node) = &msg.node_id
        && node.to_string().to_lowercase().contains(&pattern_lower)
    {
        return true;
    }
    if let Some(target) = &msg.target
        && target.to_lowercase().contains(&pattern_lower)
    {
        return true;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_message::common::LogLevelOrStdout;
    use std::path::PathBuf;

    fn make_msg(
        message: &str,
        node: Option<&str>,
        target: Option<&str>,
        ts: DateTime<Utc>,
    ) -> LogMessage {
        LogMessage {
            build_id: None,
            dataflow_id: None,
            node_id: node.map(|n| NodeId::from(n.to_string())),
            daemon_id: None,
            level: LogLevelOrStdout::LogLevel(log::Level::Info),
            target: target.map(|t| t.to_string()),
            module_path: None,
            file: None,
            line: None,
            message: message.to_string(),
            timestamp: ts,
            fields: None,
        }
    }

    // --- rotation_index ---

    #[test]
    fn rotation_index_current_file() {
        assert_eq!(rotation_index(&PathBuf::from("log_sensor.jsonl")), 0);
    }

    #[test]
    fn rotation_index_rotated_1() {
        assert_eq!(rotation_index(&PathBuf::from("log_sensor.1.jsonl")), 1);
    }

    #[test]
    fn rotation_index_rotated_5() {
        assert_eq!(rotation_index(&PathBuf::from("log_sensor.5.jsonl")), 5);
    }

    #[test]
    fn rotation_index_txt_file() {
        assert_eq!(rotation_index(&PathBuf::from("log_sensor.txt")), 0);
    }

    // --- apply_time_filters ---

    #[test]
    fn time_filter_no_filters() {
        let now = Utc::now();
        let msgs = vec![make_msg("a", None, None, now)];
        let result = apply_time_filters(msgs.clone(), None, None, now);
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn time_filter_since_only() {
        let now = Utc::now();
        let old = now - chrono::TimeDelta::hours(2);
        let recent = now - chrono::TimeDelta::minutes(5);
        let msgs = vec![
            make_msg("old", None, None, old),
            make_msg("recent", None, None, recent),
        ];
        // since=1h -> only messages from last 1 hour
        let result =
            apply_time_filters(msgs, Some(std::time::Duration::from_secs(3600)), None, now);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].message, "recent");
    }

    #[test]
    fn time_filter_until_only() {
        let now = Utc::now();
        let old = now - chrono::TimeDelta::hours(2);
        let recent = now - chrono::TimeDelta::minutes(5);
        let msgs = vec![
            make_msg("old", None, None, old),
            make_msg("recent", None, None, recent),
        ];
        // until=1h -> only messages older than 1 hour
        let result =
            apply_time_filters(msgs, None, Some(std::time::Duration::from_secs(3600)), now);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].message, "old");
    }

    #[test]
    fn time_filter_since_and_until() {
        let now = Utc::now();
        let very_old = now - chrono::TimeDelta::hours(5);
        let mid = now - chrono::TimeDelta::hours(2);
        let recent = now - chrono::TimeDelta::minutes(5);
        let msgs = vec![
            make_msg("very_old", None, None, very_old),
            make_msg("mid", None, None, mid),
            make_msg("recent", None, None, recent),
        ];
        // since=3h, until=1h -> window between 3h and 1h ago
        let result = apply_time_filters(
            msgs,
            Some(std::time::Duration::from_secs(3 * 3600)),
            Some(std::time::Duration::from_secs(3600)),
            now,
        );
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].message, "mid");
    }

    // --- apply_grep ---

    #[test]
    fn grep_none_passes_all() {
        let now = Utc::now();
        let msgs = vec![
            make_msg("a", None, None, now),
            make_msg("b", None, None, now),
        ];
        assert_eq!(apply_grep(msgs, None).len(), 2);
    }

    #[test]
    fn grep_matches_message_case_insensitive() {
        let now = Utc::now();
        let msgs = vec![make_msg("Hello World", None, None, now)];
        assert_eq!(apply_grep(msgs, Some("hello")).len(), 1);
    }

    #[test]
    fn grep_matches_node_id() {
        let now = Utc::now();
        let msgs = vec![make_msg("msg", Some("sensor"), None, now)];
        assert_eq!(apply_grep(msgs, Some("sensor")).len(), 1);
    }

    #[test]
    fn grep_matches_target() {
        let now = Utc::now();
        let msgs = vec![make_msg("msg", None, Some("my_target"), now)];
        assert_eq!(apply_grep(msgs, Some("my_target")).len(), 1);
    }

    #[test]
    fn grep_no_match() {
        let now = Utc::now();
        let msgs = vec![make_msg("hello", Some("sensor"), Some("target"), now)];
        assert_eq!(apply_grep(msgs, Some("zzz_missing")).len(), 0);
    }

    // --- apply_tail ---

    #[test]
    fn tail_none_returns_all() {
        let now = Utc::now();
        let msgs = vec![
            make_msg("a", None, None, now),
            make_msg("b", None, None, now),
        ];
        assert_eq!(apply_tail(msgs, None).len(), 2);
    }

    #[test]
    fn tail_3_on_5_returns_last_3() {
        let now = Utc::now();
        let msgs: Vec<_> = (0..5)
            .map(|i| make_msg(&i.to_string(), None, None, now))
            .collect();
        let result = apply_tail(msgs, Some(3));
        assert_eq!(result.len(), 3);
        assert_eq!(result[0].message, "2");
        assert_eq!(result[2].message, "4");
    }

    #[test]
    fn tail_larger_than_count_returns_all() {
        let now = Utc::now();
        let msgs = vec![make_msg("a", None, None, now)];
        assert_eq!(apply_tail(msgs, Some(100)).len(), 1);
    }

    // --- matches_grep ---

    #[test]
    fn matches_grep_none_returns_true() {
        let now = Utc::now();
        let msg = make_msg("anything", None, None, now);
        assert!(matches_grep(&msg, None));
    }

    #[test]
    fn matches_grep_in_message() {
        let now = Utc::now();
        let msg = make_msg("sensor reading: 42", None, None, now);
        assert!(matches_grep(&msg, Some("reading")));
    }

    #[test]
    fn matches_grep_in_node_id() {
        let now = Utc::now();
        let msg = make_msg("data", Some("camera_front"), None, now);
        assert!(matches_grep(&msg, Some("camera")));
    }

    #[test]
    fn matches_grep_in_target() {
        let now = Utc::now();
        let msg = make_msg("data", None, Some("dora::runtime"), now);
        assert!(matches_grep(&msg, Some("runtime")));
    }

    #[test]
    fn matches_grep_no_match() {
        let now = Utc::now();
        let msg = make_msg("hello", Some("sensor"), Some("target"), now);
        assert!(!matches_grep(&msg, Some("zzz_missing")));
    }
}

/// Subscribe to coordinator log stream with time/grep filtering.
fn stream_logs_from_coordinator(
    session: &WsSession,
    uuid: Uuid,
    level: &dora_core::build::LogLevelOrStdout,
    since: Option<std::time::Duration>,
    until: Option<std::time::Duration>,
    grep: Option<&str>,
    config: &LogOutputConfig,
) -> Result<()> {
    let log_level = match level {
        dora_core::build::LogLevelOrStdout::Stdout => log::LevelFilter::Trace,
        dora_core::build::LogLevelOrStdout::LogLevel(l) => l.to_level_filter(),
    };

    let now = Utc::now();
    let since_threshold =
        since.and_then(|d| chrono::TimeDelta::from_std(d).ok().map(|td| now - td));
    let until_threshold =
        until.and_then(|d| chrono::TimeDelta::from_std(d).ok().map(|td| now - td));

    let log_rx = session.subscribe_logs(
        &serde_json::to_vec(&ControlRequest::LogSubscribe {
            dataflow_id: uuid,
            level: log_level,
        })
        .wrap_err("failed to serialize message")?,
    )?;

    while let Ok(raw) = log_rx.recv() {
        let raw = match raw {
            Ok(bytes) => bytes,
            Err(err) => {
                tracing::warn!("log stream error: {err:?}");
                continue;
            }
        };
        let parsed: eyre::Result<LogMessage> =
            serde_json::from_slice(&raw).context("failed to parse log message");
        match parsed {
            Ok(log_message) => {
                if let Some(threshold) = since_threshold
                    && log_message.timestamp < threshold
                {
                    continue;
                }
                if let Some(threshold) = until_threshold
                    && log_message.timestamp > threshold
                {
                    continue;
                }
                if matches_grep(&log_message, grep) {
                    print_log_message(log_message, config);
                }
            }
            Err(err) => {
                tracing::warn!("failed to parse log message: {err:?}")
            }
        }
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub fn logs(
    session: &WsSession,
    uuid: Uuid,
    node: NodeId,
    tail: Option<usize>,
    follow: bool,
    grep: Option<&str>,
    level: &dora_core::build::LogLevelOrStdout,
    since: Option<std::time::Duration>,
    until: Option<std::time::Duration>,
    config: &LogOutputConfig,
) -> Result<()> {
    let logs = {
        let reply = send_control_request(
            session,
            &ControlRequest::Logs {
                uuid: Some(uuid),
                name: None,
                node: node.to_string(),
                tail: if since.is_some() || until.is_some() || grep.is_some() {
                    // Fetch all logs when filtering client-side, apply tail after
                    None
                } else {
                    tail
                },
            },
        )?;
        expect_reply!(reply, Logs(data))?
    };

    // Unified filter pipeline: parse -> time_filter -> grep -> tail -> print
    let now = Utc::now();
    let content = String::from_utf8_lossy(&logs);
    let messages: Vec<LogMessage> = content.lines().filter_map(parse_jsonl_line).collect();
    let filtered = apply_time_filters(messages, since, until, now);
    let grepped = apply_grep(filtered, grep);
    let display = apply_tail(grepped, tail);
    for msg in display {
        print_log_message(msg, config);
    }

    if !follow {
        return Ok(());
    }

    stream_logs_from_coordinator(session, uuid, level, since, until, grep, config)
}
