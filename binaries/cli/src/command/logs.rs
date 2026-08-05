use std::{
    collections::HashMap,
    io::{Read, Seek, Write},
    path::{Path, PathBuf},
};

use super::{Executable, default_tracing};
use crate::common::parse_duration;
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
use eyre::{Context, Result, bail};
use uuid::Uuid;

#[derive(Debug, Args)]
/// Show logs of a given dataflow.
pub struct LogsArgs {
    /// Identifier of the dataflow (UUID or name)
    #[clap(value_name = "UUID_OR_NAME")]
    pub dataflow: Option<String>,
    /// Deprecated positional node name. Use --node instead.
    #[clap(value_name = "NAME", hide = true, conflicts_with_all = ["node", "all_nodes"])]
    pub legacy_node: Option<NodeId>,
    /// Show logs for the given node
    #[clap(long, short = 'n', value_name = "NAME", conflicts_with = "all_nodes")]
    pub node: Option<NodeId>,
    /// Show logs from all nodes merged by timestamp.
    /// Streams from coordinator by default, falls back to local out/ directory.
    #[clap(long, conflicts_with = "node")]
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
    #[arg(value_parser = parse_duration)]
    pub since: Option<std::time::Duration>,
    /// Only show logs older than this duration ago (e.g. "5m", "1h")
    #[clap(long, value_name = "DURATION")]
    #[arg(value_parser = parse_duration)]
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
    ///
    /// `json` emits JSON Lines (one object per log message).
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

        reject_legacy_node_arg(&self)?;

        // --local always uses local file path
        if self.local {
            if self.follow {
                return follow_local_logs(&self);
            }
            return read_local_logs(&self);
        }

        // Single node via coordinator
        if let Some(ref node) = self.node {
            let node = node.clone();
            let config = build_log_config(&self)?;
            let session = self.coordinator.connect()?;
            let uuid = resolve_logs_dataflow_identifier(&session, self.dataflow.as_deref(), None)?;
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
                let uuid = resolve_logs_dataflow_identifier(
                    &session,
                    self.dataflow.as_deref(),
                    legacy_positional_node(&self),
                )?;
                let config = build_log_config(&self)?;
                all_nodes_logs_from_coordinator(&session, uuid, &self, &config)?;
                if !self.follow {
                    return Ok(());
                }
                stream_logs_from_coordinator(
                    &session,
                    uuid,
                    None,
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

fn reject_legacy_node_arg(args: &LogsArgs) -> Result<()> {
    if let Some(node) = &args.legacy_node {
        let dataflow = args.dataflow.as_deref().unwrap_or("<DATAFLOW>");
        bail!(
            "positional node argument `{node}` is no longer supported\n\n  \
             hint: use `dora logs {dataflow} --node {node}` instead"
        );
    }
    Ok(())
}

fn legacy_positional_node(args: &LogsArgs) -> Option<&str> {
    args.dataflow
        .as_deref()
        .filter(|_| args.node.is_none() && !args.all_nodes)
}

fn resolve_logs_dataflow_identifier(
    session: &WsSession,
    dataflow: Option<&str>,
    legacy_node_hint: Option<&str>,
) -> Result<Uuid> {
    match resolve_dataflow_identifier_interactive(session, dataflow) {
        Ok(uuid) => Ok(uuid),
        Err(err) if legacy_node_hint.is_some() => {
            let node = legacy_node_hint.unwrap();
            Err(err).wrap_err_with(|| {
                format!(
                    "failed to resolve dataflow `{node}`\n\n  \
                     hint: if you intended `{node}` as a node name, use `dora logs --node {node}`"
                )
            })
        }
        Err(err) => Err(err),
    }
}

fn find_logs_dataflow_dir(out_dir: &Path, args: &LogsArgs) -> Result<PathBuf> {
    match find_dataflow_dir(out_dir, args.dataflow.as_deref()) {
        Ok(dir) => Ok(dir),
        Err(err) if legacy_positional_node(args).is_some() => {
            let node = legacy_positional_node(args).unwrap();
            Err(err).wrap_err_with(|| {
                format!(
                    "failed to resolve local log dataflow `{node}`\n\n  \
                     hint: if you intended `{node}` as a node name, use `dora logs --local --node {node}`"
                )
            })
        }
        Err(err) => Err(err),
    }
}

fn read_local_logs(args: &LogsArgs) -> Result<()> {
    let out_dir = Path::new("out");
    if !out_dir.exists() {
        bail!(
            "no out/ directory found in current directory\n\n  \
             hint: local logs are stored in ./out/ when running with `dora run`.\n  \
             For remote dataflows, connect to the coordinator with `dora logs <DATAFLOW>`"
        );
    }

    // Find the dataflow directory (most recent if not specified)
    let dataflow_dir = find_logs_dataflow_dir(out_dir, args)?;

    let config = build_log_config(args)?;
    let now = Utc::now();

    let log_files = match &args.node {
        Some(node) => find_node_log_files(&dataflow_dir, node)?,
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
    all_messages.sort_by_key(|a| a.timestamp);
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
             For remote dataflows, connect to the coordinator with `dora logs <DATAFLOW>`"
        );
    }

    let dataflow_dir = find_logs_dataflow_dir(out_dir, args)?;
    let config = build_log_config(args)?;
    let now = Utc::now();

    let files = match &args.node {
        Some(node) => find_node_log_files(&dataflow_dir, node)?,
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
    all_messages.sort_by_key(|a| a.timestamp);
    let filtered = apply_time_filters(all_messages, args.since, args.until, now);
    let grepped = apply_grep(filtered, args.grep.as_deref());
    let display = apply_tail(grepped, args.tail);

    for msg in display {
        print_log_message(msg, &config);
    }

    // Track per-file follow state (offset already consumed + a head fingerprint
    // to recognise the same file across a rotation rename). Start after the
    // existing content printed above.
    let mut follow_state: HashMap<PathBuf, FollowedFile> = HashMap::new();
    for path in &files {
        let size = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
        follow_state.insert(
            path.clone(),
            FollowedFile {
                offset: size,
                head: read_log_head(path),
            },
        );
    }

    // Follow loop: poll for new content
    loop {
        std::thread::sleep(std::time::Duration::from_millis(200));

        // Re-glob every poll so files that appear after the follow started —
        // in particular a freshly rotated `log_<node>.1.jsonl` — are picked up.
        // A transient read error just skips this poll and keeps the state.
        let files = match &args.node {
            Some(node) => find_node_log_files(&dataflow_dir, node),
            None => find_log_files(&dataflow_dir),
        };
        let files = match files {
            Ok(files) => files,
            Err(_) => continue,
        };

        let snapshot: Vec<LogFileSnapshot> = files
            .iter()
            .map(|path| {
                let size = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
                LogFileSnapshot {
                    path: path.clone(),
                    size,
                    head: read_log_head(path),
                }
            })
            .collect();

        let mut new_messages = Vec::new();
        let mut next_state: HashMap<PathBuf, FollowedFile> = HashMap::new();
        for plan in plan_follow_reads(&follow_state, &snapshot) {
            let (msgs, new_pos) = if plan.size > plan.resume {
                read_appended_log_lines(&plan.path, plan.resume)?
            } else {
                (Vec::new(), plan.resume)
            };
            new_messages.extend(msgs);
            next_state.insert(
                plan.path,
                FollowedFile {
                    offset: new_pos,
                    head: plan.head,
                },
            );
        }
        follow_state = next_state;

        new_messages.sort_by_key(|a| a.timestamp);
        for msg in new_messages {
            if matches_grep(&msg, args.grep.as_deref()) {
                print_log_message(msg, &config);
            }
        }
    }
}

/// Number of leading bytes of a log file used as a rename/replacement
/// fingerprint (see [`plan_follow_reads`]). Large enough to span the first
/// JSONL line — whose timestamp makes it distinct per file — but small enough
/// to read cheaply on every poll.
const LOG_HEAD_FINGERPRINT_LEN: usize = 256;

/// Per-file follow state: the byte offset already consumed and a fingerprint of
/// the file's leading bytes used to recognise the same file across a rotation
/// rename.
#[derive(Clone, Debug, PartialEq)]
struct FollowedFile {
    offset: u64,
    head: Vec<u8>,
}

/// A poll-time observation of a log file.
struct LogFileSnapshot {
    path: PathBuf,
    size: u64,
    head: Vec<u8>,
}

/// A per-file decision produced by [`plan_follow_reads`].
struct FollowPlan {
    path: PathBuf,
    /// Byte offset to resume reading from.
    resume: u64,
    size: u64,
    head: Vec<u8>,
}

/// Read up to [`LOG_HEAD_FINGERPRINT_LEN`] leading bytes of `path`, used as a
/// content fingerprint. Returns an empty vec on any error (treated as "could
/// not fingerprint", which never matches another file).
fn read_log_head(path: &Path) -> Vec<u8> {
    let mut buf = vec![0u8; LOG_HEAD_FINGERPRINT_LEN];
    match std::fs::File::open(path).and_then(|mut f| f.read(&mut buf)) {
        Ok(n) => {
            buf.truncate(n);
            buf
        }
        Err(_) => Vec::new(),
    }
}

/// Whether two head fingerprints identify the same underlying file. A followed
/// file only ever grows, extending its head, so a shorter recorded head that is
/// a prefix of the current one is the same file; a rotated-in fresh file starts
/// with a different first line. Two empty heads never match — a fingerprint we
/// could not read must not be mistaken for another unreadable file.
fn same_log_file(a: &[u8], b: &[u8]) -> bool {
    let n = a.len().min(b.len());
    n > 0 && a[..n] == b[..n]
}

/// Decide, for each file in `snapshot`, the byte offset to resume reading from,
/// given the previous poll's `prev` state.
///
/// The daemon rotates logs by *renaming* (`log_<node>.jsonl` ->
/// `log_<node>.1.jsonl`, shifting older ones up) and creating a fresh current
/// file — see `binaries/daemon/src/log.rs`. Two failure modes this avoids:
///
/// 1. A fresh file that grew past the stale offset within a single poll: a
///    size-only check would seek into it mid-line and garble the output. The
///    head fingerprint differs, so it is recognised as a new file and read
///    from 0.
/// 2. The rotated file's unread tail (bytes appended to the current file
///    between the last poll and the rename) being dropped: the carried offset
///    follows the content to its new path — matched by fingerprint — so the
///    tail is drained exactly once before the fresh file is read from 0.
fn plan_follow_reads(
    prev: &HashMap<PathBuf, FollowedFile>,
    snapshot: &[LogFileSnapshot],
) -> Vec<FollowPlan> {
    snapshot
        .iter()
        .map(|snap| {
            let (resume, head) = match prev.get(&snap.path) {
                // Couldn't re-fingerprint (transient read error): assume the
                // same file and keep the recorded head so a later poll can
                // still match it — never re-read from 0 and duplicate.
                Some(state) if snap.head.is_empty() => {
                    (state.offset.min(snap.size), state.head.clone())
                }
                // Same file still at this path: resume where we left off, unless
                // it was truncated in place (size shrank below the offset).
                Some(state) if same_log_file(&state.head, &snap.head) => {
                    (state.offset.min(snap.size), snap.head.clone())
                }
                // A new path, or the content at this path changed (rotation).
                // Follow the content by fingerprint from wherever it last was to
                // drain its unread tail; otherwise it is genuinely new — read
                // from the start.
                _ => {
                    let carried = if snap.head.is_empty() {
                        None
                    } else {
                        prev.iter()
                            .find(|(path, state)| {
                                *path != &snap.path && same_log_file(&state.head, &snap.head)
                            })
                            .map(|(_, state)| state.offset.min(snap.size))
                    };
                    (carried.unwrap_or(0), snap.head.clone())
                }
            };
            FollowPlan {
                path: snap.path.clone(),
                resume,
                size: snap.size,
                head,
            }
        })
        .collect()
}

/// Read log lines appended to `path` after byte offset `pos`, returning the
/// parsed messages and the new byte offset.
///
/// Only *complete* (newline-terminated) lines are consumed. A trailing partial
/// line — the daemon may be mid-write — is left unconsumed so it is re-read once
/// completed, rather than being parsed as a broken JSON line and dropped. The
/// returned offset advances only past the bytes actually consumed, never past
/// the (racily larger) file size, so nothing is re-read and duplicated on the
/// next poll. Bytes are decoded lossily so a read that ends inside a multibyte
/// UTF-8 sequence cannot abort the follow session.
fn read_appended_log_lines(path: &Path, pos: u64) -> Result<(Vec<LogMessage>, u64)> {
    let mut file = std::fs::File::open(path)?;
    file.seek(std::io::SeekFrom::Start(pos))?;
    let mut buf = Vec::new();
    file.read_to_end(&mut buf)?;
    let consumed = match buf.iter().rposition(|&b| b == b'\n') {
        Some(idx) => idx + 1,
        None => return Ok((Vec::new(), pos)),
    };
    let text = String::from_utf8_lossy(&buf[..consumed]);
    let messages = text.lines().filter_map(parse_jsonl_line).collect();
    Ok((messages, pos + consumed as u64))
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
        let stem = format!("log_{node_str}");
        let rest = match name.strip_prefix(&stem) {
            Some(rest) => rest,
            None => continue,
        };
        if rest.starts_with('.') && (rest.ends_with(".jsonl") || rest.ends_with(".txt")) {
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

/// Subscribe to coordinator log stream with time/grep filtering.
/// Fetches historical logs of every node in the dataflow, merges them by
/// timestamp, and prints them through the shared filter pipeline.
fn all_nodes_logs_from_coordinator(
    session: &WsSession,
    uuid: Uuid,
    args: &LogsArgs,
    config: &LogOutputConfig,
) -> Result<()> {
    let reply = send_control_request(session, &ControlRequest::GetNodeInfo)?;
    let nodes = expect_reply!(reply, NodeInfoList(data))?;
    let node_ids: Vec<NodeId> = nodes
        .into_iter()
        .filter(|info| info.dataflow_id == uuid)
        .map(|info| info.node_id)
        .collect();

    // GetNodeInfo only covers *running* dataflows. For a finished/archived
    // dataflow the node list is empty — try the descriptor of a still-running
    // dataflow first, otherwise fall back to the local out/ directory (the
    // same fallback used when no coordinator is reachable) so post-mortem
    // `dora logs` still shows something instead of silently printing nothing.
    let node_ids = if node_ids.is_empty() {
        let info = send_control_request(
            session,
            &ControlRequest::Info {
                dataflow_uuid: uuid,
            },
        )
        .and_then(|reply| expect_reply!(reply, DataflowInfo { descriptor }));
        match info {
            Ok(descriptor) => descriptor.nodes.into_iter().map(|node| node.id).collect(),
            Err(_) => {
                eprintln!(
                    "note: dataflow `{uuid}` is not running — reading logs from the local out/ \
                     directory.\n  hint: the coordinator still serves archived logs per node: \
                     `dora logs {uuid} --node <NAME>`"
                );
                return read_local_logs(args);
            }
        }
    } else {
        node_ids
    };

    let mut messages = Vec::new();
    for node in node_ids {
        // One unreachable node (e.g. a disconnected daemon in a multi-machine
        // dataflow) must not discard the logs of the healthy nodes.
        let logs = send_control_request(
            session,
            &ControlRequest::Logs {
                uuid: Some(uuid),
                name: None,
                node: node.to_string(),
                // Tail must apply to the merged stream, so fetch everything
                // per node and trim after sorting.
                tail: None,
            },
        )
        .and_then(|reply| expect_reply!(reply, Logs(data)));
        match logs {
            Ok(logs) => {
                let content = String::from_utf8_lossy(&logs);
                messages.extend(content.lines().filter_map(parse_jsonl_line));
            }
            Err(err) => {
                eprintln!("warning: could not fetch logs for node `{node}`: {err}");
            }
        }
    }
    messages.sort_by_key(|msg| msg.timestamp);

    let now = Utc::now();
    let filtered = apply_time_filters(messages, args.since, args.until, now);
    let grepped = apply_grep(filtered, args.grep.as_deref());
    let display = apply_tail(grepped, args.tail);
    for msg in display {
        print_log_message(msg, config);
    }
    Ok(())
}

/// Returns whether a log message should be kept given an optional node filter.
/// `None` means no filtering (messages from every node pass); `Some(want)`
/// keeps only messages whose node id equals `want`.
fn matches_node_filter(msg_node: Option<&str>, want: Option<&str>) -> bool {
    match want {
        None => true,
        Some(want) => msg_node == Some(want),
    }
}

/// Subscribe to coordinator log stream with time/grep/node filtering.
///
/// `node`, when set, restricts the stream to messages from that single node —
/// mirroring the node scoping already applied to the historical fetch in
/// [`logs`]. Without this, `--node <N> --follow` would show history for `N`
/// but then stream live logs from every node once following began.
#[allow(clippy::too_many_arguments)]
fn stream_logs_from_coordinator(
    session: &WsSession,
    uuid: Uuid,
    node: Option<&NodeId>,
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
    let want_node = node.map(|n| n.as_ref());

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
                if !matches_node_filter(log_message.node_id.as_ref().map(|n| n.as_ref()), want_node)
                {
                    continue;
                }
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

    stream_logs_from_coordinator(
        session,
        uuid,
        Some(&node),
        level,
        since,
        until,
        grep,
        config,
    )
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

    // --- matches_node_filter ---

    #[test]
    fn node_filter_none_passes_everything() {
        assert!(matches_node_filter(Some("sensor"), None));
        assert!(matches_node_filter(None, None));
    }

    #[test]
    fn node_filter_matching_node_passes() {
        assert!(matches_node_filter(Some("sensor"), Some("sensor")));
    }

    #[test]
    fn node_filter_other_node_is_dropped() {
        assert!(!matches_node_filter(Some("processor"), Some("sensor")));
    }

    #[test]
    fn node_filter_system_message_is_dropped_when_node_requested() {
        // System messages (no node_id) should not leak through a --node filter.
        assert!(!matches_node_filter(None, Some("sensor")));
    }

    #[test]
    fn find_node_log_files_does_not_match_prefix_sharing_nodes() {
        use std::fs::File;
        use tempfile::tempdir;

        let dir = tempdir().unwrap();
        File::create(dir.path().join("log_cam.jsonl")).unwrap();
        File::create(dir.path().join("log_cam_left.jsonl")).unwrap();

        let node_id = NodeId::from("cam".to_string());
        let files = find_node_log_files(dir.path(), &node_id).unwrap();

        assert_eq!(files.len(), 1);
        assert!(files[0].file_name().unwrap() == "log_cam.jsonl");
    }

    // --- follow-mode rotation planning (plan_follow_reads / same_log_file) ---

    fn followed(offset: u64, head: &[u8]) -> FollowedFile {
        FollowedFile {
            offset,
            head: head.to_vec(),
        }
    }

    fn snap(path: &str, size: u64, head: &[u8]) -> LogFileSnapshot {
        LogFileSnapshot {
            path: PathBuf::from(path),
            size,
            head: head.to_vec(),
        }
    }

    fn resume_for<'a>(plans: &'a [FollowPlan], path: &str) -> &'a FollowPlan {
        plans
            .iter()
            .find(|p| p.path == PathBuf::from(path))
            .expect("plan for path")
    }

    #[test]
    fn same_log_file_matches_growing_prefix_only() {
        // A followed file only ever grows, extending its head — a recorded head
        // that is a prefix of the current one is the same file.
        assert!(same_log_file(b"line1\n", b"line1\nline2\n"));
        assert!(same_log_file(b"line1\nline2\n", b"line1\n"));
        // A rotated-in fresh file starts with a different first line.
        assert!(!same_log_file(b"old-first\n", b"new-first\n"));
        // Two unreadable (empty) fingerprints must never be treated as equal.
        assert!(!same_log_file(b"", b""));
    }

    #[test]
    fn plan_resumes_from_offset_on_plain_append() {
        let mut prev = HashMap::new();
        prev.insert(PathBuf::from("log_n.jsonl"), followed(10, b"AAAA"));
        let plans = plan_follow_reads(&prev, &[snap("log_n.jsonl", 25, b"AAAAmore")]);
        assert_eq!(resume_for(&plans, "log_n.jsonl").resume, 10);
    }

    #[test]
    fn plan_reads_brand_new_file_from_start() {
        let prev = HashMap::new();
        let plans = plan_follow_reads(&prev, &[snap("log_n.jsonl", 5, b"X")]);
        assert_eq!(resume_for(&plans, "log_n.jsonl").resume, 0);
    }

    #[test]
    fn plan_rotation_drains_tail_and_reads_fresh_file_from_zero() {
        // Before: current file at offset 100 (head OLD). After rotation the old
        // content is at `.1.jsonl` (grown to 150 as the final flush landed) and
        // a fresh current file (head NEW) exists.
        let mut prev = HashMap::new();
        prev.insert(PathBuf::from("log_n.jsonl"), followed(100, b"OLDHEAD"));
        let plans = plan_follow_reads(
            &prev,
            &[
                snap("log_n.1.jsonl", 150, b"OLDHEAD"),
                snap("log_n.jsonl", 40, b"NEWHEAD"),
            ],
        );
        // The rotated file's unread tail [100..150) is drained exactly once.
        assert_eq!(resume_for(&plans, "log_n.1.jsonl").resume, 100);
        // The fresh file is read from the start, not the stale offset.
        assert_eq!(resume_for(&plans, "log_n.jsonl").resume, 0);
    }

    #[test]
    fn plan_fresh_file_grown_past_offset_is_not_garbled() {
        // Residual gap #1: the fresh current file grew past the stale offset
        // (200 > 100) within one poll. A size-only check would seek to 100 and
        // start mid-line; the differing fingerprint makes it a new file at 0.
        let mut prev = HashMap::new();
        prev.insert(PathBuf::from("log_n.jsonl"), followed(100, b"OLDHEAD"));
        let plans = plan_follow_reads(
            &prev,
            &[
                snap("log_n.1.jsonl", 150, b"OLDHEAD"),
                snap("log_n.jsonl", 200, b"NEWHEAD"),
            ],
        );
        assert_eq!(resume_for(&plans, "log_n.jsonl").resume, 0);
        assert_eq!(resume_for(&plans, "log_n.1.jsonl").resume, 100);
    }

    #[test]
    fn plan_unreadable_head_keeps_offset_instead_of_rereading() {
        // A transient failure to fingerprint (empty head) must not reset a
        // tracked file to 0 — that would re-read and duplicate its whole body.
        let mut prev = HashMap::new();
        prev.insert(PathBuf::from("log_n.jsonl"), followed(80, b"OLDHEAD"));
        let plans = plan_follow_reads(&prev, &[snap("log_n.jsonl", 80, b"")]);
        let plan = resume_for(&plans, "log_n.jsonl");
        assert_eq!(plan.resume, 80);
        // The recorded head is retained so the next poll can still match it.
        assert_eq!(plan.head, b"OLDHEAD");
    }

    #[test]
    fn plan_already_read_rotated_file_is_not_reread_on_shift() {
        // A `.1.jsonl` we had fully read (offset == len) shifts to `.2.jsonl`.
        // The carried offset follows it, so nothing is re-read.
        let mut prev = HashMap::new();
        prev.insert(PathBuf::from("log_n.1.jsonl"), followed(150, b"OLDHEAD"));
        let plans = plan_follow_reads(&prev, &[snap("log_n.2.jsonl", 150, b"OLDHEAD")]);
        let plan = resume_for(&plans, "log_n.2.jsonl");
        assert_eq!(plan.resume, 150);
        assert_eq!(plan.size, 150); // resume == size ⇒ nothing to read
    }

    // --- read_appended_log_lines (follow-mode offset tracking) ---

    fn jsonl(message: &str) -> String {
        let mut line = serde_json::to_string(&make_msg(message, None, None, Utc::now())).unwrap();
        line.push('\n');
        line
    }

    #[test]
    fn read_appended_reads_complete_lines_and_advances_to_end() {
        use tempfile::tempdir;

        let dir = tempdir().unwrap();
        let path = dir.path().join("log.jsonl");
        let content = format!("{}{}", jsonl("a"), jsonl("b"));
        std::fs::write(&path, &content).unwrap();

        let (msgs, new_pos) = read_appended_log_lines(&path, 0).unwrap();
        assert_eq!(msgs.len(), 2);
        assert_eq!(msgs[0].message, "a");
        assert_eq!(msgs[1].message, "b");
        // Offset advances to exactly the bytes consumed (whole file here).
        assert_eq!(new_pos, content.len() as u64);

        // A follow-up read from the returned offset yields nothing and does not
        // move the offset (no duplication).
        let (msgs, pos2) = read_appended_log_lines(&path, new_pos).unwrap();
        assert!(msgs.is_empty());
        assert_eq!(pos2, new_pos);
    }

    #[test]
    fn read_appended_leaves_trailing_partial_line_for_next_poll() {
        use tempfile::tempdir;

        let dir = tempdir().unwrap();
        let path = dir.path().join("log.jsonl");

        // First line complete, second line still being written (no newline).
        let complete = jsonl("first");
        let partial = {
            let mut p = complete.clone();
            let second =
                serde_json::to_string(&make_msg("second", None, None, Utc::now())).unwrap();
            p.push_str(&second); // no trailing '\n' yet
            p
        };
        std::fs::write(&path, &partial).unwrap();

        let (msgs, new_pos) = read_appended_log_lines(&path, 0).unwrap();
        assert_eq!(msgs.len(), 1, "partial line must not be parsed/emitted");
        assert_eq!(msgs[0].message, "first");
        // Offset stops at the last newline, not the (larger) file size.
        assert_eq!(new_pos, complete.len() as u64);

        // Daemon finishes the second line; re-reading from the saved offset now
        // yields exactly that line — the first line is not duplicated.
        std::fs::write(&path, format!("{complete}{}", jsonl("second"))).unwrap();
        let (msgs, _pos) = read_appended_log_lines(&path, new_pos).unwrap();
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].message, "second");
    }

    #[test]
    fn read_appended_does_not_abort_on_split_multibyte_utf8() {
        use tempfile::tempdir;

        let dir = tempdir().unwrap();
        let path = dir.path().join("log.jsonl");

        // One complete line, then a lone leading byte of a multibyte UTF-8
        // sequence with no newline (a write caught mid-flush). `read_to_string`
        // would return an InvalidData error here; the lossy path must not.
        let complete = jsonl("ok");
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(complete.as_bytes()).unwrap();
        f.write_all(&[0xE2]).unwrap(); // first byte of a 3-byte char, no newline
        drop(f);

        let (msgs, new_pos) = read_appended_log_lines(&path, 0).unwrap();
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].message, "ok");
        // The dangling partial byte is left unconsumed.
        assert_eq!(new_pos, complete.len() as u64);
    }
}
