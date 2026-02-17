use std::{
    collections::HashMap,
    io::{Read, Seek, Write},
    net::{SocketAddr, TcpStream},
    path::{Path, PathBuf},
};

use super::{Executable, default_tracing};
use crate::{
    common::{connect_to_coordinator, resolve_dataflow_identifier_interactive},
    output::{LogOutputConfig, parse_jsonl_line, print_log_message},
};
use adora_core::topics::{ADORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use adora_message::{
    cli_to_coordinator::ControlRequest, common::LogMessage,
    coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use chrono::{DateTime, Utc};
use clap::Args;
use communication_layer_request_reply::{TcpConnection, TcpRequestReplyConnection};
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
    /// Show logs from all nodes merged by timestamp
    #[clap(long)]
    pub all_nodes: bool,
    /// Number of lines to show from the end of the logs
    #[clap(long, short = 'n')]
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
    /// Filter logs by text pattern (case-insensitive substring match)
    #[clap(long, value_name = "PATTERN")]
    pub grep: Option<String>,
    /// Address of the adora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: std::net::IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = ADORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
}

impl Executable for LogsArgs {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        if self.local || self.all_nodes {
            if self.follow {
                return follow_local_logs(&self);
            }
            return read_local_logs(&self);
        }

        // Need a node for coordinator path
        let node = match self.node {
            Some(n) => n,
            None => bail!("node name is required (or use --local --all-nodes)"),
        };

        let mut session =
            connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
                .wrap_err("failed to connect to adora coordinator")?;
        let uuid =
            resolve_dataflow_identifier_interactive(&mut *session, self.dataflow.as_deref())?;
        logs(
            &mut *session,
            uuid,
            node,
            self.tail,
            self.follow,
            (self.coordinator_addr, self.coordinator_port).into(),
            self.grep.as_deref(),
        )
    }
}

fn read_local_logs(args: &LogsArgs) -> Result<()> {
    let out_dir = Path::new("out");
    if !out_dir.exists() {
        bail!("no out/ directory found in current directory");
    }

    // Find the dataflow directory (most recent if not specified)
    let dataflow_dir = find_dataflow_dir(out_dir, args.dataflow.as_deref())?;

    let config = LogOutputConfig::default();
    let now = Utc::now();

    if args.all_nodes || args.node.is_none() {
        // Read all log files and merge-sort
        let log_files = find_log_files(&dataflow_dir)?;
        if log_files.is_empty() {
            bail!("no log files found in {}", dataflow_dir.display());
        }

        let mut all_messages: Vec<LogMessage> = Vec::new();
        for path in &log_files {
            let messages = read_log_file(path)?;
            all_messages.extend(messages);
        }

        // Sort by timestamp
        all_messages.sort_by(|a, b| a.timestamp.cmp(&b.timestamp));

        // Apply time filters, then grep, then tail
        let filtered = apply_time_filters(all_messages, args.since, args.until, now);
        let grepped = apply_grep(filtered, args.grep.as_deref());
        let display = apply_tail(grepped, args.tail);

        for msg in display {
            print_log_message(msg, &config);
        }
    } else {
        let node = args.node.as_ref().unwrap();
        let log_file = find_node_log_file(&dataflow_dir, node)?;
        let messages = read_log_file(&log_file)?;
        let filtered = apply_time_filters(messages, args.since, args.until, now);
        let grepped = apply_grep(filtered, args.grep.as_deref());
        let display = apply_tail(grepped, args.tail);

        for msg in display {
            print_log_message(msg, &config);
        }
    }

    Ok(())
}

fn follow_local_logs(args: &LogsArgs) -> Result<()> {
    let out_dir = Path::new("out");
    if !out_dir.exists() {
        bail!("no out/ directory found in current directory");
    }

    let dataflow_dir = find_dataflow_dir(out_dir, args.dataflow.as_deref())?;
    let config = LogOutputConfig::default();
    let now = Utc::now();

    let files = if args.all_nodes || args.node.is_none() {
        find_log_files(&dataflow_dir)?
    } else {
        vec![find_node_log_file(&dataflow_dir, args.node.as_ref().unwrap())?]
    };

    if files.is_empty() {
        bail!("no log files found in {}", dataflow_dir.display());
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
        if dir.exists() {
            return Ok(dir);
        }
        bail!("dataflow directory not found: {}", dir.display());
    }

    // Find the most recent dataflow directory by modification time
    let mut entries: Vec<_> = std::fs::read_dir(out_dir)
        .wrap_err("failed to read out/ directory")?
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().map(|t| t.is_dir()).unwrap_or(false))
        .collect();

    if entries.is_empty() {
        bail!("no dataflow directories found in out/");
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
    Ok(files)
}

fn find_node_log_file(dataflow_dir: &Path, node: &NodeId) -> Result<PathBuf> {
    // Try .jsonl first, then .txt
    let jsonl = dataflow_dir.join(format!("log_{node}.jsonl"));
    if jsonl.exists() {
        return Ok(jsonl);
    }
    let txt = dataflow_dir.join(format!("log_{node}.txt"));
    if txt.exists() {
        return Ok(txt);
    }
    bail!(
        "no log file found for node '{node}' in {}",
        dataflow_dir.display()
    );
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
            .filter_map(|line| parse_jsonl_line(line))
            .collect())
    } else {
        // Legacy .txt files: try to parse each line as JSON (LogMessage)
        // If that fails, treat as raw text
        let messages: Vec<LogMessage> = content
            .lines()
            .filter(|line| !line.trim().is_empty())
            .filter_map(|line| parse_jsonl_line(line))
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
            if let Some(threshold) = since_threshold {
                if msg.timestamp < threshold {
                    return false;
                }
            }
            if let Some(threshold) = until_threshold {
                if msg.timestamp > threshold {
                    return false;
                }
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
    if let Some(node) = &msg.node_id {
        if node.to_string().to_lowercase().contains(&pattern_lower) {
            return true;
        }
    }
    if let Some(target) = &msg.target {
        if target.to_lowercase().contains(&pattern_lower) {
            return true;
        }
    }
    false
}

pub fn logs(
    session: &mut TcpRequestReplyConnection,
    uuid: Uuid,
    node: NodeId,
    tail: Option<usize>,
    follow: bool,
    coordinator_addr: SocketAddr,
    grep: Option<&str>,
) -> Result<()> {
    let logs = {
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Logs {
                    uuid: Some(uuid),
                    name: None,
                    node: node.to_string(),
                    tail,
                })
                .wrap_err("")?,
            )
            .wrap_err("failed to send Logs request message")?;

        let reply = serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match reply {
            ControlRequestReply::Logs(data) => data,
            other => bail!("unexpected reply to daemon logs: {other:?}"),
        }
    };

    if let Some(pattern) = grep {
        // Parse and filter historical logs
        let content = String::from_utf8_lossy(&logs);
        for line in content.lines() {
            if let Some(msg) = parse_jsonl_line(line) {
                if matches_grep(&msg, Some(pattern)) {
                    print_log_message(msg, &LogOutputConfig::default());
                }
            } else if line.to_lowercase().contains(&pattern.to_lowercase()) {
                println!("{line}");
            }
        }
    } else {
        std::io::stdout()
            .write_all(&logs)
            .expect("failed to write logs to stdout");
    }

    if !follow {
        return Ok(());
    }
    let log_level = env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .build()
        .filter();

    // subscribe to log messages
    let mut log_session = TcpConnection {
        stream: TcpStream::connect(coordinator_addr)
            .wrap_err("failed to connect to adora coordinator")?,
    };
    log_session
        .send(
            &serde_json::to_vec(&ControlRequest::LogSubscribe {
                dataflow_id: uuid,
                level: log_level,
            })
            .wrap_err("failed to serialize message")?,
        )
        .wrap_err("failed to send log subscribe request to coordinator")?;
    while let Ok(raw) = log_session.receive() {
        let parsed: eyre::Result<LogMessage> =
            serde_json::from_slice(&raw).context("failed to parse log message");
        match parsed {
            Ok(log_message) => {
                if matches_grep(&log_message, grep) {
                    print_log_message(log_message, &LogOutputConfig::default());
                }
            }
            Err(err) => {
                tracing::warn!("failed to parse log message: {err:?}")
            }
        }
    }

    Ok(())
}
