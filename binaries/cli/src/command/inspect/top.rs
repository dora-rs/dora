use std::{
    io,
    time::{Duration, Instant},
};

use clap::Args;
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, NodeInfo},
    id::NodeId,
};
use eyre::{Context, eyre};
use ratatui::{
    Frame, Terminal,
    backend::{Backend, CrosstermBackend},
    layout::{Constraint, Layout},
    style::{Color, Modifier, Style},
    widgets::{Block, Borders, Cell, Row, Table, TableState},
};
use sysinfo::System;
use uuid::Uuid;

use crate::{LOCALHOST, common::connect_to_coordinator};

use super::super::{Executable, default_tracing};

/// Real-time monitor node resource usage (similar to Linux top)
///
/// Note: Process metrics (CPU/Memory) are only available for nodes running
/// on the same machine as the CLI. For distributed dataflows, only node
/// information (name, dataflow, daemon) will be displayed.
#[derive(Debug, Args)]
pub struct Top {
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: std::net::IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
    /// Refresh interval in seconds
    #[clap(long, value_name = "SECONDS", default_value_t = 2)]
    pub refresh_interval: u64,
}

impl Executable for Top {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        // Setup terminal
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;

        // Create app and run it
        let refresh_duration = Duration::from_secs(self.refresh_interval);
        let res = run_app(
            &mut terminal,
            self.coordinator_addr,
            self.coordinator_port,
            refresh_duration,
        );

        // Restore terminal
        disable_raw_mode()?;
        execute!(
            terminal.backend_mut(),
            LeaveAlternateScreen,
            DisableMouseCapture
        )?;
        terminal.show_cursor()?;

        if let Err(err) = res {
            eprintln!("Error: {err:?}");
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SortColumn {
    Node,
    Cpu,
    Memory,
}

struct App {
    node_stats: Vec<NodeStats>,
    table_state: TableState,
    sort_column: SortColumn,
    sort_ascending: bool,
}

#[derive(Debug, Clone)]
struct NodeStats {
    dataflow_id: Uuid,
    dataflow_name: String,
    node_id: NodeId,
    pid: Option<u32>,
    cpu_usage: f32,
    memory_mb: f64,
}

impl App {
    fn new() -> Self {
        let mut table_state = TableState::default();
        table_state.select(Some(0));
        Self {
            node_stats: Vec::new(),
            table_state,
            sort_column: SortColumn::Cpu,
            sort_ascending: false,
        }
    }

    fn next(&mut self) {
        if self.node_stats.is_empty() {
            return;
        }
        let i = match self.table_state.selected() {
            Some(i) => {
                if i >= self.node_stats.len() - 1 {
                    0
                } else {
                    i + 1
                }
            }
            None => 0,
        };
        self.table_state.select(Some(i));
    }

    fn previous(&mut self) {
        if self.node_stats.is_empty() {
            return;
        }
        let i = match self.table_state.selected() {
            Some(i) => {
                if i == 0 {
                    self.node_stats.len() - 1
                } else {
                    i - 1
                }
            }
            None => 0,
        };
        self.table_state.select(Some(i));
    }

    fn toggle_sort(&mut self, column: SortColumn) {
        if self.sort_column == column {
            self.sort_ascending = !self.sort_ascending;
        } else {
            self.sort_column = column;
            self.sort_ascending = false;
        }
        self.sort();
    }

    fn sort(&mut self) {
        match self.sort_column {
            SortColumn::Node => {
                self.node_stats.sort_by(|a, b| {
                    let cmp = a.node_id.as_ref().cmp(b.node_id.as_ref());
                    if self.sort_ascending {
                        cmp
                    } else {
                        cmp.reverse()
                    }
                });
            }
            SortColumn::Cpu => {
                self.node_stats.sort_by(|a, b| {
                    let cmp = a
                        .cpu_usage
                        .partial_cmp(&b.cpu_usage)
                        .unwrap_or(std::cmp::Ordering::Equal);
                    if self.sort_ascending {
                        cmp
                    } else {
                        cmp.reverse()
                    }
                });
            }
            SortColumn::Memory => {
                self.node_stats.sort_by(|a, b| {
                    let cmp = a
                        .memory_mb
                        .partial_cmp(&b.memory_mb)
                        .unwrap_or(std::cmp::Ordering::Equal);
                    if self.sort_ascending {
                        cmp
                    } else {
                        cmp.reverse()
                    }
                });
            }
        }
    }

    fn update_stats(&mut self, node_infos: Vec<NodeInfo>, system: &System) {
        self.node_stats.clear();

        // Build a map of process info for efficient lookup
        // Note: This heuristic matching only works for local nodes where the
        // executable path or name might contain the node ID. It will not work
        // for distributed nodes or nodes with different executable names.
        let mut process_map: std::collections::HashMap<String, (u32, f32, f64)> =
            std::collections::HashMap::new();

        for (process_pid, process) in system.processes() {
            // Build searchable string from process name (cmd() requires admin on Windows)
            let name = process.name().to_string_lossy().to_string();

            // Store process metrics
            let pid = process_pid.as_u32();
            let cpu = process.cpu_usage();
            let mem = process.memory() as f64 / 1024.0 / 1024.0;

            process_map.insert(name, (pid, cpu, mem));
        }

        // Match nodes to processes
        for node_info in node_infos {
            let node_id_str = node_info.node_id.as_ref();
            let mut pid = None;
            let mut cpu_usage = 0.0;
            let mut memory_mb = 0.0;

            // Try to find a matching process
            // This is a best-effort heuristic that may not work for all cases
            for (proc_name, &(proc_pid, proc_cpu, proc_mem)) in &process_map {
                if proc_name.contains(node_id_str) {
                    pid = Some(proc_pid);
                    cpu_usage = proc_cpu;
                    memory_mb = proc_mem;
                    break;
                }
            }

            self.node_stats.push(NodeStats {
                dataflow_id: node_info.dataflow_id,
                dataflow_name: node_info
                    .dataflow_name
                    .unwrap_or_else(|| "<unnamed>".to_string()),
                node_id: node_info.node_id,
                pid,
                cpu_usage,
                memory_mb,
            });
        }

        self.sort();
    }
}

fn run_app<B: Backend>(
    terminal: &mut Terminal<B>,
    coordinator_addr: std::net::IpAddr,
    coordinator_port: u16,
    refresh_duration: Duration,
) -> eyre::Result<()> {
    let mut app = App::new();
    let mut last_update = Instant::now();
    let mut last_node_query = Instant::now();
    let mut system = System::new_all();
    let mut node_infos: Vec<NodeInfo> = Vec::new();

    // Reuse coordinator connection
    let mut session = connect_to_coordinator((coordinator_addr, coordinator_port).into())
        .wrap_err("Failed to connect to coordinator")?;

    // Query node info once initially
    let request = ControlRequest::GetNodeInfo;
    let reply_raw = session
        .request(&serde_json::to_vec(&request).unwrap())
        .wrap_err("failed to send initial request to coordinator")?;

    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;

    match reply {
        ControlRequestReply::NodeInfoList(infos) => {
            node_infos = infos;
        }
        ControlRequestReply::Error(err) => {
            return Err(eyre!("coordinator error: {err}"));
        }
        _ => {
            return Err(eyre!("unexpected reply from coordinator"));
        }
    }

    loop {
        terminal.draw(|f| ui(f, &mut app, refresh_duration))?;

        let timeout = refresh_duration
            .checked_sub(last_update.elapsed())
            .unwrap_or(Duration::from_millis(100));

        if event::poll(timeout)? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Esc => {
                            return Ok(());
                        }
                        KeyCode::Down | KeyCode::Char('j') => {
                            app.next();
                        }
                        KeyCode::Up | KeyCode::Char('k') => {
                            app.previous();
                        }
                        KeyCode::Char('n') => {
                            app.toggle_sort(SortColumn::Node);
                        }
                        KeyCode::Char('c') => {
                            app.toggle_sort(SortColumn::Cpu);
                        }
                        KeyCode::Char('m') => {
                            app.toggle_sort(SortColumn::Memory);
                        }
                        KeyCode::Char('r') => {
                            // Force refresh node list
                            last_node_query = Instant::now()
                                .checked_sub(Duration::from_secs(60))
                                .unwrap_or(Instant::now());
                        }
                        _ => {}
                    }
                }
            }
        }

        // Update data if refresh interval has passed
        if last_update.elapsed() >= refresh_duration {
            // Refresh system info (process metrics)
            system.refresh_all();

            // Re-query node info periodically (every 30 seconds) or on demand
            // Node list changes less frequently than metrics
            if last_node_query.elapsed() >= Duration::from_secs(30) {
                let request = ControlRequest::GetNodeInfo;
                let reply_raw = session
                    .request(&serde_json::to_vec(&request).unwrap())
                    .wrap_err("failed to send request to coordinator")?;

                let reply: ControlRequestReply =
                    serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;

                match reply {
                    ControlRequestReply::NodeInfoList(infos) => {
                        node_infos = infos;
                    }
                    ControlRequestReply::Error(err) => {
                        return Err(eyre!("coordinator error: {err}"));
                    }
                    _ => {
                        return Err(eyre!("unexpected reply from coordinator"));
                    }
                }

                last_node_query = Instant::now();
            }

            // Update stats with current node info and fresh system metrics
            app.update_stats(node_infos.clone(), &system);
            last_update = Instant::now();
        }
    }
}

fn ui(f: &mut Frame, app: &mut App, refresh_duration: Duration) {
    let chunks = Layout::default()
        .constraints([Constraint::Min(0)])
        .split(f.area());

    let sort_indicator = |col: SortColumn| {
        if app.sort_column == col {
            if app.sort_ascending { " ▲" } else { " ▼" }
        } else {
            ""
        }
    };

    let header_strings = vec![
        format!("NODE{}", sort_indicator(SortColumn::Node)),
        "DATAFLOW".to_string(),
        "PID".to_string(),
        format!("CPU%{}", sort_indicator(SortColumn::Cpu)),
        format!("MEMORY (MB){}", sort_indicator(SortColumn::Memory)),
    ];

    let header_cells = header_strings.iter().map(|h| {
        Cell::from(h.as_str()).style(
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        )
    });

    let header = Row::new(header_cells).height(1).bottom_margin(1);

    let rows = app.node_stats.iter().map(|stats| {
        let cells = vec![
            Cell::from(stats.node_id.as_ref()),
            Cell::from(stats.dataflow_name.as_str()),
            Cell::from(
                stats
                    .pid
                    .map(|p| p.to_string())
                    .unwrap_or_else(|| "N/A".to_string()),
            ),
            Cell::from(format!("{:.1}%", stats.cpu_usage)),
            Cell::from(format!("{:.1}", stats.memory_mb)),
        ];
        Row::new(cells).height(1)
    });

    let widths = [
        Constraint::Percentage(25),
        Constraint::Percentage(25),
        Constraint::Percentage(10),
        Constraint::Percentage(15),
        Constraint::Percentage(25),
    ];

    let title = format!(
        " Dora Inspect Top - Refreshing every {}s (q: quit, n/c/m: sort, r: refresh nodes) ",
        refresh_duration.as_secs()
    );

    let table = Table::new(rows, widths)
        .header(header)
        .block(Block::default().borders(Borders::ALL).title(title))
        .row_highlight_style(
            Style::default()
                .bg(Color::DarkGray)
                .add_modifier(Modifier::BOLD),
        )
        .highlight_symbol(">> ");

    f.render_stateful_widget(table, chunks[0], &mut app.table_state);
}
