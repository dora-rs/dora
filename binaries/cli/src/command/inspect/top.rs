use std::{
    io,
    time::{Duration, Instant},
};

use clap::Args;
use crossterm::{
    event::{
        self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEvent, KeyEventKind,
    },
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{coordinator_to_cli::NodeInfo, id::NodeId, tarpc};
use eyre::Context;

use crate::common::{connect_to_coordinator_rpc, rpc};
use ratatui::{
    Frame, Terminal,
    backend::{Backend, CrosstermBackend},
    layout::{Constraint, Layout},
    style::{Color, Modifier, Style},
    widgets::{Block, Borders, Cell, Row, Table, TableState},
};
use uuid::Uuid;

use super::super::{Executable, default_tracing};

/// Real-time monitor node resource usage (similar to Linux top)
///
/// Metrics are collected by daemons and reported to the coordinator,
/// so this works for distributed dataflows across multiple machines.
///
/// Note:
/// - Values are averaged over the last refresh period
/// - CPU percentage is of a single core (values can add to more than 100% if multiple cores are used)
/// - Nodes can run on different machines with potentially different CPUs, so percentages are not comparable across machines
#[derive(Debug, Args)]
pub struct Top {
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP")]
    pub coordinator_addr: Option<std::net::IpAddr>,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT")]
    pub coordinator_port: Option<u16>,
    /// Refresh interval in seconds
    #[clap(long, value_name = "SECONDS", default_value_t = 2)]
    pub refresh_interval: u64,
}

impl Executable for Top {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        use crate::common::resolve_coordinator_addr;
        let (addr, port) = resolve_coordinator_addr(
            self.coordinator_addr,
            self.coordinator_port,
            DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
        );

        // Setup terminal
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;

        // Create app and run it
        let refresh_duration = Duration::from_secs(self.refresh_interval);
        let res = run_app(&mut terminal, addr, port, refresh_duration).await;

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
    #[allow(dead_code)]
    dataflow_id: Uuid,
    dataflow_name: String,
    node_id: NodeId,
    pid: Option<u32>,
    cpu_usage: f32,
    memory_mb: f64,
    disk_read_mb_s: Option<f64>,
    disk_write_mb_s: Option<f64>,
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

    fn update_stats(&mut self, node_infos: Vec<NodeInfo>) {
        self.node_stats.clear();

        // Use daemon-provided metrics (works for distributed nodes!)
        for node_info in node_infos {
            let (pid, cpu_usage, memory_mb, disk_read_mb_s, disk_write_mb_s) =
                if let Some(metrics) = &node_info.metrics {
                    (
                        Some(metrics.pid),
                        metrics.cpu_usage,
                        metrics.memory_mb,
                        metrics.disk_read_mb_s,
                        metrics.disk_write_mb_s,
                    )
                } else {
                    (None, 0.0, 0.0, None, None)
                };

            self.node_stats.push(NodeStats {
                dataflow_id: node_info.dataflow_id,
                dataflow_name: node_info
                    .dataflow_name
                    .unwrap_or_else(|| "<unnamed>".to_string()),
                node_id: node_info.node_id,
                pid,
                cpu_usage,
                memory_mb,
                disk_read_mb_s,
                disk_write_mb_s,
            });
        }

        self.sort();
    }
}

async fn run_app<B: Backend>(
    terminal: &mut Terminal<B>,
    coordinator_addr: std::net::IpAddr,
    coordinator_port: u16,
    refresh_duration: Duration,
) -> eyre::Result<()> {
    let mut app = App::new();
    let mut last_update = Instant::now();
    let mut node_infos: Vec<NodeInfo> = Vec::new();

    // Reuse coordinator connection
    let client = connect_to_coordinator_rpc(coordinator_addr, coordinator_port)
        .await
        .wrap_err("Failed to connect to coordinator")?;

    // Query node info once initially
    node_infos = rpc(
        "get node info",
        client.get_node_info(tarpc::context::current()),
    )
    .await?;

    loop {
        terminal.draw(|f| ui(f, &mut app, refresh_duration))?;

        let timeout = refresh_duration
            .checked_sub(last_update.elapsed())
            .unwrap_or(Duration::from_millis(100));

        let key_event: Option<KeyEvent> = tokio::task::spawn_blocking(move || {
            if event::poll(timeout)? {
                if let Event::Key(key) = event::read()? {
                    return Ok(Some(key));
                }
            }
            Ok::<_, std::io::Error>(None)
        })
        .await??;

        if let Some(key) = key_event {
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
                        // Force refresh by resetting last_update
                        last_update = Instant::now()
                            .checked_sub(refresh_duration)
                            .unwrap_or(Instant::now());
                    }
                    _ => {}
                }
            }
        }

        // Update data if refresh interval has passed
        if last_update.elapsed() >= refresh_duration {
            // Query node info every refresh interval to get updated metrics
            node_infos = rpc(
                "refresh node info",
                client.get_node_info(tarpc::context::current()),
            )
            .await?;

            // Update stats with current node info
            app.update_stats(node_infos.clone());
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
        "I/O READ (MB/s)".to_string(),
        "I/O WRITE (MB/s)".to_string(),
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
            Cell::from(
                stats
                    .disk_read_mb_s
                    .map(|v| format!("{:.1}", v))
                    .unwrap_or_else(|| "N/A".to_string()),
            ),
            Cell::from(
                stats
                    .disk_write_mb_s
                    .map(|v| format!("{:.1}", v))
                    .unwrap_or_else(|| "N/A".to_string()),
            ),
        ];
        Row::new(cells).height(1)
    });

    let widths = [
        Constraint::Percentage(20),
        Constraint::Percentage(20),
        Constraint::Percentage(8),
        Constraint::Percentage(12),
        Constraint::Percentage(12),
        Constraint::Percentage(14),
        Constraint::Percentage(14),
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
