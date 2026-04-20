mod build;
mod cluster;
mod completion;
mod coordinator;
mod daemon;
mod doctor;
mod down;
mod expand;
mod graph;
pub mod inspect;
mod list;
mod logs;
mod new;
mod node;
mod node_binary;
mod param;
mod record;
mod replay;
mod restart;
mod run;
mod runtime;
mod self_;
mod start;
mod stop;
mod system;
mod topic;
mod trace;
mod up;
mod validate;

pub use build::{BuildConfig, build};
pub use run::{Run, run};

use build::Build;
use cluster::Cluster;
use completion::Completion;
use coordinator::Coordinator;
use daemon::Daemon;
use doctor::Doctor;
use down::Down;
use expand::Expand;
use eyre::Context;
use graph::Graph;
use inspect::Inspect;
use list::ListArgs;
use logs::LogsArgs;
use new::NewArgs;
use node::Node;
use param::Param;
use record::Record;
use replay::Replay;
use restart::Restart;
use runtime::Runtime;
use self_::SelfSubCommand;
use start::Start;
use stop::Stop;
use system::System;
use topic::Topic;
use trace::Trace;
use up::Up;
use validate::Validate;

/// dora-rs cli client
#[derive(Debug, clap::Subcommand)]
pub enum Command {
    // -- Lifecycle --
    /// Run a dataflow locally with embedded coordinator and daemon
    #[clap(display_order = 1)]
    Run(Run),
    /// Start coordinator and daemon in local mode
    #[clap(display_order = 2)]
    Up(Up),
    /// Tear down coordinator and daemon. Stops any running dataflows first.
    #[clap(name = "down", alias = "destroy", display_order = 3)]
    Down(Down),
    /// Manage a multi-machine cluster (up, status, down)
    #[clap(subcommand, display_order = 4)]
    Cluster(Cluster),
    /// Run build commands provided in the given dataflow
    #[clap(display_order = 5)]
    Build(Build),
    /// Start a dataflow on a running coordinator
    #[clap(display_order = 6)]
    Start(Start),
    /// Stop a running dataflow
    #[clap(display_order = 7)]
    Stop(Stop),
    /// Restart a running dataflow (stop + re-start with stored descriptor)
    #[clap(display_order = 8)]
    Restart(Restart),

    // -- Monitoring --
    /// List running dataflows
    #[clap(alias = "ps", display_order = 10)]
    List(ListArgs),
    /// Show logs of a given dataflow and node
    #[command(allow_missing_positional = true)]
    #[clap(display_order = 11)]
    Logs(LogsArgs),
    /// Inspect running dataflows in real-time
    #[clap(subcommand, display_order = 12)]
    Inspect(Inspect),
    /// Manage and inspect dataflow topics
    #[clap(subcommand, display_order = 13)]
    Topic(Topic),
    /// Manage and inspect dataflow nodes
    #[clap(subcommand, display_order = 14)]
    Node(Node),
    /// Manage runtime parameters on running nodes
    #[clap(subcommand, display_order = 15)]
    Param(Param),
    /// Record dataflow messages to a file for offline replay
    #[clap(display_order = 16)]
    Record(Record),
    /// Replay a recorded dataflow from a `.drec` file
    #[clap(display_order = 17)]
    Replay(Replay),
    /// View coordinator tracing spans
    #[clap(subcommand, display_order = 18)]
    Trace(Trace),

    // -- Setup --
    /// Check system health
    #[clap(alias = "check", display_order = 20)]
    Status(system::status::Status),
    /// Run comprehensive system diagnostics
    #[clap(display_order = 19)]
    Doctor(Doctor),
    /// Generate a new project or node
    #[clap(display_order = 21)]
    New(NewArgs),
    /// Visualize a dataflow as a graph
    #[clap(display_order = 22)]
    Graph(Graph),
    /// Expand module references and print the flat dataflow YAML
    #[clap(display_order = 23)]
    Expand(Expand),
    /// Validate a dataflow YAML file and check type annotations
    #[clap(display_order = 24)]
    Validate(Validate),
    /// System management commands
    #[clap(subcommand, display_order = 25)]
    System(System),

    // -- Utility --
    /// Generate shell completions
    #[clap(display_order = 30)]
    Completion(Completion),
    /// CLI self-management (update, uninstall)
    #[clap(display_order = 31)]
    Self_ {
        #[clap(subcommand)]
        command: SelfSubCommand,
    },

    // -- Hidden: internal / aliases --
    #[clap(hide = true)]
    Daemon(Daemon),
    #[clap(hide = true)]
    Runtime(Runtime),
    #[clap(hide = true)]
    Coordinator(Coordinator),
    /// Real-time resource monitor (shortcut for `inspect top`)
    #[clap(display_order = 12)]
    Top(inspect::top::Top),
}

fn default_tracing() -> eyre::Result<()> {
    #[cfg(feature = "tracing")]
    {
        use dora_tracing::TracingBuilder;

        TracingBuilder::new("dora-cli")
            .with_stdout("warn", false)
            .build()
            .wrap_err("failed to set up tracing subscriber")?;
    }
    Ok(())
}

pub trait Executable {
    fn execute(self) -> eyre::Result<()>;
}

impl Executable for Command {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Command::Run(args) => args.execute(),
            Command::Up(args) => args.execute(),
            Command::Down(args) => args.execute(),
            Command::Cluster(args) => args.execute(),
            Command::Build(args) => args.execute(),
            Command::Start(args) => args.execute(),
            Command::Stop(args) => args.execute(),
            Command::Restart(args) => args.execute(),
            Command::List(args) => args.execute(),
            Command::Logs(args) => args.execute(),
            Command::Inspect(args) => args.execute(),
            Command::Topic(args) => args.execute(),
            Command::Node(args) => args.execute(),
            Command::Param(args) => args.execute(),
            Command::Record(args) => args.execute(),
            Command::Replay(args) => args.execute(),
            Command::Trace(args) => args.execute(),
            Command::Status(args) => args.execute(),
            Command::Doctor(args) => args.execute(),
            Command::New(args) => args.execute(),
            Command::Graph(args) => args.execute(),
            Command::Expand(args) => args.execute(),
            Command::Validate(args) => args.execute(),
            Command::System(args) => args.execute(),
            Command::Completion(args) => args.execute(),
            Command::Self_ { command } => command.execute(),
            Command::Daemon(args) => args.execute(),
            Command::Runtime(args) => args.execute(),
            Command::Coordinator(args) => args.execute(),
            Command::Top(args) => args.execute(),
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::Args;
    use clap::{CommandFactory, Parser};

    #[test]
    fn verify_cli() {
        Args::command().debug_assert();
    }

    fn parse_ok(args: &[&str]) {
        Args::try_parse_from(args).unwrap_or_else(|e| panic!("failed to parse {args:?}: {e}"));
    }

    fn parse_err(args: &[&str]) {
        assert!(
            Args::try_parse_from(args).is_err(),
            "expected parse error for {args:?}"
        );
    }

    #[test]
    fn parse_run() {
        parse_ok(&["dora", "run", "foo.yml"]);
    }

    #[test]
    fn parse_run_locked() {
        parse_ok(&["dora", "run", "foo.yml", "--locked"]);
    }

    #[test]
    fn reject_run_locked_and_write_lockfile() {
        parse_err(&["dora", "run", "foo.yml", "--locked", "--write-lockfile"]);
    }

    #[test]
    fn parse_up() {
        parse_ok(&["dora", "up"]);
    }

    #[test]
    fn parse_down() {
        parse_ok(&["dora", "down"]);
    }

    #[test]
    fn parse_start() {
        parse_ok(&["dora", "start", "foo.yml"]);
    }

    #[test]
    fn parse_stop_uuid() {
        parse_ok(&["dora", "stop", "a1a2a3a4-b1b2-c1c2-d1d2-e1e2e3e4e5e6"]);
    }

    #[test]
    fn parse_list() {
        parse_ok(&["dora", "list"]);
    }

    #[test]
    fn parse_logs() {
        parse_ok(&["dora", "logs"]);
    }

    #[test]
    fn parse_build() {
        parse_ok(&["dora", "build", "foo.yml"]);
    }

    #[test]
    fn parse_build_locked() {
        parse_ok(&["dora", "build", "foo.yml", "--locked"]);
    }

    #[test]
    fn reject_build_locked_and_write_lockfile() {
        parse_err(&["dora", "build", "foo.yml", "--locked", "--write-lockfile"]);
    }

    #[test]
    fn parse_graph() {
        parse_ok(&["dora", "graph", "foo.yml"]);
    }

    #[test]
    fn parse_expand() {
        parse_ok(&["dora", "expand", "foo.yml"]);
    }

    #[test]
    fn parse_expand_module() {
        parse_ok(&["dora", "expand", "--module", "module.yml"]);
    }

    #[test]
    fn parse_validate() {
        parse_ok(&["dora", "validate", "dataflow.yml"]);
    }

    #[test]
    fn parse_validate_strict() {
        parse_ok(&["dora", "validate", "--strict-types", "dataflow.yml"]);
    }

    #[test]
    fn parse_new() {
        parse_ok(&["dora", "new", "test"]);
    }

    #[test]
    fn parse_status() {
        parse_ok(&["dora", "status"]);
    }

    #[test]
    fn parse_status_json() {
        parse_ok(&["dora", "status", "--format", "json"]);
    }

    #[test]
    fn parse_status_json_short() {
        parse_ok(&["dora", "status", "-f", "json"]);
    }

    #[test]
    fn parse_inspect_top() {
        parse_ok(&["dora", "inspect", "top"]);
    }

    #[test]
    fn parse_topic_list() {
        parse_ok(&["dora", "topic", "list"]);
    }

    #[test]
    fn parse_topic_hz() {
        parse_ok(&["dora", "topic", "hz"]);
    }

    #[test]
    fn parse_topic_echo() {
        parse_ok(&["dora", "topic", "echo"]);
    }

    #[test]
    fn parse_node_list() {
        parse_ok(&["dora", "node", "list"]);
    }

    #[test]
    fn parse_node_info() {
        parse_ok(&["dora", "node", "info", "camera_node"]);
    }

    #[test]
    fn parse_node_info_with_dataflow() {
        parse_ok(&["dora", "node", "info", "sensor", "-d", "my-dataflow"]);
    }

    #[test]
    fn reject_node_info_no_node() {
        parse_err(&["dora", "node", "info"]);
    }

    #[test]
    fn parse_topic_pub() {
        parse_ok(&[
            "dora",
            "topic",
            "pub",
            "-d",
            "my-dataflow",
            "sensor/reading",
            r#"{"value": 42}"#,
        ]);
    }

    #[test]
    fn parse_topic_pub_with_file() {
        parse_ok(&[
            "dora",
            "topic",
            "pub",
            "-d",
            "my-dataflow",
            "sensor/reading",
            "--file",
            "data.json",
        ]);
    }

    #[test]
    fn reject_topic_pub_no_data_or_file() {
        parse_err(&[
            "dora",
            "topic",
            "pub",
            "-d",
            "my-dataflow",
            "sensor/reading",
        ]);
    }

    #[test]
    fn parse_node_restart() {
        parse_ok(&["dora", "node", "restart", "camera_node"]);
    }

    #[test]
    fn parse_node_restart_with_grace() {
        parse_ok(&["dora", "node", "restart", "sensor", "--grace", "30s"]);
    }

    #[test]
    fn reject_node_restart_no_node() {
        parse_err(&["dora", "node", "restart"]);
    }

    #[test]
    fn parse_node_stop() {
        parse_ok(&["dora", "node", "stop", "camera_node"]);
    }

    #[test]
    fn parse_node_stop_with_grace() {
        parse_ok(&[
            "dora", "node", "stop", "sensor", "--grace", "10s", "-d", "my-flow",
        ]);
    }

    #[test]
    fn reject_node_stop_no_node() {
        parse_err(&["dora", "node", "stop"]);
    }

    #[test]
    fn parse_param_list() {
        parse_ok(&["dora", "param", "list", "camera_node"]);
    }

    #[test]
    fn parse_param_list_with_dataflow() {
        parse_ok(&["dora", "param", "list", "sensor", "-d", "my-dataflow"]);
    }

    #[test]
    fn parse_param_get() {
        parse_ok(&["dora", "param", "get", "camera_node", "fps"]);
    }

    #[test]
    fn parse_param_set() {
        parse_ok(&["dora", "param", "set", "camera_node", "fps", "60"]);
    }

    #[test]
    fn parse_param_delete() {
        parse_ok(&["dora", "param", "delete", "camera_node", "fps"]);
    }

    #[test]
    fn reject_param_set_no_value() {
        parse_err(&["dora", "param", "set", "camera_node", "fps"]);
    }

    #[test]
    fn reject_param_get_no_key() {
        parse_err(&["dora", "param", "get", "camera_node"]);
    }

    #[test]
    fn parse_doctor() {
        parse_ok(&["dora", "doctor"]);
    }

    #[test]
    fn parse_doctor_with_dataflow() {
        parse_ok(&["dora", "doctor", "--dataflow", "dataflow.yml"]);
    }

    #[test]
    fn parse_record() {
        parse_ok(&["dora", "record", "dataflow.yml"]);
    }

    #[test]
    fn parse_record_with_output() {
        parse_ok(&["dora", "record", "dataflow.yml", "-o", "capture.drec"]);
    }

    #[test]
    fn parse_record_with_topics() {
        parse_ok(&[
            "dora",
            "record",
            "dataflow.yml",
            "--topics",
            "sensor/image,lidar/points",
        ]);
    }

    #[test]
    fn parse_record_output_yaml() {
        parse_ok(&[
            "dora",
            "record",
            "dataflow.yml",
            "--output-yaml",
            "modified.yml",
        ]);
    }

    #[test]
    fn reject_record_no_file() {
        parse_err(&["dora", "record"]);
    }

    #[test]
    fn parse_replay() {
        parse_ok(&["dora", "replay", "recording.drec"]);
    }

    #[test]
    fn parse_replay_with_options() {
        parse_ok(&[
            "dora",
            "replay",
            "recording.drec",
            "--speed",
            "2.0",
            "--loop",
            "--replace",
            "sensor,camera",
        ]);
    }

    #[test]
    fn parse_replay_output_yaml() {
        parse_ok(&[
            "dora",
            "replay",
            "recording.drec",
            "--output-yaml",
            "modified.yml",
        ]);
    }

    #[test]
    fn reject_replay_no_file() {
        parse_err(&["dora", "replay"]);
    }

    #[test]
    fn parse_trace_list() {
        parse_ok(&["dora", "trace", "list"]);
    }

    #[test]
    fn parse_trace_view() {
        parse_ok(&["dora", "trace", "view", "abc123"]);
    }

    #[test]
    fn reject_trace_view_no_id() {
        parse_err(&["dora", "trace", "view"]);
    }

    #[test]
    fn parse_cluster_up() {
        parse_ok(&["dora", "cluster", "up", "cluster.yml"]);
    }

    #[test]
    fn parse_cluster_status() {
        parse_ok(&["dora", "cluster", "status"]);
    }

    #[test]
    fn parse_cluster_down() {
        parse_ok(&["dora", "cluster", "down"]);
    }

    #[test]
    fn reject_cluster_up_no_file() {
        parse_err(&["dora", "cluster", "up"]);
    }

    #[test]
    fn reject_unknown_subcommand() {
        parse_err(&["dora", "foo"]);
    }

    #[test]
    fn help_exits_cleanly() {
        let err = Args::try_parse_from(["dora", "--help"]).unwrap_err();
        assert_eq!(err.kind(), clap::error::ErrorKind::DisplayHelp);
    }

    #[test]
    fn version_exits_cleanly() {
        let err = Args::try_parse_from(["dora", "--version"]).unwrap_err();
        assert_eq!(err.kind(), clap::error::ErrorKind::DisplayVersion);
    }
}
