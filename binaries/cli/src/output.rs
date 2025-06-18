use colored::Colorize;
use dora_core::build::LogLevelOrStdout;
use dora_message::common::LogMessage;

pub fn print_log_message(
    log_message: LogMessage,
    print_dataflow_id: bool,
    print_daemon_name: bool,
) {
    let LogMessage {
        build_id: _,
        dataflow_id,
        node_id,
        daemon_id,
        level,
        target,
        module_path: _,
        file: _,
        line: _,
        message,
    } = log_message;
    let level = match level {
        LogLevelOrStdout::LogLevel(level) => match level {
            log::Level::Error => "ERROR ".red(),
            log::Level::Warn => "WARN  ".yellow(),
            log::Level::Info => "INFO  ".green(),
            log::Level::Debug => "DEBUG ".bright_blue(),
            log::Level::Trace => "TRACE ".dimmed(),
        },
        LogLevelOrStdout::Stdout => "stdout".bright_blue().italic().dimmed(),
    };

    let dataflow = match dataflow_id {
        Some(dataflow_id) if print_dataflow_id => format!("dataflow `{dataflow_id}` ").cyan(),
        _ => String::new().cyan(),
    };
    let daemon = match daemon_id {
        Some(id) if print_daemon_name => match id.machine_id() {
            Some(machine_id) => format!("on daemon `{machine_id}`"),
            None => "on default daemon ".to_string(),
        },
        None if print_daemon_name => "on default daemon".to_string(),
        _ => String::new(),
    }
    .bright_black();
    let colon = ":".bright_black().bold();
    let node = match node_id {
        Some(node_id) => {
            let node_id = node_id.to_string().dimmed().bold();
            let padding = if daemon.is_empty() { "" } else { " " };
            format!("{node_id}{padding}{daemon}{colon} ")
        }
        None if daemon.is_empty() => "".into(),
        None => format!("{daemon}{colon} "),
    };
    let target = match target {
        Some(target) => format!("{target} ").dimmed(),
        None => "".normal(),
    };

    println!("{node}{level} {target}{dataflow}   {message}");
}
