use std::net::{IpAddr, Ipv4Addr};

pub const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const DORA_COORDINATOR_PORT_DEFAULT: u16 = 53290;
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT: u16 = 53291;
pub const DORA_COORDINATOR_PORT_CONTROL_DEFAULT: u16 = 6012;

/// Calculate the RPC port from the control port.
/// The RPC port is always control_port + 1.
pub const fn dora_coordinator_port_rpc(control_port: u16) -> u16 {
    control_port + 1
}

pub const MANUAL_STOP: &str = "dora/stop";

/// Default port for the zenoh peer endpoint used by daemons.
pub const DORA_ZENOH_PEER_PORT_DEFAULT: u16 = 5456;

/// Build a zenoh config with common settings.
///
/// When `listen_port` is `Some`, the session will bind on that port so that
/// other zenoh peers (e.g. CLI) can connect to it deterministically.
#[cfg(feature = "zenoh")]
fn build_zenoh_config(coordinator_addr: Option<IpAddr>, listen_port: Option<u16>) -> zenoh::Config {
    let mut zenoh_config = zenoh::Config::default();
    // Linkstate make it possible to connect two daemons on different network through a public daemon
    // TODO: There is currently a CI/CD Error in windows linkstate.
    if cfg!(not(target_os = "windows")) {
        if let Err(err) = zenoh_config
            .insert_json5("routing/peer", r#"{ mode: "linkstate" }"#)
        {
            tracing::warn!("failed to set zenoh routing peer mode to linkstate: {err}");
        }
    }

    if let Some(port) = listen_port {
        if let Err(err) = zenoh_config
            .insert_json5("listen/endpoints", &format!(r#"["tcp/0.0.0.0:{port}"]"#))
        {
            tracing::warn!("failed to set zenoh listen endpoint: {err}");
        }
    }

    if let Some(addr) = coordinator_addr {
        // Skip connect endpoint when the daemon is already listening on the
        // same port on this machine — connecting to ourselves causes
        // CONNECTION_TO_SELF errors from zenoh.
        let would_connect_to_self =
            listen_port == Some(DORA_ZENOH_PEER_PORT_DEFAULT) && addr.is_loopback();
        if !would_connect_to_self {
            if let Err(err) = zenoh_config
                .insert_json5(
                    "connect/endpoints",
                    &format!(r#"["tcp/{}:{}"]"#, addr, DORA_ZENOH_PEER_PORT_DEFAULT),
                )
            {
                tracing::warn!("failed to set zenoh connect endpoint: {err}");
            }
        }
    }
    zenoh_config
}

/// Open a zenoh session for a **client** (CLI or similar).
///
/// The session connects to the daemon's known peer port but does not
/// listen on a fixed port itself.
#[cfg(feature = "zenoh")]
pub async fn open_zenoh_session(coordinator_addr: Option<IpAddr>) -> eyre::Result<zenoh::Session> {
    use eyre::{Context, eyre};
    use tracing::warn;

    let zenoh_session = match std::env::var(zenoh::Config::DEFAULT_CONFIG_PATH_ENV) {
        Ok(path) => {
            let zenoh_config = zenoh::Config::from_file(&path)
                .map_err(|e| eyre!(e))
                .wrap_err_with(|| format!("failed to read zenoh config from {path}"))?;
            zenoh::open(zenoh_config)
                .await
                .map_err(|e| eyre!(e))
                .context("failed to open zenoh session")?
        }
        Err(std::env::VarError::NotPresent) => {
            let zenoh_config = build_zenoh_config(coordinator_addr, None);
            if let Ok(zenoh_session) = zenoh::open(zenoh_config).await {
                zenoh_session
            } else {
                warn!("failed to open zenoh session, retrying with default config");
                let zenoh_config = zenoh::Config::default();
                zenoh::open(zenoh_config)
                    .await
                    .map_err(|e| eyre!(e))
                    .context("failed to open zenoh session")?
            }
        }
        Err(std::env::VarError::NotUnicode(_)) => eyre::bail!(
            "{} env variable is not valid unicode",
            zenoh::Config::DEFAULT_CONFIG_PATH_ENV
        ),
    };
    Ok(zenoh_session)
}

/// Open a zenoh session for a **daemon**.
///
/// The daemon listens on [`DORA_ZENOH_PEER_PORT_DEFAULT`] so that CLI
/// and other peers can connect deterministically without relying on
/// multicast scouting.
#[cfg(feature = "zenoh")]
pub async fn open_zenoh_session_as_daemon(
    coordinator_addr: Option<IpAddr>,
) -> eyre::Result<zenoh::Session> {
    use eyre::{Context, eyre};
    use tracing::warn;

    let zenoh_session = match std::env::var(zenoh::Config::DEFAULT_CONFIG_PATH_ENV) {
        Ok(path) => {
            let zenoh_config = zenoh::Config::from_file(&path)
                .map_err(|e| eyre!(e))
                .wrap_err_with(|| format!("failed to read zenoh config from {path}"))?;
            zenoh::open(zenoh_config)
                .await
                .map_err(|e| eyre!(e))
                .context("failed to open zenoh session")?
        }
        Err(std::env::VarError::NotPresent) => {
            let zenoh_config =
                build_zenoh_config(coordinator_addr, Some(DORA_ZENOH_PEER_PORT_DEFAULT));
            if let Ok(zenoh_session) = zenoh::open(zenoh_config).await {
                zenoh_session
            } else {
                warn!(
                    "failed to open zenoh session with listen port {}, retrying without",
                    DORA_ZENOH_PEER_PORT_DEFAULT
                );
                // Fall back to a session without a fixed listen port (e.g. if
                // the port is already in use by another daemon on the same host).
                let zenoh_config = build_zenoh_config(coordinator_addr, None);
                if let Ok(zenoh_session) = zenoh::open(zenoh_config).await {
                    zenoh_session
                } else {
                    warn!("failed to open zenoh session, retrying with default config");
                    let zenoh_config = zenoh::Config::default();
                    zenoh::open(zenoh_config)
                        .await
                        .map_err(|e| eyre!(e))
                        .context("failed to open zenoh session")?
                }
            }
        }
        Err(std::env::VarError::NotUnicode(_)) => eyre::bail!(
            "{} env variable is not valid unicode",
            zenoh::Config::DEFAULT_CONFIG_PATH_ENV
        ),
    };
    Ok(zenoh_session)
}

#[cfg(feature = "zenoh")]
pub fn zenoh_output_publish_topic(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
    output_id: &dora_message::id::DataId,
) -> String {
    let network_id = "default";
    format!("dora/{network_id}/{dataflow_id}/output/{node_id}/{output_id}")
}

/// Return the zenoh topic suffix for a given log level.
///
/// The suffix is appended to the base topic so that subscribers can
/// match only the levels they care about.
#[cfg(feature = "zenoh")]
pub fn log_level_topic_suffix(level: &dora_message::common::LogLevelOrStdout) -> &'static str {
    use dora_message::common::LogLevelOrStdout;
    match level {
        LogLevelOrStdout::Stdout => "stdout",
        LogLevelOrStdout::LogLevel(l) => match l {
            log::Level::Error => "error",
            log::Level::Warn => "warn",
            log::Level::Info => "info",
            log::Level::Debug => "debug",
            log::Level::Trace => "trace",
        },
    }
}

/// All topic suffixes that should be subscribed to for a given
/// [`log::LevelFilter`].
///
/// Stdout is always included because it should always be displayed.
#[cfg(feature = "zenoh")]
pub fn log_level_suffixes_for_filter(filter: log::LevelFilter) -> Vec<&'static str> {
    let mut suffixes = vec!["stdout"];
    // log levels: Error(1) < Warn(2) < Info(3) < Debug(4) < Trace(5)
    // LevelFilter uses the same ordering; a filter of Info means
    // Error, Warn, and Info should pass.
    if filter >= log::LevelFilter::Error {
        suffixes.push("error");
    }
    if filter >= log::LevelFilter::Warn {
        suffixes.push("warn");
    }
    if filter >= log::LevelFilter::Info {
        suffixes.push("info");
    }
    if filter >= log::LevelFilter::Debug {
        suffixes.push("debug");
    }
    if filter >= log::LevelFilter::Trace {
        suffixes.push("trace");
    }
    suffixes
}

/// Zenoh base key expression for subscribing to log messages from **all**
/// sources (nodes and daemons) of a dataflow.
///
/// Format: `dora/log/dataflow/{dataflow_id}/*/*`
///
/// Append a level suffix (e.g. `/info`) to form a complete subscribe topic.
#[cfg(feature = "zenoh")]
pub fn zenoh_log_base_topic_for_dataflow(dataflow_id: uuid::Uuid) -> String {
    format!("dora/log/dataflow/{dataflow_id}/*/*")
}

/// Zenoh base key expression for subscribing to log messages from a specific
/// node in a dataflow.
///
/// Format: `dora/log/dataflow/{dataflow_id}/node/{node_id}`
///
/// Append a level suffix (e.g. `/info`) to form a complete subscribe topic.
#[cfg(feature = "zenoh")]
pub fn zenoh_log_base_topic_for_dataflow_node(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
) -> String {
    format!("dora/log/dataflow/{dataflow_id}/node/{node_id}")
}

/// Zenoh base key expression for subscribing to log messages from **all**
/// sources (nodes and daemons) of a build.
///
/// Format: `dora/log/build/{build_id}/*/*`
///
/// Append a level suffix (e.g. `/info`) to form a complete subscribe topic.
#[cfg(feature = "zenoh")]
pub fn zenoh_log_base_topic_for_build(build_id: &dora_message::BuildId) -> String {
    format!("dora/log/build/{build_id}/*/*")
}

/// Zenoh key expression for publishing a log message originating from a
/// specific node in a dataflow.
///
/// Format: `dora/log/dataflow/{dataflow_id}/node/{node_id}/{level}`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_topic_for_dataflow_node(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
    level: &dora_message::common::LogLevelOrStdout,
) -> String {
    let suffix = log_level_topic_suffix(level);
    format!("dora/log/dataflow/{dataflow_id}/node/{node_id}/{suffix}")
}

/// Zenoh key expression for publishing a daemon-level log message (not
/// associated with a specific node) for a dataflow.
///
/// Format: `dora/log/dataflow/{dataflow_id}/daemon/{daemon_id}/{level}`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_topic_for_dataflow_daemon(
    dataflow_id: uuid::Uuid,
    daemon_id: &dora_message::common::DaemonId,
    level: &dora_message::common::LogLevelOrStdout,
) -> String {
    let suffix = log_level_topic_suffix(level);
    format!("dora/log/dataflow/{dataflow_id}/daemon/{daemon_id}/{suffix}")
}

/// Zenoh key expression for publishing a log message originating from a
/// specific node during a build.
///
/// Format: `dora/log/build/{build_id}/node/{node_id}/{level}`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_topic_for_build_node(
    build_id: &dora_message::BuildId,
    node_id: &dora_message::id::NodeId,
    level: &dora_message::common::LogLevelOrStdout,
) -> String {
    let suffix = log_level_topic_suffix(level);
    format!("dora/log/build/{build_id}/node/{node_id}/{suffix}")
}

/// Zenoh key expression for publishing a daemon-level log message during a
/// build (not associated with a specific node).
///
/// Format: `dora/log/build/{build_id}/daemon/{daemon_id}/{level}`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_topic_for_build_daemon(
    build_id: &dora_message::BuildId,
    daemon_id: &dora_message::common::DaemonId,
    level: &dora_message::common::LogLevelOrStdout,
) -> String {
    let suffix = log_level_topic_suffix(level);
    format!("dora/log/build/{build_id}/daemon/{daemon_id}/{suffix}")
}
