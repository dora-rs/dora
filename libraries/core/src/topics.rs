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
        zenoh_config
            .insert_json5("routing/peer", r#"{ mode: "linkstate" }"#)
            .unwrap();
    }

    if let Some(port) = listen_port {
        zenoh_config
            .insert_json5("listen/endpoints", &format!(r#"["tcp/0.0.0.0:{port}"]"#))
            .unwrap();
    }

    if let Some(addr) = coordinator_addr {
        zenoh_config
            .insert_json5(
                "connect/endpoints",
                &format!(r#"["tcp/{}:{}"]"#, addr, DORA_ZENOH_PEER_PORT_DEFAULT),
            )
            .unwrap();
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

/// Zenoh key expression for publishing a log message originating from a
/// specific node in a dataflow.
///
/// Format: `dora/log/dataflow/{dataflow_id}/node/{node_id}`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_topic_for_dataflow_node(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
) -> String {
    format!("dora/log/dataflow/{dataflow_id}/node/{node_id}")
}

/// Zenoh key expression for publishing a daemon-level log message (not
/// associated with a specific node) for a dataflow.
///
/// Format: `dora/log/dataflow/{dataflow_id}/daemon/{daemon_id}`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_topic_for_dataflow_daemon(
    dataflow_id: uuid::Uuid,
    daemon_id: &dora_message::common::DaemonId,
) -> String {
    format!("dora/log/dataflow/{dataflow_id}/daemon/{daemon_id}")
}

/// Zenoh key expression for subscribing to log messages from **all** sources
/// (nodes and daemons) of a dataflow.
///
/// Format: `dora/log/dataflow/{dataflow_id}/**`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_subscribe_all_for_dataflow(dataflow_id: uuid::Uuid) -> String {
    format!("dora/log/dataflow/{dataflow_id}/**")
}

/// Zenoh key expression for publishing a log message originating from a
/// specific node during a build.
///
/// Format: `dora/log/build/{build_id}/node/{node_id}`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_topic_for_build_node(
    build_id: &dora_message::BuildId,
    node_id: &dora_message::id::NodeId,
) -> String {
    format!("dora/log/build/{build_id}/node/{node_id}")
}

/// Zenoh key expression for publishing a daemon-level log message during a
/// build (not associated with a specific node).
///
/// Format: `dora/log/build/{build_id}/daemon/{daemon_id}`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_topic_for_build_daemon(
    build_id: &dora_message::BuildId,
    daemon_id: &dora_message::common::DaemonId,
) -> String {
    format!("dora/log/build/{build_id}/daemon/{daemon_id}")
}

/// Zenoh key expression for subscribing to log messages from **all** sources
/// (nodes and daemons) of a build.
///
/// Format: `dora/log/build/{build_id}/**`
#[cfg(feature = "zenoh")]
pub fn zenoh_log_subscribe_all_for_build(build_id: &dora_message::BuildId) -> String {
    format!("dora/log/build/{build_id}/**")
}
