use std::net::{IpAddr, Ipv4Addr};

pub const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const DORA_COORDINATOR_PORT_DEFAULT: u16 = 53290;
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT: u16 = 53291;
pub const DORA_COORDINATOR_PORT_CONTROL_DEFAULT: u16 = 6012;

pub const MANUAL_STOP: &str = "dora/stop";

#[cfg(feature = "zenoh")]
pub async fn open_zenoh_session(coordinator_addr: Option<IpAddr>) -> eyre::Result<zenoh::Session> {
    use eyre::{eyre, Context};
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
            let mut zenoh_config = zenoh::Config::default();

            if let Some(addr) = coordinator_addr {
                // Linkstate make it possible to connect two daemons on different network through a public daemon
                // TODO: There is currently a CI/CD Error in windows linkstate.
                if cfg!(not(target_os = "windows")) {
                    zenoh_config
                        .insert_json5("routing/peer", r#"{ mode: "linkstate" }"#)
                        .unwrap();
                }

                zenoh_config
                    .insert_json5(
                        "connect/endpoints",
                        &format!(
                            r#"{{ router: ["tcp/[::]:7447"], peer: ["tcp/{}:5456"] }}"#,
                            addr
                        ),
                    )
                    .unwrap();
                zenoh_config
                    .insert_json5(
                        "listen/endpoints",
                        r#"{ router: ["tcp/[::]:7447"], peer: ["tcp/[::]:5456"] }"#,
                    )
                    .unwrap();
                if cfg!(target_os = "macos") {
                    warn!("disabling multicast on macos systems. Enable it with the ZENOH_CONFIG env variable or file");
                    zenoh_config
                        .insert_json5("scouting/multicast", r#"{ enabled: false }"#)
                        .unwrap();
                }
            };
            if let Ok(zenoh_session) = zenoh::open(zenoh_config).await {
                zenoh_session
            } else {
                warn!("failed to open zenoh session, retrying with default config + coordinator");
                let mut zenoh_config = zenoh::Config::default();
                // Linkstate make it possible to connect two daemons on different network through a public daemon
                // TODO: There is currently a CI/CD Error in windows linkstate.
                if cfg!(not(target_os = "windows")) {
                    zenoh_config
                        .insert_json5("routing/peer", r#"{ mode: "linkstate" }"#)
                        .unwrap();
                }

                if let Some(addr) = coordinator_addr {
                    zenoh_config
                        .insert_json5(
                            "connect/endpoints",
                            &format!(
                                r#"{{ router: ["tcp/[::]:7447"], peer: ["tcp/{}:5456"] }}"#,
                                addr
                            ),
                        )
                        .unwrap();
                    if cfg!(target_os = "macos") {
                        warn!("disabling multicast on macos systems. Enable it with the ZENOH_CONFIG env variable or file");
                        zenoh_config
                            .insert_json5("scouting/multicast", r#"{ enabled: false }"#)
                            .unwrap();
                    }
                }
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
