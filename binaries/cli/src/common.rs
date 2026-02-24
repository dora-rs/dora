use crate::{LOCALHOST, formatting::FormatDataflowError};
use dora_core::{
    descriptor::{Descriptor, source_is_url},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, dora_coordinator_port_rpc},
};
use dora_download::download_file;
use dora_message::{
    cli_to_coordinator::CliControlClient,
    coordinator_to_cli::{DataflowList, DataflowResult},
    tarpc::{self, client, tokio_serde},
};
use eyre::{Context, ContextCompat, bail};
use std::{
    env::current_dir,
    future::Future,
    net::IpAddr,
    path::{Path, PathBuf},
    time::Duration,
};
use uuid::Uuid;

/// Call a tarpc RPC method, unwrapping the two Result layers:
///
/// 1. Transport-level error (e.g. `RpcError`)
/// 2. `Result<T, String>` from the application (coordinator) layer
pub(crate) async fn rpc<T, E: std::error::Error + Send + Sync + 'static>(
    operation: &str,
    future: impl Future<Output = Result<Result<T, String>, E>>,
) -> eyre::Result<T> {
    future
        .await
        .wrap_err_with(|| format!("RPC transport error during {operation}"))?
        .map_err(|e| eyre::eyre!("{operation} failed: {e}"))
}

/// Create a tarpc context with a long deadline (10 minutes) for RPCs that
/// may block for an extended time (e.g. `wait_for_build`, `stop`, `destroy`).
///
/// The default `tarpc::context::current()` has a 10-second deadline which
/// is too short for operations that wait on dataflow lifecycle events.
pub(crate) fn long_context() -> tarpc::context::Context {
    let mut ctx = tarpc::context::current();
    ctx.deadline = std::time::Instant::now() + Duration::from_secs(600);
    ctx
}

pub(crate) fn handle_dataflow_result(
    result: DataflowResult,
    uuid: Option<Uuid>,
) -> Result<(), eyre::Error> {
    if result.is_ok() {
        Ok(())
    } else {
        Err(match uuid {
            Some(uuid) => {
                eyre::eyre!("Dataflow {uuid} failed:\n{}", FormatDataflowError(&result))
            }
            None => {
                eyre::eyre!("Dataflow failed:\n{}", FormatDataflowError(&result))
            }
        })
    }
}

pub(crate) async fn query_running_dataflows(
    client: &CliControlClient,
) -> eyre::Result<DataflowList> {
    rpc("list dataflows", client.list(tarpc::context::current())).await
}

pub(crate) async fn resolve_dataflow_identifier_interactive(
    client: &CliControlClient,
    name_or_uuid: Option<&str>,
) -> eyre::Result<Uuid> {
    if let Some(uuid) = name_or_uuid.and_then(|s| Uuid::parse_str(s).ok()) {
        return Ok(uuid);
    }

    let list = query_running_dataflows(client)
        .await
        .wrap_err("failed to query running dataflows")?;
    let active: Vec<dora_message::coordinator_to_cli::DataflowIdAndName> = list.get_active();
    if let Some(name) = name_or_uuid {
        let Some(dataflow) = active.iter().find(|it| it.name.as_deref() == Some(name)) else {
            bail!("No dataflow with name `{name}` is running");
        };
        return Ok(dataflow.uuid);
    }
    Ok(match &active[..] {
        [] => bail!("No dataflows are running"),
        [entry] => entry.uuid,
        _ => {
            inquire::Select::new("Choose dataflow:", active)
                .prompt()?
                .uuid
        }
    })
}

#[derive(Debug, clap::Args)]
pub(crate) struct CoordinatorOptions {
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
}

impl CoordinatorOptions {
    pub async fn connect_rpc(&self) -> eyre::Result<CliControlClient> {
        connect_and_check_version(self.coordinator_addr, self.coordinator_port).await
    }
}

/// Connect to the coordinator's tarpc RPC service.
pub(crate) async fn connect_to_coordinator_rpc(
    addr: IpAddr,
    control_port: u16,
) -> eyre::Result<CliControlClient> {
    let rpc_port = dora_coordinator_port_rpc(control_port);
    let transport =
        tarpc::serde_transport::tcp::connect((addr, rpc_port), tokio_serde::formats::Json::default)
            .await
            .context("failed to connect tarpc client to coordinator")?;
    let client = CliControlClient::new(client::Config::default(), transport).spawn();
    Ok(client)
}

pub(crate) async fn resolve_dataflow(dataflow: String) -> eyre::Result<PathBuf> {
    let dataflow = if source_is_url(&dataflow) {
        // try to download the shared library
        let target_path = current_dir().context("Could not access the current dir")?;
        download_file(&dataflow, &target_path)
            .await
            .wrap_err("failed to download dataflow yaml file")?
    } else {
        PathBuf::from(dataflow)
    };
    Ok(dataflow)
}

pub(crate) async fn local_working_dir(
    dataflow_path: &Path,
    dataflow_descriptor: &Descriptor,
    client: &CliControlClient,
) -> eyre::Result<Option<PathBuf>> {
    Ok(
        if dataflow_descriptor
            .nodes
            .iter()
            .all(|n| n.deploy.as_ref().map(|d| d.machine.as_ref()).is_none())
            && cli_and_daemon_on_same_machine(client).await?
        {
            Some(
                dunce::canonicalize(dataflow_path)
                    .context("failed to canonicalize dataflow file path")?
                    .parent()
                    .context("dataflow path has no parent dir")?
                    .to_owned(),
            )
        } else {
            None
        },
    )
}

pub(crate) async fn cli_and_daemon_on_same_machine(
    client: &CliControlClient,
) -> eyre::Result<bool> {
    rpc(
        "check if CLI and daemon on same machine",
        client.cli_and_default_daemon_on_same_machine(tarpc::context::current()),
    )
    .await
}

pub(crate) fn write_events_to() -> Option<PathBuf> {
    std::env::var("DORA_WRITE_EVENTS_TO")
        .ok()
        .map(PathBuf::from)
}

/// Connect to the coordinator and check that the message format version is compatible.
pub(crate) async fn connect_and_check_version(
    addr: IpAddr,
    control_port: u16,
) -> eyre::Result<CliControlClient> {
    let client = connect_to_coordinator_rpc(addr, control_port).await?;
    check_coordinator_version(&client).await?;
    Ok(client)
}

/// Check that the coordinator's message format version matches this CLI's.
pub(crate) async fn check_coordinator_version(client: &CliControlClient) -> eyre::Result<()> {
    let version_info = match client.get_version(tarpc::context::current()).await {
        Ok(v) => v,
        Err(_) => {
            bail!(
                "Failed to query coordinator version. \
                 The coordinator may be running an older version of dora \
                 that is incompatible with this CLI (message format v{}).",
                dora_message::VERSION
            );
        }
    };
    let local = semver::Version::parse(dora_message::VERSION)
        .map_err(|e| eyre::eyre!("failed to parse local message format version: {e}"))?;
    let remote = semver::Version::parse(&version_info.message_format_version)
        .map_err(|e| eyre::eyre!("failed to parse coordinator message format version: {e}"))?;
    if !semver_compatible(&local, &remote) {
        bail!(
            "CLI message format (v{local}) is not compatible with \
             coordinator message format (v{remote}). \
             Please ensure CLI and coordinator are the same version."
        );
    }
    Ok(())
}

/// Check if two semver versions are compatible using Rust/Cargo conventions:
/// - For `0.0.x`: only the exact same version is compatible.
/// - For `0.x.y` (x > 0): major and minor must match (patch may differ).
/// - For `>=1.0.0`: only major must match.
fn semver_compatible(a: &semver::Version, b: &semver::Version) -> bool {
    if a.major != b.major {
        return false;
    }
    if a.major == 0 {
        if a.minor != b.minor {
            return false;
        }
        if a.minor == 0 {
            return a.patch == b.patch;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(s: &str) -> semver::Version {
        semver::Version::parse(s).unwrap()
    }

    #[test]
    fn test_semver_compatible_pre_1_0() {
        // Same minor: compatible (patch may differ)
        assert!(semver_compatible(&v("0.7.0"), &v("0.7.0")));
        assert!(semver_compatible(&v("0.7.0"), &v("0.7.1")));
        assert!(semver_compatible(&v("0.7.3"), &v("0.7.1")));

        // Different minor: incompatible
        assert!(!semver_compatible(&v("0.7.0"), &v("0.8.0")));
        assert!(!semver_compatible(&v("0.6.0"), &v("0.7.0")));
    }

    #[test]
    fn test_semver_compatible_0_0_x() {
        // 0.0.x requires exact patch match
        assert!(semver_compatible(&v("0.0.1"), &v("0.0.1")));
        assert!(!semver_compatible(&v("0.0.1"), &v("0.0.2")));
    }

    #[test]
    fn test_semver_compatible_post_1_0() {
        // Same major: compatible
        assert!(semver_compatible(&v("1.0.0"), &v("1.0.0")));
        assert!(semver_compatible(&v("1.0.0"), &v("1.2.3")));
        assert!(semver_compatible(&v("2.1.0"), &v("2.5.3")));

        // Different major: incompatible
        assert!(!semver_compatible(&v("1.0.0"), &v("2.0.0")));
    }
}
