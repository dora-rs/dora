use super::Executable;
use crate::LISTEN_DEFAULT;
use adora_coordinator::Event;
use adora_coordinator::{CoordinatorStore, InMemoryStore};
use adora_core::topics::ADORA_COORDINATOR_PORT_WS_DEFAULT;

#[cfg(feature = "tracing")]
use adora_tracing::TracingBuilder;

use eyre::Context;
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use tokio::runtime::Builder;
use tracing::level_filters::LevelFilter;

#[derive(Debug, clap::Args)]
/// Run coordinator
pub struct Coordinator {
    /// Network interface to bind to
    #[clap(long, default_value_t = LISTEN_DEFAULT)]
    interface: IpAddr,
    /// Port number to bind to
    #[clap(long, default_value_t = ADORA_COORDINATOR_PORT_WS_DEFAULT)]
    port: u16,
    /// Suppresses all log output to stdout.
    #[clap(long)]
    quiet: bool,
    /// State store backend: "memory" (default) or "redb" or "redb:/path/to/file.redb".
    /// The "redb" backend persists coordinator state to disk so it survives restarts.
    /// Requires the `redb-backend` feature.
    #[clap(long, default_value = "memory")]
    store: String,
}

impl Executable for Coordinator {
    fn execute(self) -> eyre::Result<()> {
        #[cfg(feature = "tracing")]
        {
            let name = "adora-coordinator";
            let mut builder = TracingBuilder::new(name);
            if !self.quiet {
                builder = builder.with_stdout("info", false);
            }
            builder = builder.with_file(name, LevelFilter::INFO)?;
            builder
                .build()
                .wrap_err("failed to set up tracing subscriber")?;
        }

        let store = create_store(&self.store)?;

        let rt = Builder::new_multi_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;
        rt.block_on(async {
            let bind = SocketAddr::new(self.interface, self.port);
            let (port, task) =
                adora_coordinator::start(bind, futures::stream::empty::<Event>(), store).await?;
            if !self.quiet {
                println!("Listening on port {port}");
            }
            task.await
        })
        .context("failed to run adora-coordinator")
    }
}

fn create_store(spec: &str) -> eyre::Result<Arc<dyn CoordinatorStore>> {
    match spec {
        "memory" => Ok(Arc::new(InMemoryStore::new())),

        #[cfg(feature = "redb-backend")]
        "redb" => {
            let path = default_redb_path()?;
            Ok(Arc::new(
                adora_coordinator::adora_coordinator_store::RedbStore::open(&path)?,
            ))
        }

        #[cfg(feature = "redb-backend")]
        s if s.starts_with("redb:") => {
            let raw = &s["redb:".len()..];
            if raw.is_empty() {
                eyre::bail!("redb path cannot be empty; use `redb` for the default path");
            }
            let path = std::path::PathBuf::from(raw);
            if path
                .components()
                .any(|c| c == std::path::Component::ParentDir)
            {
                eyre::bail!("redb path must not contain `..` components");
            }
            Ok(Arc::new(
                adora_coordinator::adora_coordinator_store::RedbStore::open(&path)?,
            ))
        }

        #[cfg(not(feature = "redb-backend"))]
        s if s == "redb" || s.starts_with("redb:") => {
            eyre::bail!(
                "redb store requested but the `redb-backend` feature is not enabled. \
                 Rebuild with `--features redb-backend`."
            )
        }

        other => eyre::bail!("unknown store backend: `{other}` (expected `memory` or `redb`)"),
    }
}

#[cfg(feature = "redb-backend")]
fn default_redb_path() -> eyre::Result<std::path::PathBuf> {
    let home = std::env::var("HOME")
        .or_else(|_| std::env::var("USERPROFILE"))
        .unwrap_or_else(|_| ".".into());
    let dir = std::path::PathBuf::from(home).join(".adora");
    // Set restrictive umask before creating directory to close the TOCTOU window
    // between creation and set_permissions.
    #[cfg(unix)]
    {
        // SAFETY: umask is always safe to call and has no undefined behavior.
        let old = unsafe { libc::umask(0o077) };
        let result = std::fs::create_dir_all(&dir);
        unsafe { libc::umask(old) };
        result?;
    }
    #[cfg(not(unix))]
    std::fs::create_dir_all(&dir)?;
    // Defense-in-depth: also set permissions explicitly.
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o700))?;
    }
    Ok(dir.join("coordinator.redb"))
}
