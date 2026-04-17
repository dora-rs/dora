use eyre::{Context, bail};
use std::{
    collections::{BTreeMap, HashSet},
    net::IpAddr,
    path::Path,
};

use dora_core::topics::DORA_COORDINATOR_PORT_WS_DEFAULT;

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClusterConfig {
    pub coordinator: CoordinatorConfig,
    pub machines: Vec<MachineConfig>,
}

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CoordinatorConfig {
    pub addr: IpAddr,
    #[serde(default = "default_port")]
    pub port: u16,
}

fn default_port() -> u16 {
    DORA_COORDINATOR_PORT_WS_DEFAULT
}

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MachineConfig {
    pub id: String,
    pub host: String,
    #[serde(default)]
    pub user: Option<String>,
    /// Labels for label-based scheduling (e.g. `gpu: "true"`, `arch: arm64`).
    #[serde(default)]
    pub labels: BTreeMap<String, String>,
}

impl ClusterConfig {
    pub fn load(path: &Path) -> eyre::Result<Self> {
        let raw = std::fs::read_to_string(path)
            .with_context(|| format!("failed to read `{}`", path.display()))?;
        let config: Self = serde_yaml::from_str(&raw)
            .with_context(|| format!("failed to parse `{}`", path.display()))?;
        config.validate()?;
        Ok(config)
    }

    fn validate(&self) -> eyre::Result<()> {
        if self.machines.is_empty() {
            bail!("cluster config must define at least one machine");
        }
        let mut seen = HashSet::new();
        for m in &self.machines {
            if m.id.is_empty() {
                bail!("machine id must not be empty");
            }
            if !seen.insert(&m.id) {
                bail!("duplicate machine id: `{}`", m.id);
            }
            if m.host.is_empty() {
                bail!("machine `{}` host must not be empty", m.id);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    fn write_yaml(content: &str) -> NamedTempFile {
        let mut f = NamedTempFile::new().unwrap();
        f.write_all(content.as_bytes()).unwrap();
        f
    }

    #[test]
    fn parse_minimal() {
        let f = write_yaml(
            "coordinator:\n  addr: 192.168.1.100\nmachines:\n  - id: arm\n    host: 192.168.1.101\n",
        );
        let cfg = ClusterConfig::load(f.path()).unwrap();
        assert_eq!(cfg.coordinator.port, DORA_COORDINATOR_PORT_WS_DEFAULT);
        assert_eq!(cfg.machines.len(), 1);
        assert_eq!(cfg.machines[0].id, "arm");
        assert!(cfg.machines[0].user.is_none());
        assert!(cfg.machines[0].labels.is_empty());
    }

    #[test]
    fn parse_full() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\n  port: 7000\nmachines:\n  - id: a\n    host: 10.0.0.2\n    user: bob\n    labels:\n      gpu: \"true\"\n      arch: arm64\n  - id: b\n    host: 10.0.0.3\n",
        );
        let cfg = ClusterConfig::load(f.path()).unwrap();
        assert_eq!(cfg.coordinator.port, 7000);
        assert_eq!(cfg.machines.len(), 2);
        assert_eq!(cfg.machines[0].user.as_deref(), Some("bob"));
        assert_eq!(
            cfg.machines[0].labels.get("gpu").map(|s| s.as_str()),
            Some("true")
        );
        assert_eq!(
            cfg.machines[0].labels.get("arch").map(|s| s.as_str()),
            Some("arm64")
        );
    }

    #[test]
    fn reject_duplicate_ids() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: dup\n    host: a\n  - id: dup\n    host: b\n",
        );
        let err = ClusterConfig::load(f.path()).unwrap_err();
        assert!(err.to_string().contains("duplicate machine id"));
    }

    #[test]
    fn reject_empty_machines() {
        let f = write_yaml("coordinator:\n  addr: 10.0.0.1\nmachines: []\n");
        let err = ClusterConfig::load(f.path()).unwrap_err();
        assert!(err.to_string().contains("at least one machine"));
    }

    #[test]
    fn reject_empty_id() {
        let f =
            write_yaml("coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: \"\"\n    host: a\n");
        let err = ClusterConfig::load(f.path()).unwrap_err();
        assert!(err.to_string().contains("must not be empty"));
    }

    #[test]
    fn reject_unknown_field() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\n  bogus: true\nmachines:\n  - id: a\n    host: b\n",
        );
        assert!(ClusterConfig::load(f.path()).is_err());
    }
}
