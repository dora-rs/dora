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
    /// Shared Zenoh peer endpoint that all daemons use as a rendezvous
    /// for cross-daemon discovery (e.g. `tcp/192.168.1.1:5456`). When
    /// set, `dora cluster up` passes it to every daemon via
    /// `dora daemon --zenoh-peer <ep>`. The first daemon to bind the
    /// endpoint serves as the gossip hub; the rest fall through to
    /// connect-only. Set this when running on networks without working
    /// multicast (dev containers, hardened deployments, many CI
    /// runners) — otherwise daemons can't find each other.
    #[serde(default)]
    pub zenoh_peer: Option<String>,
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
    /// Custom SSH port. Defaults to the SSH client default (22).
    #[serde(default)]
    pub port: Option<u16>,
    /// Custom daemon local-listen port. Defaults to 53291.
    /// Set when running multiple daemons on the same host
    /// (see `examples/multiple-daemons`).
    #[serde(default)]
    pub daemon_port: Option<u16>,
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
        // `zenoh_peer` is interpolated verbatim into the same remote command
        // (`dora daemon --zenoh-peer {ep}`), so it needs the same guard as the
        // id/labels below — but with a charset wide enough for endpoint syntax
        // (`tcp/1.2.3.4:7447`, IPv6 literals), which the id charset would reject.
        if let Some(ep) = &self.zenoh_peer {
            validate_endpoint_shell_safe(&format!("zenoh_peer `{ep}`"), ep)?;
        }
        let mut seen = HashSet::new();
        for m in &self.machines {
            if m.id.is_empty() {
                bail!("machine id must not be empty");
            }
            // `machine.id` and every label key/value are interpolated verbatim
            // into the shell command that `dora cluster up` runs over SSH
            // (`dora daemon --machine-id {id} --labels {k}={v},... >
            // /tmp/dora-daemon-{id}.log`). A value containing whitespace or shell
            // metacharacters would corrupt that command (or, for the id, the log
            // path), so restrict these fields to the same safe identifier charset
            // dora uses for node/data ids (`libraries/message/src/id.rs`).
            // `host`/`user` are deliberately left unrestricted: they are network
            // addresses that legitimately carry characters outside this set
            // (e.g. IPv6 literals), and are folded into the ssh *target* rather
            // than the remote command string. `zenoh_peer` *does* land in the
            // remote command, so it is validated (with a wider charset) above.
            validate_shell_safe(&format!("machine id `{}`", m.id), &m.id)?;
            for (k, v) in &m.labels {
                validate_shell_safe(&format!("label key `{k}` on machine `{}`", m.id), k)?;
                validate_shell_safe(&format!("label value `{v}` on machine `{}`", m.id), v)?;
            }
            if !seen.insert(&m.id) {
                bail!("duplicate machine id: `{}`", m.id);
            }
            if m.host.is_empty() {
                bail!("machine `{}` host must not be empty", m.id);
            }
            // `host`/`user` are folded into the ssh *target* (`{user}@{host}`)
            // and passed to the local `ssh` binary. They are deliberately not
            // run through `validate_shell_safe` (a target legitimately carries
            // `:`/`[`/`]` for IPv6 literals, `@`, etc.), but a value that begins
            // with `-` is parsed by `ssh`'s own argument parser as an *option*
            // rather than a hostname — e.g. `-oProxyCommand=...` executes an
            // arbitrary command on the local machine before connecting (the
            // classic ssh/git argument-injection class, cf. CVE-2017-1000117).
            // Reject a leading dash on both fields; `run_ssh` additionally
            // passes `--` before the target as defense-in-depth.
            validate_no_leading_dash(&format!("machine `{}` host", m.id), &m.host)?;
            if let Some(user) = &m.user {
                validate_no_leading_dash(&format!("machine `{}` user", m.id), user)?;
            }
            // Reject daemon_port = 0: the daemon would bind an ephemeral port
            // but exports DORA_DAEMON_LOCAL_LISTEN_PORT=0 to spawned nodes
            // (binaries/cli/src/command/daemon.rs), so they fail to connect.
            if m.daemon_port == Some(0) {
                bail!("machine `{}` daemon_port must not be 0", m.id);
            }
        }
        Ok(())
    }
}

/// Reject a value that will be interpolated into the remote SSH command unless
/// it consists only of the safe identifier charset `[a-zA-Z0-9_.-]`. `what`
/// names the field for the error message.
fn validate_shell_safe(what: &str, value: &str) -> eyre::Result<()> {
    if let Some(ch) = value
        .chars()
        .find(|c| !c.is_ascii_alphanumeric() && *c != '_' && *c != '-' && *c != '.')
    {
        bail!("{what} contains invalid character `{ch}` -- only [a-zA-Z0-9_.-] are allowed");
    }
    Ok(())
}

/// Reject a value that begins with `-`, which the local `ssh` binary would
/// otherwise parse as an option rather than a hostname/user (argument
/// injection). Unlike [`validate_shell_safe`] this places no charset
/// restriction, so IPv6 literals and normal `user@host` values stay valid.
fn validate_no_leading_dash(what: &str, value: &str) -> eyre::Result<()> {
    if value.starts_with('-') {
        bail!("{what} must not start with `-` (would be parsed as an ssh option)");
    }
    Ok(())
}

/// Like [`validate_shell_safe`], but also permits the extra characters a Zenoh
/// endpoint string carries — `/` and `:` in `tcp/1.2.3.4:7447`, and the
/// `[`/`]` around an IPv6 literal like `tcp/[::1]:7447`. It still rejects
/// whitespace and shell metacharacters (`;`, `|`, `$`, backtick, ...), so the
/// value cannot break out of the remote command.
fn validate_endpoint_shell_safe(what: &str, value: &str) -> eyre::Result<()> {
    if let Some(ch) = value.chars().find(|c| {
        !c.is_ascii_alphanumeric() && !matches!(c, '_' | '-' | '.' | '/' | ':' | '[' | ']')
    }) {
        bail!(
            "{what} contains invalid character `{ch}` -- only [a-zA-Z0-9_.:/] and `[`/`]` are allowed"
        );
    }
    Ok(())
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
        assert!(cfg.machines[0].port.is_none());
        assert!(cfg.machines[0].daemon_port.is_none());
        assert!(cfg.machines[0].labels.is_empty());
    }

    #[test]
    fn parse_with_daemon_port() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: 10.0.0.2\n    daemon_port: 53292\n",
        );
        let cfg = ClusterConfig::load(f.path()).unwrap();
        assert_eq!(cfg.machines[0].daemon_port, Some(53292));
    }

    #[test]
    fn reject_invalid_daemon_port() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: 10.0.0.2\n    daemon_port: 99999\n",
        );
        assert!(ClusterConfig::load(f.path()).is_err());
    }

    #[test]
    fn reject_zero_daemon_port() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: 10.0.0.2\n    daemon_port: 0\n",
        );
        let err = ClusterConfig::load(f.path()).unwrap_err();
        assert!(err.to_string().contains("daemon_port must not be 0"));
    }

    #[test]
    fn parse_with_zenoh_peer() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nzenoh_peer: tcp/10.0.0.1:5456\nmachines:\n  - id: a\n    host: 10.0.0.2\n",
        );
        let cfg = ClusterConfig::load(f.path()).unwrap();
        assert_eq!(cfg.zenoh_peer.as_deref(), Some("tcp/10.0.0.1:5456"));
    }

    #[test]
    fn parse_without_zenoh_peer_defaults_to_none() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: 10.0.0.2\n",
        );
        let cfg = ClusterConfig::load(f.path()).unwrap();
        assert!(cfg.zenoh_peer.is_none());
    }

    #[test]
    fn parse_with_port() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: 10.0.0.2\n    port: 2222\n",
        );
        let cfg = ClusterConfig::load(f.path()).unwrap();
        assert_eq!(cfg.machines[0].port, Some(2222));
    }

    #[test]
    fn reject_invalid_port() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: 10.0.0.2\n    port: 99999\n",
        );
        assert!(ClusterConfig::load(f.path()).is_err());
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
    fn reject_id_with_shell_metacharacters() {
        // An id with a space or shell metacharacter would corrupt the remote
        // `dora daemon --machine-id {id} ...` command run over SSH.
        for bad in ["a b", "a;rm -rf /", "a$(whoami)", "a/b", "a`id`"] {
            let f = write_yaml(&format!(
                "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: \"{bad}\"\n    host: h\n"
            ));
            let err = ClusterConfig::load(f.path())
                .unwrap_err()
                .to_string()
                .to_lowercase();
            assert!(
                err.contains("invalid character"),
                "id `{bad}` should be rejected, got: {err}"
            );
        }
    }

    #[test]
    fn reject_label_with_shell_metacharacters() {
        // Label keys/values are interpolated into `--labels k=v` on the remote
        // shell, so a metacharacter there is the same injection vector as the id.
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: h\n    labels:\n      gpu: \"x;rm -rf /\"\n",
        );
        let err = ClusterConfig::load(f.path())
            .unwrap_err()
            .to_string()
            .to_lowercase();
        assert!(
            err.contains("invalid character") && err.contains("label value"),
            "malicious label value should be rejected, got: {err}"
        );
    }

    #[test]
    fn reject_zenoh_peer_with_shell_metacharacters() {
        // `zenoh_peer` is interpolated into `dora daemon --zenoh-peer {ep}` on
        // the remote shell, so a metacharacter there is an injection vector too.
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nzenoh_peer: \"tcp/1.2.3.4:7447;rm -rf /\"\nmachines:\n  - id: a\n    host: h\n",
        );
        let err = ClusterConfig::load(f.path())
            .unwrap_err()
            .to_string()
            .to_lowercase();
        assert!(
            err.contains("invalid character") && err.contains("zenoh_peer"),
            "malicious zenoh_peer should be rejected, got: {err}"
        );
    }

    #[test]
    fn accept_ipv6_zenoh_peer() {
        // The endpoint charset must still permit a legitimate IPv6 literal
        // endpoint, whose `[`/`]`/`:` the id charset would reject.
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nzenoh_peer: \"tcp/[::1]:7447\"\nmachines:\n  - id: a\n    host: h\n",
        );
        let cfg = ClusterConfig::load(f.path()).unwrap();
        assert_eq!(cfg.zenoh_peer.as_deref(), Some("tcp/[::1]:7447"));
    }

    #[test]
    fn reject_host_with_leading_dash() {
        // A `host` beginning with `-` is parsed by the local `ssh` binary as an
        // option (e.g. `-oProxyCommand=touch /tmp/pwned;false`), giving arbitrary
        // local command execution before any network connection is made.
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: \"-oProxyCommand=touch /tmp/pwned;false\"\n",
        );
        let err = ClusterConfig::load(f.path()).unwrap_err().to_string();
        assert!(
            err.contains("must not start with `-`") && err.contains("host"),
            "host with leading dash should be rejected, got: {err}"
        );
    }

    #[test]
    fn reject_user_with_leading_dash() {
        // `ssh_target` yields `{user}@{host}`, so a `user` beginning with `-`
        // produces a target string that also starts with `-` — same vector.
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: h\n    user: \"-oProxyCommand=x\"\n",
        );
        let err = ClusterConfig::load(f.path()).unwrap_err().to_string();
        assert!(
            err.contains("must not start with `-`") && err.contains("user"),
            "user with leading dash should be rejected, got: {err}"
        );
    }

    #[test]
    fn accept_ipv6_host() {
        // An IPv6 literal host carries `:` (and may be bracketed) — the id
        // charset would reject it, but host/user are only checked for a leading
        // dash, so it must stay valid.
        for host in ["::1", "2001:db8::1", "192.168.1.10"] {
            let f = write_yaml(&format!(
                "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: a\n    host: \"{host}\"\n"
            ));
            assert!(
                ClusterConfig::load(f.path()).is_ok(),
                "host `{host}` should be accepted"
            );
        }
    }

    #[test]
    fn accept_conventional_ids() {
        // Hostname-style ids (alphanumerics plus `.`, `_`, `-`) stay valid.
        for good in ["arm", "jetson-01", "node_2", "gpu.host-1"] {
            let f = write_yaml(&format!(
                "coordinator:\n  addr: 10.0.0.1\nmachines:\n  - id: {good}\n    host: h\n"
            ));
            assert!(
                ClusterConfig::load(f.path()).is_ok(),
                "id `{good}` should be accepted"
            );
        }
    }

    #[test]
    fn reject_unknown_field() {
        let f = write_yaml(
            "coordinator:\n  addr: 10.0.0.1\n  bogus: true\nmachines:\n  - id: a\n    host: b\n",
        );
        assert!(ClusterConfig::load(f.path()).is_err());
    }
}
