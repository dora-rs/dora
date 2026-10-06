//! #2721: a late dynamic peer must connect without multicast or gossip.
//! Run with `cargo test -p dora-examples --test multi-daemon-e2e dynamic_peering`.
use std::{path::Path, process::Command, time::Duration};

use super::{Deployment, bin, ensure_built, free_ports, port_open};

fn wait_until(check: impl FnMut() -> bool) -> bool {
    super::wait_until(check, Duration::from_secs(20))
}

impl Deployment {
    fn spawn(&mut self, mut command: Command, log: &Path) -> usize {
        let file = std::fs::File::create(log).unwrap();
        command.stdout(file.try_clone().unwrap()).stderr(file);
        self.children.push(command.spawn().unwrap());
        self.logs.push(log.to_owned());
        self.children.len() - 1
    }

    fn check(&self, condition: bool, message: &str) {
        if !condition {
            self.fail(message);
        }
    }
}

fn has_both_sizes(directory: &Path, name: &str) -> bool {
    let received =
        std::fs::read_to_string(directory.join(format!("{name}.received"))).unwrap_or_default();
    received.lines().any(|l| l == "8") && received.lines().any(|l| l == "131072")
}

fn dynamic_peering_case(both_dynamic: bool, restart: bool, local: bool) {
    ensure_built();
    let ports = free_ports(2);
    let port = ports[0];
    let local_port = if local {
        dora_core::topics::DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT
    } else {
        ports[1]
    };
    // Declared before the deployment so its logs outlive the teardown.
    let directory = tempfile::tempdir().unwrap();
    let dir = directory.path().to_owned();
    let log = |name: &str| dir.join(format!("{name}.log"));
    let mut deployment = Deployment {
        dora: bin("dora"),
        coordinator_port: port,
        children: vec![],
        logs: vec![],
    };
    let overlay = dir.join("zenoh.json5");
    std::fs::write(
        &overlay,
        r#"{ scouting: { multicast: { enabled: false }, gossip: { enabled: false } } }"#,
    )
    .unwrap();
    if !local {
        let mut coordinator = Command::new(bin("dora"));
        coordinator.args([
            "coordinator",
            "--port",
            &port.to_string(),
            "--store",
            "memory",
        ]);
        deployment.spawn(coordinator, &log("coordinator"));
        deployment.check(wait_until(|| port_open(port)), "coordinator startup");
        let mut daemon = Command::new(bin("dora"));
        daemon
            .args([
                "daemon",
                "--coordinator-port",
                &port.to_string(),
                "--local-listen-port",
                &local_port.to_string(),
                "--zenoh-config-overlay",
            ])
            .arg(&overlay);
        deployment.spawn(daemon, &log("daemon"));
        deployment.check(wait_until(|| port_open(local_port)), "daemon startup");
    }
    let fixture = bin("dynamic-peering-node");
    let static_node = |name: &str| {
        format!(
            "path: {}\n    args: '{name} {}'",
            fixture.display(),
            dir.display()
        )
    };
    let first = if both_dynamic {
        "path: dynamic".to_owned()
    } else {
        static_node("first")
    };
    let yaml = format!(
        "nodes:\n  - id: anchor\n    {}\n    shared_memory_pool_size: 1048576\n    inputs:\n      tick: dora/timer/millis/20\n  - id: first\n    {first}\n    shared_memory_pool_size: 1048576\n    inputs:\n      tick: dora/timer/millis/20\n      value: second/value\n    outputs: [value]\n  - id: second\n    path: dynamic\n    shared_memory_pool_size: 1048576\n    inputs:\n      tick: dora/timer/millis/20\n      value: first/value\n    outputs: [value]\n",
        static_node("anchor")
    );
    let dataflow = dir.join("dataflow.yml");
    std::fs::write(&dataflow, yaml).unwrap();
    let run = if local {
        let mut command = Command::new(bin("dora"));
        command
            .arg("run")
            .arg(&dataflow)
            .args(["--stop-after", "5s"])
            .env("DORA_ZENOH_CONFIG_OVERLAY", &overlay);
        Some(deployment.spawn(command, &log("local-run")))
    } else {
        let start = Command::new(bin("dora"))
            .args(["start", "--detach", "--coordinator-port", &port.to_string()])
            .arg(&dataflow)
            .output()
            .unwrap();
        deployment.check(
            start.status.success(),
            &format!("start: {}", String::from_utf8_lossy(&start.stderr)),
        );
        None
    };
    deployment.check(
        wait_until(|| dir.join("anchor.ready").exists()),
        "static anchor initialized",
    );
    if !both_dynamic {
        deployment.check(
            wait_until(|| dir.join("first.ready").exists()),
            "static peer must be initialized before the dynamic peer joins",
        );
    }
    let dynamic = |name: &str| {
        let mut cmd = Command::new(&fixture);
        cmd.arg(name)
            .arg(&dir)
            .arg("dynamic")
            .env("DORA_DAEMON_LOCAL_LISTEN_PORT", local_port.to_string())
            .env("DORA_ZENOH_CONFIG_OVERLAY", &overlay)
            .env_remove("DORA_NODE_CONFIG")
            .env_remove("DORA_ZENOH_CONNECT")
            .env_remove("DORA_ZENOH_LISTEN");
        cmd
    };
    // No readiness wait between these two: exercise overlapping dynamic joins.
    if both_dynamic {
        deployment.spawn(dynamic("first"), &log("first"));
    }
    let second = deployment.spawn(dynamic("second"), &log("second"));
    deployment.check(
        wait_until(|| has_both_sizes(&dir, "first") && has_both_sizes(&dir, "second")),
        "both nodes must receive small and large payloads without discovery",
    );
    if let Some(run) = run {
        let finished = wait_until(|| deployment.children[run].try_wait().unwrap().is_some());
        deployment.check(finished, "local run must finish after stop-after");
        let status = deployment.children[run].wait().unwrap();
        deployment.check(status.success(), "local run must exit successfully");
    }
    if restart {
        deployment.children[second].kill().unwrap();
        deployment.children[second].wait().unwrap();
        std::fs::remove_file(dir.join("second.ready")).unwrap();
        std::fs::remove_file(dir.join("second.received")).unwrap();
        deployment.spawn(dynamic("second"), &log("second-restarted"));
        deployment.check(
            wait_until(|| has_both_sizes(&dir, "second")),
            "restarted dynamic consumer must reconnect",
        );
    }
}

#[test]
fn late_dynamic_peer_connects_to_static_neighbours_and_reconnects() {
    dynamic_peering_case(false, true, false);
}

#[test]
fn simultaneous_dynamic_peers_connect_without_discovery() {
    dynamic_peering_case(true, false, false);
}

#[test]
fn local_run_dynamic_peer_connects_without_discovery() {
    dynamic_peering_case(false, false, true);
}
