//! Regression test for the QoS the bridge advertises on its DDS service and
//! action endpoints: all reliable, action status additionally transient-local.
//!
//! A call site in `run_service_mode` / `run_action_mode` that rebuilds its QoS
//! from the generic `config.qos` (best-effort, volatile) degrades these
//! endpoints silently, so the test launches the real binary per role and reads
//! the advertised policies from a separate DDS participant. No ROS install is
//! needed: `AMENT_PREFIX_PATH` is empty and the only input event is a delayed
//! `Stop`, so nothing is serialized.
//!
//! The observer joins the same non-zero domain the child is given through
//! `ROS_DOMAIN_ID`, so discovering the child's endpoints also proves that the
//! variable reaches the node the bridge actually creates.

use std::{
    fs::File,
    path::PathBuf,
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};

use dora_message::descriptor::{Ros2BridgeConfig, Ros2Role};
use dora_ros2_bridge::rustdds::{DomainParticipant, policy};

/// Isolated DDS domain so the test never sees a real ROS 2 graph.
const DOMAIN_ID: &str = "181";
/// How long the child stays alive before its `Stop` event fires.
const STOP_DELAY_SECS: f64 = 60.0;
/// Upper bound for endpoint discovery.
const DISCOVERY_TIMEOUT: Duration = Duration::from_secs(15);

/// `pid` plus nanoseconds, so no earlier run can satisfy this run's assertions.
fn unique_suffix() -> String {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock before unix epoch")
        .as_nanos();
    format!("{}_{nanos}", std::process::id())
}

/// Owns the temporary directory of one child run and removes it on drop.
struct TempDir(PathBuf);

impl TempDir {
    fn new(label: &str) -> Self {
        let path = std::env::temp_dir().join(format!("dora-ros2-qos-{label}-{}", unique_suffix()));
        std::fs::create_dir_all(&path).expect("create temp dir");
        Self(path)
    }

    fn join(&self, name: &str) -> PathBuf {
        self.0.join(name)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Kills and reaps the bridge child on drop, including on panic.
struct ChildGuard {
    child: Child,
    log: PathBuf,
}

impl ChildGuard {
    fn spawn(config: &Ros2BridgeConfig, dir: &TempDir) -> Self {
        // `Stop` is the serialized spelling of `IncomingEvent::Stop` (the enum
        // has no `rename_all`), so the child stays alive for the whole run.
        let inputs = dir.join("inputs.json");
        let stop_event = serde_json::json!({
            "id": "qos_probe",
            "events": [{ "time_offset_secs": STOP_DELAY_SECS, "type": "Stop" }],
        });
        std::fs::write(&inputs, stop_event.to_string()).expect("write integration test inputs");

        // Log to a file rather than a pipe: the node logs continuously and an
        // unread pipe would eventually block it.
        let log = dir.join("child.log");
        let log_file = File::create(&log).expect("create child log");
        let stderr = log_file.try_clone().expect("clone child log");

        let child = Command::new(env!("CARGO_BIN_EXE_dora-ros2-bridge-node"))
            .env(
                "DORA_ROS2_BRIDGE_CONFIG",
                serde_json::to_string(config).expect("serialize bridge config"),
            )
            .env("DORA_TEST_WITH_INPUTS", &inputs)
            .env("DORA_TEST_WRITE_OUTPUTS_TO", dir.join("outputs.jsonl"))
            .env("ROS_DOMAIN_ID", DOMAIN_ID)
            .env("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
            .env("AMENT_PREFIX_PATH", "")
            .env_remove("DORA_NODE_CONFIG")
            .env_remove("ROS_DISCOVERY_SERVER")
            .env_remove("ROS_STATIC_PEERS")
            .stdin(Stdio::null())
            .stdout(Stdio::from(log_file))
            .stderr(Stdio::from(stderr))
            .spawn()
            .expect("spawn dora-ros2-bridge-node");
        Self { child, log }
    }

    /// Fails with the child's log when it exited before discovery finished.
    fn assert_running(&mut self, label: &str) {
        if let Some(status) = self.child.try_wait().expect("poll bridge child") {
            panic!(
                "bridge child for {label} exited early with {status}:\n{}",
                self.log_contents()
            );
        }
    }

    fn log_contents(&self) -> String {
        std::fs::read_to_string(&self.log).unwrap_or_else(|e| format!("<unreadable log: {e}>"))
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// One endpoint the bridge is expected to advertise.
#[derive(Clone)]
struct Expected {
    topic: String,
    /// `true` for a topic the bridge publishes, `false` for one it subscribes to.
    published: bool,
    transient_local: bool,
}

fn endpoint(topic: String, published: bool) -> Expected {
    Expected {
        topic,
        published,
        transient_local: false,
    }
}

/// Service endpoints as seen by the **server**: it reads requests and writes
/// replies. The client advertises the mirror image.
fn service_endpoints(name: &str) -> Vec<Expected> {
    vec![
        endpoint(format!("rq/{name}Request"), false),
        endpoint(format!("rr/{name}Reply"), true),
    ]
}

/// Action endpoints as seen by the **server**: it reads the three goal
/// services' requests and writes their replies, feedback and status.
fn action_endpoints(name: &str) -> Vec<Expected> {
    let mut endpoints = Vec::new();
    for service in ["send_goal", "get_result", "cancel_goal"] {
        endpoints.push(endpoint(
            format!("rq/{name}/_action/{service}Request"),
            false,
        ));
        endpoints.push(endpoint(format!("rr/{name}/_action/{service}Reply"), true));
    }
    endpoints.push(endpoint(format!("rt/{name}/_action/feedback"), true));
    endpoints.push(Expected {
        topic: format!("rt/{name}/_action/status"),
        published: true,
        transient_local: true,
    });
    endpoints
}

/// The peer role advertises the mirror image of the same topics.
fn mirrored(endpoints: Vec<Expected>) -> Vec<Expected> {
    endpoints
        .into_iter()
        .map(|endpoint| Expected {
            published: !endpoint.published,
            ..endpoint
        })
        .collect()
}

fn config(service: Option<&str>, action: Option<&str>, role: Ros2Role) -> Ros2BridgeConfig {
    let type_name = |name: Option<&str>| name.map(|_| "test_interfaces/Test".to_string());
    Ros2BridgeConfig {
        service: service.map(str::to_string),
        service_type: type_name(service),
        action: action.map(str::to_string),
        action_type: type_name(action),
        role: Some(role),
        node_name: Some("qos_probe".to_string()),
        ..Default::default()
    }
}

/// Runs one role and asserts every expected endpoint is advertised with
/// reliable delivery and the expected durability.
fn assert_role(label: &str, config: &Ros2BridgeConfig, expected: &[Expected]) {
    let dir = TempDir::new(label);
    let mut child = ChildGuard::spawn(config, &dir);
    let participant = DomainParticipant::new(DOMAIN_ID.parse().expect("domain id"))
        .expect("create observer participant");

    let deadline = Instant::now() + DISCOVERY_TIMEOUT;
    let mut missing: Vec<String> = Vec::new();
    loop {
        let readers = participant.discovered_readers();
        let writers = participant.discovered_writers();

        missing.clear();
        for endpoint in expected {
            let direction = if endpoint.published {
                "writer"
            } else {
                "reader"
            };
            let qos = if endpoint.published {
                writers
                    .iter()
                    .find(|data| data.publication_topic_data.topic_name() == &endpoint.topic)
                    .map(|data| data.publication_topic_data.qos())
            } else {
                readers
                    .iter()
                    .find(|data| data.subscription_topic_data.topic_name() == &endpoint.topic)
                    .map(|data| data.subscription_topic_data.qos())
            };
            let Some(qos) = qos else {
                missing.push(format!("{direction} {}", endpoint.topic));
                continue;
            };

            let durability = if endpoint.transient_local {
                policy::Durability::TransientLocal
            } else {
                policy::Durability::Volatile
            };
            assert!(
                matches!(
                    qos.reliability(),
                    Some(policy::Reliability::Reliable { .. })
                ),
                "{label}: {direction} {} must be reliable, got {:?}",
                endpoint.topic,
                qos.reliability()
            );
            assert_eq!(
                qos.durability(),
                Some(durability),
                "{label}: {direction} {} has the wrong durability",
                endpoint.topic
            );
        }

        if missing.is_empty() {
            break;
        }
        child.assert_running(label);
        if Instant::now() >= deadline {
            panic!(
                "{label}: endpoints not discovered within {DISCOVERY_TIMEOUT:?}: {missing:?}\n{}",
                child.log_contents()
            );
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    // The child must still be alive: the endpoints were advertised by the real
    // service/action runtime, not by a node that already shut down.
    child.assert_running(label);
}

/// Detects a call site in `run_service_mode` / `run_action_mode` that rebuilds
/// its QoS from the generic `config.qos`: the default config is best-effort and
/// volatile, so the regression shows up as a degraded endpoint here.
#[test]
fn service_and_action_endpoints_are_reliable_with_transient_local_status() {
    let suffix = unique_suffix();
    let service = format!("qos_probe_service_{suffix}");
    let action = format!("qos_probe_action_{suffix}");

    let service_topics = service_endpoints(&service);
    assert_role(
        "service-client",
        &config(Some(&service), None, Ros2Role::Client),
        &mirrored(service_topics.clone()),
    );
    assert_role(
        "service-server",
        &config(Some(&service), None, Ros2Role::Server),
        &service_topics,
    );

    let action_topics = action_endpoints(&action);
    assert_role(
        "action-client",
        &config(None, Some(&action), Ros2Role::Client),
        &mirrored(action_topics.clone()),
    );
    assert_role(
        "action-server",
        &config(None, Some(&action), Ros2Role::Server),
        &action_topics,
    );
}
