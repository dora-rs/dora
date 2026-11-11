//! Regression test for the QoS that the bridge node actually advertises on its
//! DDS service and action endpoints.
//!
//! ROS 2 service and action endpoints must be reliable, and the action status
//! channel must additionally be transient-local. The bridge derives those
//! policies in `bridge_endpoint_qos` / `action_status_qos`; if a call site in
//! `run_service_mode` / `run_action_mode` ever falls back to the generic
//! `config.qos.to_rustdds_qos()` (best-effort, volatile by default), the
//! endpoints below silently degrade and ROS 2 peers stop interoperating.
//!
//! The test therefore launches the real binary once per role and observes the
//! advertised endpoint policies from a separate DDS participant, instead of
//! asserting on helper return values or source text.
//!
//! No ROS installation is required: `AMENT_PREFIX_PATH` is empty, so the node
//! loads no message definitions. Nothing is serialized because the only input
//! event is a delayed `Stop`, which keeps the service/action runtime alive
//! while the endpoints are discovered.

use std::{
    path::PathBuf,
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};

use dora_ros2_bridge::rustdds::{self, policy, DomainParticipant};

/// Isolated DDS domain so the test never sees a real ROS 2 graph.
const DOMAIN_ID: &str = "181";
/// How long the child stays alive before its `Stop` event fires.
const STOP_DELAY_SECS: f64 = 60.0;
/// Upper bound for endpoint discovery.
const DISCOVERY_TIMEOUT: Duration = Duration::from_secs(15);

/// Owns the temporary directory of one child run and removes it on drop.
struct TempDir(PathBuf);

impl TempDir {
    fn new(label: &str) -> Self {
        let unique = format!(
            "dora-ros2-endpoint-qos-{label}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("system clock before unix epoch")
                .as_nanos()
        );
        let path = std::env::temp_dir().join(unique);
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
struct ChildGuard(Child);

impl ChildGuard {
    fn spawn(config: &str, dir: &TempDir) -> Self {
        let inputs = dir.join("inputs.json");
        std::fs::write(
            &inputs,
            format!(
                r#"{{"id":"qos_probe","events":[{{"time_offset_secs":{STOP_DELAY_SECS},"type":"Stop"}}]}}"#
            ),
        )
        .expect("write integration test inputs");

        let child = Command::new(env!("CARGO_BIN_EXE_dora-ros2-bridge-node"))
            .env("DORA_ROS2_BRIDGE_CONFIG", config)
            .env("DORA_TEST_WITH_INPUTS", &inputs)
            .env("DORA_TEST_WRITE_OUTPUTS_TO", dir.join("outputs.jsonl"))
            .env("ROS_DOMAIN_ID", DOMAIN_ID)
            .env("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
            .env("AMENT_PREFIX_PATH", "")
            .env_remove("DORA_NODE_CONFIG")
            .env_remove("ROS_DISCOVERY_SERVER")
            .env_remove("ROS_STATIC_PEERS")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()
            .expect("spawn dora-ros2-bridge-node");
        Self(child)
    }

    /// Fails with the child's stderr when it exited before discovery finished.
    fn assert_running(&mut self, role: &str) {
        if let Some(status) = self.0.try_wait().expect("poll bridge child") {
            let mut stderr = String::new();
            if let Some(mut pipe) = self.0.stderr.take() {
                use std::io::Read;
                let _ = pipe.read_to_string(&mut stderr);
            }
            panic!("bridge child for {role} exited early with {status}:\n{stderr}");
        }
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// One endpoint the bridge is expected to advertise.
struct Expected {
    topic: String,
    /// `true` for a topic the bridge publishes, `false` for one it subscribes to.
    published: bool,
    transient_local: bool,
}

fn service_endpoints(name: &str) -> Vec<Expected> {
    vec![
        Expected {
            topic: format!("rq/{name}Request"),
            published: false,
            transient_local: false,
        },
        Expected {
            topic: format!("rr/{name}Reply"),
            published: true,
            transient_local: false,
        },
    ]
}

fn action_endpoints(name: &str) -> Vec<Expected> {
    let mut endpoints = Vec::new();
    for service in ["send_goal", "get_result", "cancel_goal"] {
        endpoints.push(Expected {
            topic: format!("rq/{name}/_action/{service}Request"),
            published: false,
            transient_local: false,
        });
        endpoints.push(Expected {
            topic: format!("rr/{name}/_action/{service}Reply"),
            published: true,
            transient_local: false,
        });
    }
    endpoints.push(Expected {
        topic: format!("rt/{name}/_action/feedback"),
        published: true,
        transient_local: false,
    });
    endpoints.push(Expected {
        topic: format!("rt/{name}/_action/status"),
        published: true,
        transient_local: true,
    });
    endpoints
}

/// Flips the direction of every endpoint: the peer role advertises the mirror
/// image of the same topics.
fn mirrored(endpoints: Vec<Expected>) -> Vec<Expected> {
    endpoints
        .into_iter()
        .map(|endpoint| Expected {
            published: !endpoint.published,
            ..endpoint
        })
        .collect()
}

fn config(service: Option<&str>, action: Option<&str>, role: &str) -> String {
    let mut value = serde_json::json!({
        "role": role,
        "namespace": "/",
        "node_name": "qos_probe",
    });
    let object = value.as_object_mut().expect("json object");
    match (service, action) {
        (Some(name), None) => {
            object.insert("service".into(), name.into());
            object.insert("service_type".into(), "test_interfaces/Test".into());
        }
        (None, Some(name)) => {
            object.insert("action".into(), name.into());
            object.insert("action_type".into(), "test_interfaces/Test".into());
        }
        _ => panic!("exactly one of service/action must be set"),
    }
    value.to_string()
}

/// Runs one role and asserts every expected endpoint is advertised with
/// reliable delivery and the expected durability.
fn assert_role(label: &str, config: &str, expected: &[Expected]) {
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
            let found = if endpoint.published {
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

            let Some(qos) = found else {
                missing.push(format!(
                    "{} {}",
                    if endpoint.published {
                        "writer"
                    } else {
                        "reader"
                    },
                    endpoint.topic
                ));
                continue;
            };

            assert!(
                matches!(qos.reliability(), Some(policy::Reliability::Reliable { .. })),
                "{label}: {} {} must be reliable, got {:?}",
                if endpoint.published {
                    "writer"
                } else {
                    "reader"
                },
                endpoint.topic,
                qos.reliability()
            );

            let durability = qos.durability();
            if endpoint.transient_local {
                assert_eq!(
                    durability,
                    Some(policy::Durability::TransientLocal),
                    "{label}: {} {} must be transient-local",
                    if endpoint.published {
                        "writer"
                    } else {
                        "reader"
                    },
                    endpoint.topic
                );
            } else {
                assert_eq!(
                    durability,
                    Some(policy::Durability::Volatile),
                    "{label}: {} {} must stay volatile",
                    if endpoint.published {
                        "writer"
                    } else {
                        "reader"
                    },
                    endpoint.topic
                );
            }
        }

        if missing.is_empty() {
            break;
        }
        if Instant::now() >= deadline {
            child.assert_running(label);
            panic!("{label}: endpoints not discovered within {DISCOVERY_TIMEOUT:?}: {missing:?}");
        }
        child.assert_running(label);
        std::thread::sleep(Duration::from_millis(100));
    }

    // The child must still be alive: the endpoints were advertised by the real
    // service/action runtime, not by a node that already shut down.
    child.assert_running(label);
    drop(child);
    drop(dir);
}

/// The bridge's service and action endpoints must be reliable, and the action
/// status channel transient-local, in every role.
///
/// Detects a call site in `run_service_mode` / `run_action_mode` that rebuilds
/// its QoS from the generic `config.qos` instead of the service/action
/// selection: the default config is best-effort and volatile, so such a
/// regression shows up as a best-effort or volatile endpoint here.
#[test]
fn service_and_action_endpoints_are_reliable_with_transient_local_status() {
    let service = "qos_probe_service";
    let action = "qos_probe_action";

    let service_topics = service_endpoints(service);
    assert_role(
        "service-client",
        &config(Some(service), None, "client"),
        &service_topics,
    );
    assert_role(
        "service-server",
        &config(Some(service), None, "server"),
        &mirrored(service_topics),
    );

    let action_topics = action_endpoints(action);
    assert_role(
        "action-client",
        &config(None, Some(action), "client"),
        &mirrored(action_topics),
    );
    assert_role(
        "action-server",
        &config(None, Some(action), "server"),
        &action_topics,
    );
}
