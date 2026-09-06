//! Real-runtime regression for required events lost behind a timer flood.
//! Run with an installed matching CLI:
//! cargo test -p dora-node-api --test backpressure_timer slow_consumer -- --ignored --nocapture

use dora_node_api::{DoraNode, Event, IntoArrow};
use std::{
    fs,
    process::Command,
    thread,
    time::{Duration, Instant},
};

#[test]
#[ignore = "requires the matching dora CLI on PATH"]
fn slow_consumer_keeps_backpressure_commit() -> eyre::Result<()> {
    let executable = std::env::current_exe()?;
    for with_timer in [false, true] {
        let root = std::env::temp_dir().join(format!("dora-pressure-{}", uuid::Uuid::now_v7()));
        fs::create_dir_all(&root)?;
        let result = root.join("received.txt");
        let mut inputs = serde_json::json!({
            "commit": {"source": "sender/commit", "queue_size": 4, "queue_policy": "backpressure"}
        });
        if with_timer {
            inputs["tick"] = serde_json::json!("dora/timer/millis/10");
        }
        let graph = serde_json::json!({"nodes": [
            {"id": "receiver", "path": executable, "args": "--ignored --exact receiver_node --nocapture",
             "env": {"DORA_PRESSURE_RESULT": result}, "inputs": inputs},
            {"id": "sender", "path": executable, "args": "--ignored --exact sender_node --nocapture", "outputs": ["commit"]}
        ]});
        let graph_path = root.join("graph.yaml");
        fs::write(&graph_path, serde_yaml::to_string(&graph)?)?;
        let cli = std::env::var_os("DORA_BACKPRESSURE_TEST_CLI").unwrap_or_else(|| "dora".into());
        let output = Command::new(cli)
            .args(["run", "--stop-after", "8s"])
            .arg(&graph_path)
            .current_dir(&root)
            .output()?;
        fs::write(root.join("stdout.log"), &output.stdout)?;
        fs::write(root.join("stderr.log"), &output.stderr)?;
        assert!(
            output.status.success(),
            "runtime failed; evidence: {}",
            root.display()
        );
        assert_eq!(
            fs::read_to_string(&result)?,
            "received",
            "with_timer={with_timer}; evidence: {}",
            root.display()
        );
    }
    Ok(())
}

#[test]
#[ignore = "helper process launched as a dataflow node"]
fn sender_node() -> eyre::Result<()> {
    let (mut node, _events) = DoraNode::init_from_env()?;
    thread::sleep(Duration::from_millis(1500));
    node.send_output(
        "commit".into(),
        Default::default(),
        vec![1_u64].into_arrow(),
    )?;
    thread::sleep(Duration::from_secs(4));
    Ok(())
}

#[test]
#[ignore = "helper process launched as a dataflow node"]
fn receiver_node() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;
    let started = Instant::now();
    thread::sleep(Duration::from_secs(3));
    let mut received = false;
    while let Some(event) = events.recv() {
        if matches!(event, Event::Input { ref id, .. } if id.as_str() == "commit") {
            received = true;
            break;
        }
        if started.elapsed() > Duration::from_secs(5) {
            break;
        }
    }
    fs::write(
        std::env::var("DORA_PRESSURE_RESULT")?,
        if received { "received" } else { "lost" },
    )?;
    Ok(())
}
