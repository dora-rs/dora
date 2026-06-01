//! dora node exposing ROS2 parameters via the bridge.
//!
//! Declares parameters on the ROS2 node `/demo_params` and exercises the
//! parameter API. The six rcl_interfaces parameter services are hosted natively
//! by ros2-client (driven by the node spinner), so `ros2 param` / rclpy clients
//! can query them over DDS (subject to ros2-client#4 discoverability); this node
//! uses the local get/set/list API, which needs no discovery and is therefore
//! deterministic on every platform. This mirrors the Python `parameter` example.

use std::{
    collections::HashMap,
    mem::{Discriminant, discriminant},
};

use dora_node_api::{self, DoraNode, Event};
use dora_ros2_bridge::ros2_client::{self, NodeOptions, ParameterValue};
use eyre::{Context, eyre};
use futures::task::SpawnExt;

fn main() -> eyre::Result<()> {
    // Connect to the dora daemon first (before the slower ROS2 setup), so the
    // daemon does not kill this node for not initializing its connection in time.
    let (_node, mut dora_events) = DoraNode::init_from_env()?;

    // Initial parameters to declare on the node, covering the ParameterValue
    // kinds exercised here (scalar int/float/str plus float and bool arrays).
    let declared: Vec<(&str, ParameterValue)> = vec![
        ("speed", ParameterValue::Double(1.5)),
        ("name", ParameterValue::String("robot".to_string())),
        ("gain", ParameterValue::Integer(7)),
        (
            "waypoints",
            ParameterValue::DoubleArray(vec![1.0, 2.0, 3.0]),
        ),
        ("flags", ParameterValue::BooleanArray(vec![true, false])),
    ];

    // Record each declared parameter's value type so the validator can enforce
    // type stability on *every* set path (the local `set_parameter` and any
    // remote `ros2 param set`).
    let declared_types: HashMap<String, Discriminant<ParameterValue>> = declared
        .iter()
        .map(|(name, value)| (name.to_string(), discriminant(value)))
        .collect();

    let mut node_options = NodeOptions::new().enable_rosout(true);
    for (name, value) in &declared {
        node_options = node_options.declare_parameter(name, value.clone());
    }
    // Type-stability validator: reject a set whose value type differs from the
    // declared one. ros2-client invokes this on both the local and the
    // service-served set paths.
    node_options =
        node_options.parameter_validator(Box::new(move |name: &str, value: &ParameterValue| {
            match declared_types.get(name) {
                Some(declared) if *declared != discriminant(value) => Err(format!(
                    "parameter `{name}` is type-stable; cannot change its declared type"
                )),
                _ => Ok(()),
            }
        }));

    let ros_context =
        ros2_client::Context::new().map_err(|e| eyre!("failed to create ROS2 context: {e:?}"))?;
    let mut ros_node = ros_context
        .new_node(
            ros2_client::NodeName::new("/", "demo_params")
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
            node_options,
        )
        .map_err(|e| eyre!("failed to create ros2 node: {e:?}"))?;

    // Background spinner: hosts the parameter services over DDS so an external
    // `ros2 param` / rclpy client could query them. The local API below does not
    // depend on it, but a "parameter server" example should keep it running.
    let pool = futures::executor::ThreadPool::new()?;
    let spinner = ros_node
        .spinner()
        .map_err(|e| eyre!("failed to create spinner: {e:?}"))?;
    pool.spawn(async move {
        if let Err(err) = spinner.spin().await {
            eprintln!("ros2 spinner failed: {err:?}");
        }
    })
    .context("failed to spawn ros2 spinner")?;

    // Local parameter API (no cross-node discovery needed). `ParameterValue`
    // does not implement `PartialEq`, so assert by matching the variant.
    println!("list_parameters: {:?}", ros_node.list_parameters());
    assert!(ros_node.has_parameter("speed"));
    assert!(matches!(ros_node.get_parameter("speed"), Some(ParameterValue::Double(v)) if v == 1.5));
    assert!(
        matches!(ros_node.get_parameter("name"), Some(ParameterValue::String(s)) if s == "robot")
    );
    assert!(matches!(
        ros_node.get_parameter("gain"),
        Some(ParameterValue::Integer(7))
    ));
    assert!(
        matches!(ros_node.get_parameter("waypoints"), Some(ParameterValue::DoubleArray(v)) if v == [1.0, 2.0, 3.0])
    );
    assert!(
        matches!(ros_node.get_parameter("flags"), Some(ParameterValue::BooleanArray(v)) if v == [true, false])
    );
    assert!(ros_node.get_parameter("missing").is_none());

    // Update a parameter (same type) and read it back.
    ros_node
        .set_parameter("speed", ParameterValue::Double(2.0))
        .map_err(|e| eyre!("failed to set `speed`: {e}"))?;
    assert!(matches!(ros_node.get_parameter("speed"), Some(ParameterValue::Double(v)) if v == 2.0));

    // Changing a declared parameter's type is rejected (type-stable contract,
    // enforced by the validator on every set path).
    assert!(
        ros_node
            .set_parameter("gain", ParameterValue::String("not an int".to_string()))
            .is_err(),
        "expected a type-change rejection for `gain`"
    );

    println!("PARAM API OK");

    // Keep the node (and its parameter services) alive briefly so an external
    // ROS2 client could query it; the asserts above are the smoke contract.
    for _ in 0..50 {
        match dora_events.recv() {
            Some(Event::Stop(_)) | None => break,
            _ => {}
        }
    }

    Ok(())
}
