//! The DDS parameter client against a node whose parameters ros2-client
//! hosts (#4297). Pure RustDDS: needs no ROS installation.

use std::time::Duration;

use dora_ros2_bridge::{
    ros2_client::{
        Context, ContextOptions, NodeName, NodeOptions, ParameterValue, ServiceMapping,
        parameters::raw,
    },
    transport::parameters::{DdsParameterClient, ParameterChange, ParameterClient},
};

/// A domain no other test in this crate uses, so parallel tests don't match.
const DOMAIN_ID: u16 = 90;
const TIMEOUT: Duration = Duration::from_secs(10);

fn context() -> Context {
    Context::with_options(ContextOptions::new().domain_id(DOMAIN_ID)).expect("DDS context")
}

/// Runs the node's spinner, which answers its parameter services and
/// tracks matched endpoints for `wait_for_service`.
fn spin(node: &mut dora_ros2_bridge::ros2_client::Node) {
    let spinner = node.spinner().expect("spinner");
    std::thread::spawn(move || futures::executor::block_on(spinner.spin()));
}

#[test]
fn a_parameter_client_reaches_a_ros2_client_node() {
    // One participant per node, as with separate processes.
    let mut host = context()
        .new_node(
            NodeName::new("/robot", "arm").unwrap(),
            NodeOptions::new()
                .declare_parameter("speed", ParameterValue::Double(1.5))
                .declare_parameter("gains", ParameterValue::DoubleArray(vec![1.0, 2.0]))
                .parameter_validator(Box::new(|name, value| {
                    if name == "speed" && !matches!(value, ParameterValue::Double(_)) {
                        Err("speed must stay a double".into())
                    } else {
                        Ok(())
                    }
                })),
        )
        .expect("host node");
    spin(&mut host);

    let mut tuner = context()
        .new_node(
            NodeName::new("/tools", "tuner").unwrap(),
            NodeOptions::new(),
        )
        .expect("client node");
    // ros2-client always serves parameters with the Enhanced mapping (#4215).
    let dds = DdsParameterClient::with_service_mapping(
        &mut tuner,
        "/robot/arm",
        ServiceMapping::Enhanced,
    )
    .expect("parameter client");
    spin(&mut tuner);
    futures::executor::block_on(dds.wait_for_services(&tuner, TIMEOUT)).expect("services");
    let client = ParameterClient::Dds(Box::new(dds));

    futures::executor::block_on(async {
        let names = |names: &[&str]| names.iter().map(|n| n.to_string()).collect::<Vec<_>>();
        assert_eq!(
            client
                .get(&names(&["speed", "missing"]), TIMEOUT)
                .await
                .unwrap(),
            [Some(ParameterValue::Double(1.5)), None]
        );
        assert_eq!(
            client
                .get_types(&names(&["speed", "missing"]), TIMEOUT)
                .await
                .unwrap(),
            [raw::ParameterType::DOUBLE, raw::ParameterType::NOT_SET]
        );
        let mut listed = client.list(vec![], 0, TIMEOUT).await.unwrap().names;
        listed.sort();
        assert_eq!(listed, ["gains", "speed", "use_sim_time"]);
        let described = client.describe(&names(&["gains"]), TIMEOUT).await.unwrap();
        assert_eq!(
            described[0].as_ref().unwrap().r#type,
            raw::ParameterType::DOUBLE_ARRAY
        );

        let results = client
            .set(
                vec![
                    ("speed".into(), ParameterValue::Double(2.0)),
                    ("speed".into(), ParameterValue::String("fast".into())),
                ],
                TIMEOUT,
            )
            .await
            .unwrap();
        assert!(results[0].is_ok(), "{results:?}");
        assert!(results[1].is_err(), "{results:?}");
        assert_eq!(
            client.get(&names(&["speed"]), TIMEOUT).await.unwrap(),
            [Some(ParameterValue::Double(2.0))]
        );
        let change = futures::future::select(
            Box::pin(client.next_change()),
            futures_timer::Delay::new(TIMEOUT),
        )
        .await;
        let futures::future::Either::Left((change, _)) = change else {
            panic!("no parameter event within {TIMEOUT:?}");
        };
        assert_eq!(
            change.unwrap(),
            ParameterChange {
                new: vec![],
                changed: vec![("speed".into(), ParameterValue::Double(2.0))],
                deleted: vec![],
            }
        );
    });
}
