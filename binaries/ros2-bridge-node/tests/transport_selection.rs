use std::{cell::RefCell, path::PathBuf};

use dora_message::descriptor::{RmwZenohCompatibility, Ros2BridgeConfig};
use dora_ros2_bridge_node::{ContextFactory, SelectedTransport, select_transport};

#[derive(Default)]
struct FakeFactory {
    calls: RefCell<Vec<SelectedTransport>>,
}
impl ContextFactory for FakeFactory {
    type Context = ();
    type Error = &'static str;
    fn dds(&self) -> Result<Self::Context, Self::Error> {
        self.calls.borrow_mut().push(SelectedTransport::Dds);
        Ok(())
    }
    fn zenoh(
        &self,
        compatibility: RmwZenohCompatibility,
        config_uri: Option<PathBuf>,
    ) -> Result<Self::Context, Self::Error> {
        self.calls.borrow_mut().push(SelectedTransport::Zenoh {
            compatibility,
            config_uri,
        });
        Ok(())
    }
}

fn parse(yaml: &str) -> Ros2BridgeConfig {
    serde_yaml::from_str(yaml).unwrap()
}

#[test]
fn omitted_transport_selects_dds_even_when_rmw_environment_says_zenoh() {
    let factory = FakeFactory::default();
    let config = parse("topic: /chatter\nmessage_type: std_msgs/String\n");
    select_transport(&config, &factory, Some("rmw_zenoh_cpp")).unwrap();
    assert_eq!(*factory.calls.borrow(), vec![SelectedTransport::Dds]);
}

#[test]
fn explicit_zenoh_passes_profile_and_config_uri() {
    let factory = FakeFactory::default();
    let config = parse(
        "transport:\n  kind: zenoh\n  compatibility: humble\n  config_uri: /tmp/rmw.json5\ntopic: /chatter\nmessage_type: std_msgs/String\n",
    );
    select_transport(&config, &factory, None).unwrap();
    assert_eq!(
        *factory.calls.borrow(),
        vec![SelectedTransport::Zenoh {
            compatibility: RmwZenohCompatibility::Humble,
            config_uri: Some("/tmp/rmw.json5".into())
        }]
    );
}

#[test]
fn invalid_zenoh_config_fails_before_factory_initialization() {
    let factory = FakeFactory::default();
    let config = parse(
        "transport:\n  kind: zenoh\n  compatibility: humble\n  config_uri: ''\ntopic: /chatter\nmessage_type: std_msgs/String\n",
    );
    assert!(select_transport(&config, &factory, None).is_err());
    assert!(factory.calls.borrow().is_empty());
}
