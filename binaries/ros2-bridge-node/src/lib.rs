use std::path::PathBuf;

use dora_message::descriptor::{RmwZenohCompatibility, Ros2BridgeConfig, Ros2TransportConfig};

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SelectedTransport {
    Dds,
    Zenoh {
        compatibility: RmwZenohCompatibility,
        config_uri: Option<PathBuf>,
    },
}

pub trait ContextFactory {
    type Context;
    type Error: std::fmt::Display;
    fn dds(&self) -> Result<Self::Context, Self::Error>;
    fn zenoh(
        &self,
        compatibility: RmwZenohCompatibility,
        config_uri: Option<PathBuf>,
    ) -> Result<Self::Context, Self::Error>;
}

pub fn select_transport<F: ContextFactory>(
    config: &Ros2BridgeConfig,
    factory: &F,
    _rmw_implementation: Option<&str>,
) -> Result<F::Context, String> {
    match &config.transport {
        Ros2TransportConfig::Dds => factory.dds().map_err(|error| error.to_string()),
        Ros2TransportConfig::Zenoh {
            compatibility,
            config_uri,
        } => {
            if config_uri
                .as_ref()
                .is_some_and(|uri| uri.as_os_str().is_empty())
            {
                return Err("ros2 Zenoh config_uri must not be empty".into());
            }
            factory
                .zenoh(*compatibility, config_uri.clone())
                .map_err(|error| error.to_string())
        }
    }
}

pub fn dds_rmw_mismatch_warning(
    selected: &SelectedTransport,
    rmw_implementation: Option<&str>,
) -> Option<&'static str> {
    matches!((selected, rmw_implementation), (SelectedTransport::Dds, Some("rmw_zenoh_cpp")))
        .then_some("DDS transport selected while RMW_IMPLEMENTATION=rmw_zenoh_cpp; set transport.kind: zenoh explicitly")
}
