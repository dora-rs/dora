use dora_core::topics::{ZenohSessionParams, open_zenoh_session, open_zenoh_session_with_listen};
use dora_message::dynamic_node::DynamicNodePeering;

pub(super) async fn open_session(
    peering: Option<&DynamicNodePeering>,
) -> eyre::Result<zenoh::Session> {
    let Some(peering) = peering else {
        // Static nodes use the spawn environment; old daemons supply no
        // dynamic bootstrap information and retain their discovery behaviour.
        return open_zenoh_session(None).await;
    };
    if std::env::var_os(zenoh::Config::DEFAULT_CONFIG_PATH_ENV).is_some() {
        // Preserve the explicit full-configuration override, just as for
        // static nodes. An overlay is the way to retain automatic peering.
        tracing::warn!(
            "ZENOH_CONFIG overrides dynamic node peer connections; use DORA_ZENOH_CONFIG_OVERLAY to retain automatic peering"
        );
        return open_zenoh_session(None).await;
    }
    let (session, bound) = open_zenoh_session_with_listen(ZenohSessionParams {
        listen_endpoint: Some(&peering.listen),
        connect_endpoints: &peering.connect,
        ..Default::default()
    })
    .await?;
    if bound.as_deref() != Some(peering.listen.as_str()) {
        eyre::bail!(
            "dynamic node zenoh listener `{}` did not bind (port taken since the daemon reserved it); restart the node to get a new one",
            peering.listen,
        );
    }
    Ok(session)
}
