use super::DaemonChannel;
use dora_core::{
    config::NodeId,
    daemon_messages::{DaemonReply, DaemonRequest, DataflowId, DropToken, NodeDropEvent},
};
use eyre::{eyre, Context};

pub(crate) fn init(
    dataflow_id: DataflowId,
    node_id: &NodeId,
    mut channel: DaemonChannel,
) -> eyre::Result<flume::Receiver<DropToken>> {
    channel.register(dataflow_id, node_id.clone())?;

    channel
        .request(&DaemonRequest::SubscribeDrop)
        .map_err(|e| eyre!(e))
        .wrap_err("failed to create subscription with dora-daemon")?;

    let (tx, rx) = flume::bounded(0);
    let node_id_cloned = node_id.clone();

    std::thread::spawn(|| drop_stream_loop(node_id_cloned, tx, channel));

    Ok(rx)
}

#[tracing::instrument(skip(tx, channel))]
fn drop_stream_loop(node_id: NodeId, tx: flume::Sender<DropToken>, mut channel: DaemonChannel) {
    'outer: loop {
        let daemon_request = DaemonRequest::NextFinishedDropTokens;
        let events = match channel.request(&daemon_request) {
            Ok(DaemonReply::NextDropEvents(events)) if events.is_empty() => {
                tracing::debug!("Drop stream closed for node ID `{node_id}`");
                break;
            }
            Ok(DaemonReply::NextDropEvents(events)) => events,
            Ok(other) => {
                let err = eyre!("unexpected drop reply: {other:?}");
                tracing::warn!("{err:?}");
                continue;
            }
            Err(err) => {
                let err = eyre!(err).wrap_err("failed to receive incoming drop event");
                tracing::warn!("{err:?}");
                continue;
            }
        };
        for event in events {
            match event {
                NodeDropEvent::OutputDropped { drop_token } => {
                    if tx.send(drop_token).is_err() {
                        tracing::warn!(
                            "drop channel was closed already, could not forward \
                            drop token`{drop_token:?}`"
                        );
                        break 'outer;
                    }
                }
            }
        }
    }
}
