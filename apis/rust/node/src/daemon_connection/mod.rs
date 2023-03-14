use communication::DaemonChannel;
use dora_core::{
    config::NodeId,
    daemon_messages::{DaemonCommunication, DataflowId, DropToken},
};
use eyre::Context;
use flume::RecvTimeoutError;
use std::{net::TcpStream, sync::Arc, time::Duration};

pub(crate) use control_channel::ControlChannel;
pub use event_stream::EventStream;

mod communication;
mod control_channel;
mod event_stream;

pub(crate) struct DaemonConnection {
    pub control_channel: ControlChannel,
    pub event_stream: EventStream,
    pub finished_drop_tokens: flume::Receiver<DropToken>,
}

impl DaemonConnection {
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_communication: &DaemonCommunication,
    ) -> eyre::Result<Self> {
        let (control, events) = match daemon_communication {
            DaemonCommunication::Shmem {
                daemon_control_region_id,
                daemon_events_region_id,
            } => {
                let control = unsafe { DaemonChannel::new_shmem(daemon_control_region_id) }
                    .wrap_err("failed to create shmem control channel")?;
                let events = unsafe { DaemonChannel::new_shmem(daemon_events_region_id) }
                    .wrap_err("failed to create shmem event channel")?;
                (control, events)
            }
            DaemonCommunication::Tcp { socket_addr } => {
                let control = DaemonChannel::new_tcp(
                    TcpStream::connect(socket_addr).wrap_err("failed to connect control stream")?,
                )?;
                let events = DaemonChannel::new_tcp(
                    TcpStream::connect(socket_addr).wrap_err("failed to connect event stream")?,
                )?;
                (control, events)
            }
        };

        let (event_stream, event_stream_thread_handle, finished_drop_tokens) =
            EventStream::init(dataflow_id, node_id, events)
                .wrap_err("failed to init event stream")?;
        let control_channel =
            ControlChannel::init(dataflow_id, node_id, control, event_stream_thread_handle)
                .wrap_err("failed to init control stream")?;

        Ok(Self {
            control_channel,
            event_stream,
            finished_drop_tokens,
        })
    }
}

pub(crate) struct EventStreamThreadHandle(flume::Receiver<std::thread::Result<()>>);
impl EventStreamThreadHandle {
    fn new(join_handle: std::thread::JoinHandle<()>) -> Arc<Self> {
        let (tx, rx) = flume::bounded(1);
        std::thread::spawn(move || {
            let _ = tx.send(join_handle.join());
        });
        Arc::new(Self(rx))
    }
}

impl Drop for EventStreamThreadHandle {
    fn drop(&mut self) {
        match self.0.recv_timeout(Duration::from_secs(2)) {
            Ok(Ok(())) => {}
            Ok(Err(_)) => {
                tracing::error!("event stream thread panicked");
            }
            Err(RecvTimeoutError::Timeout) => {
                tracing::warn!("timeout while waiting for event stream thread");
            }
            Err(RecvTimeoutError::Disconnected) => {
                tracing::warn!("event stream thread result channel closed unexpectedly");
            }
        }
    }
}
