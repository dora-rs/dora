use crate::{
    socket_stream_utils::{socket_stream_receive, socket_stream_send},
    DaemonCoordinatorEvent,
};
use dora_core::uhlc::HLC;
use dora_message::{
    common::Timestamped,
    coordinator_to_daemon::RegisterResult,
    daemon_to_coordinator::{CoordinatorRequest, DaemonCoordinatorReply, DaemonRegisterRequest},
};
use eyre::{eyre, Context};
use std::{io::ErrorKind, net::SocketAddr, time::Duration};
use tokio::{
    net::TcpStream,
    sync::{mpsc, oneshot},
    time::sleep,
};
use tokio_stream::{wrappers::ReceiverStream, Stream};
use tracing::warn;

const DAEMON_COORDINATOR_RETRY_INTERVAL: std::time::Duration = Duration::from_secs(1);

#[derive(Debug)]
pub struct CoordinatorEvent {
    pub event: DaemonCoordinatorEvent,
    pub reply_tx: oneshot::Sender<Option<DaemonCoordinatorReply>>,
}

pub async fn register(
    addr: SocketAddr,
    machine_id: String,
    listen_port: u16,
    clock: &HLC,
) -> eyre::Result<impl Stream<Item = Timestamped<CoordinatorEvent>>> {
    let mut stream = loop {
        match TcpStream::connect(addr)
            .await
            .wrap_err("failed to connect to dora-coordinator")
        {
            Err(err) => {
                warn!("Could not connect to: {addr}, with error: {err}. Retring in {DAEMON_COORDINATOR_RETRY_INTERVAL:#?}..");
                sleep(DAEMON_COORDINATOR_RETRY_INTERVAL).await;
            }
            Ok(stream) => {
                break stream;
            }
        };
    };
    stream
        .set_nodelay(true)
        .wrap_err("failed to set TCP_NODELAY")?;
    let register = serde_json::to_vec(&Timestamped {
        inner: CoordinatorRequest::Register(DaemonRegisterRequest::new(machine_id, listen_port)),
        timestamp: clock.new_timestamp(),
    })?;
    socket_stream_send(&mut stream, &register)
        .await
        .wrap_err("failed to send register request to dora-coordinator")?;
    let reply_raw = socket_stream_receive(&mut stream)
        .await
        .wrap_err("failed to register reply from dora-coordinator")?;
    let result: Timestamped<RegisterResult> = serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize dora-coordinator reply")?;
    result.inner.to_result()?;
    if let Err(err) = clock.update_with_timestamp(&result.timestamp) {
        tracing::warn!("failed to update timestamp after register: {err}");
    }

    tracing::info!("Connected to dora-coordinator at {:?}", addr);

    let (tx, rx) = mpsc::channel(1);
    tokio::spawn(async move {
        loop {
            let event = match socket_stream_receive(&mut stream).await {
                Ok(raw) => match serde_json::from_slice(&raw) {
                    Ok(event) => event,
                    Err(err) => {
                        let err =
                            eyre!(err).wrap_err("failed to deserialize incoming coordinator event");
                        tracing::warn!("{err:?}");
                        continue;
                    }
                },
                Err(err) if err.kind() == ErrorKind::UnexpectedEof => break,
                Err(err) => {
                    let err = eyre!(err).wrap_err("failed to receive incoming event");
                    tracing::warn!("{err:?}");
                    continue;
                }
            };
            let Timestamped {
                inner: event,
                timestamp,
            } = event;
            let (reply_tx, reply_rx) = oneshot::channel();
            match tx
                .send(Timestamped {
                    inner: CoordinatorEvent { event, reply_tx },
                    timestamp,
                })
                .await
            {
                Ok(()) => {}
                Err(_) => {
                    // receiving end of channel was closed
                    break;
                }
            }

            let Ok(reply) = reply_rx.await else {
                tracing::warn!("daemon sent no reply");
                continue;
            };
            if let Some(reply) = reply {
                let serialized = match serde_json::to_vec(&reply)
                    .wrap_err("failed to serialize DaemonCoordinatorReply")
                {
                    Ok(r) => r,
                    Err(err) => {
                        tracing::error!("{err:?}");
                        continue;
                    }
                };
                if let Err(err) = socket_stream_send(&mut stream, &serialized).await {
                    tracing::warn!("failed to send reply to coordinator: {err}");
                    continue;
                };
                if let DaemonCoordinatorReply::DestroyResult { notify, .. } = reply {
                    if let Some(notify) = notify {
                        let _ = notify.send(());
                    }
                    break;
                }
            }
        }
    });

    Ok(ReceiverStream::new(rx))
}
