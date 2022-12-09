use crate::{
    tcp_utils::{tcp_receive, tcp_send},
    DaemonCoordinatorEvent,
};
use dora_core::coordinator_messages::{CoordinatorRequest, RegisterResult};
use eyre::{eyre, Context};
use std::{io::ErrorKind, net::SocketAddr};
use tokio::{net::TcpStream, sync::mpsc};
use tokio_stream::{wrappers::ReceiverStream, Stream};

pub async fn connect(addr: SocketAddr) -> eyre::Result<impl Stream<Item = DaemonCoordinatorEvent>> {
    let mut stream = TcpStream::connect(addr)
        .await
        .wrap_err("failed to connect to dora-coordinator")?;
    let register = serde_json::to_vec(&CoordinatorRequest::Register {
        machine_id: String::new(), // TODO
    })?;
    tcp_send(&mut stream, &register)
        .await
        .wrap_err("failed to send register request to dora-coordinator")?;
    let reply_raw = tcp_receive(&mut stream)
        .await
        .wrap_err("failed to register reply from dora-coordinator")?;
    let result: RegisterResult = serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize dora-coordinator reply")?;
    result.to_result()?;
    tracing::info!("Connected to dora-coordinator at {:?}", addr);

    let (tx, rx) = mpsc::channel(1);
    tokio::spawn(async move {
        loop {
            let event = match tcp_receive(&mut stream).await {
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
            match tx.send(event).await {
                Ok(()) => {}
                Err(_) => {
                    // receiving end of channel was closed
                    break;
                }
            }
        }
    });

    Ok(ReceiverStream::new(rx))
}
