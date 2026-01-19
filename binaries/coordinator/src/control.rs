use crate::{
    Coordinator, handler::CliRequestHandler, log_subscriber::LogSubscriber, send_log_message,
};
use communication_layer_request_reply::{AsyncTransport, transport::FramedTransport};
use dora_message::cli_to_coordinator::{
    CliToCoordinator, CliToCoordinatorEncoding, CliToCoordinatorRequest, CliToCoordinatorResponse,
};
use eyre::{Context, eyre};
use futures::{
    FutureExt,
    future::{self, Either},
};
use futures_concurrency::future::Race;
use std::{
    io::ErrorKind,
    net::SocketAddr,
    sync::{Arc, Weak},
};
use tokio::{
    net::{TcpListener, TcpStream},
    sync::{RwLock, Semaphore},
};
use tokio_util::sync::CancellationToken;

/// Shared state (held inside an `Arc`) for the control TCP listener tasks.
struct ListenState {
    /// `Weak` so the listener doesn't keep [`Coordinator`] alive; `upgrade()`
    /// returns `None` once the coordinator is dropped and the task exits.
    coordinator: Weak<RwLock<Coordinator>>,
    /// Caps concurrent requests that may hold locks on the coordinator,
    /// preventing overload (currently 10 permits).
    ///
    /// **TODO: add a const for this value and make it configurable.**
    semaphore: Semaphore,
    /// Allows the outer coordinator to signal all listener tasks to shut down.
    cancel_token: CancellationToken,
}

pub(crate) async fn listen(
    coordinator: Weak<RwLock<Coordinator>>,
    bind_control: SocketAddr,
    cancel_token: CancellationToken,
) {
    let result = TcpListener::bind(bind_control)
        .await
        .wrap_err("failed to listen for control messages");
    let incoming = match result {
        Ok(incoming) => incoming,
        Err(err) => {
            tracing::error!("{err:?}");
            return;
        }
    };

    let state = Arc::new(ListenState {
        coordinator,
        semaphore: Semaphore::new(10),
        cancel_token,
    });

    loop {
        let new_connection = incoming.accept().map(Either::Left);
        let coordinator_stop = state.cancel_token.cancelled().map(Either::Right);
        let connection = match (new_connection, coordinator_stop).race().await {
            future::Either::Left(connection) => connection,
            future::Either::Right(()) => {
                // coordinator was stopped
                break;
            }
        };
        match connection.wrap_err("failed to connect") {
            Ok((connection, _)) => {
                tokio::spawn(handle_requests(state.clone(), connection));
            }
            Err(err) => {
                tracing::error!("{err:?}");
            }
        }
    }
}

async fn handle_requests(state: Arc<ListenState>, connection: TcpStream) {
    let peer_addr = connection.peer_addr().ok();
    let mut transport = FramedTransport::new(connection)
        .with_encoding::<_, CliToCoordinatorResponse, CliToCoordinatorRequest>(
            CliToCoordinatorEncoding,
        );
    loop {
        let next_request = transport.receive().map(Either::Left);
        let coordinator_stopped = state.cancel_token.cancelled().map(Either::Right);
        let request = match (next_request, coordinator_stopped).race().await {
            Either::Right(()) => break,
            Either::Left(request) => match request {
                Ok(Some(request)) => request,
                Ok(None) => {
                    tracing::trace!("Control connection closed by peer");
                    break;
                }
                Err(err) => match err.kind() {
                    ErrorKind::UnexpectedEof => {
                        tracing::trace!("Control connection closed");
                        break;
                    }
                    err => {
                        let err = eyre!(err).wrap_err("failed to receive incoming message");
                        tracing::error!("{err}");
                        break;
                    }
                },
            },
        };

        if let CliToCoordinatorRequest::LogSubscribe { .. }
        | CliToCoordinatorRequest::BuildLogSubscribe { .. } = &request
        {
            let _guard = state.semaphore.acquire().await;
            let Some(coordinator) = state.coordinator.upgrade() else {
                break;
            };
            let mut this = coordinator.write().await;

            let (log_subscribers, buffered_log_messages, level) = match request {
                CliToCoordinatorRequest::LogSubscribe { dataflow_id, level } => {
                    if let Some(dataflow) = this.running_dataflows.get_mut(&dataflow_id) {
                        (
                            &mut dataflow.log_subscribers,
                            &mut dataflow.buffered_log_messages,
                            level,
                        )
                    } else {
                        break;
                    }
                }
                CliToCoordinatorRequest::BuildLogSubscribe { build_id, level } => {
                    if let Some(build) = this.running_builds.get_mut(&build_id) {
                        (
                            &mut build.log_subscribers,
                            &mut build.buffered_log_messages,
                            level,
                        )
                    } else {
                        break;
                    }
                }
                _ => unreachable!(),
            };

            log_subscribers.push(LogSubscriber::new(
                level,
                transport.into_inner(),
            ));
            let buffered = std::mem::take(buffered_log_messages);
            for message in buffered {
                send_log_message(log_subscribers, &message).await;
            }

            break;
        }

        let mut result: eyre::Result<CliToCoordinatorResponse> = {
            let _guard = state.semaphore.acquire().await;
            if let Some(coordinator) = state.coordinator.upgrade() {
                CliRequestHandler(coordinator).handle(request).await
            } else {
                Err(eyre!("coordinator has been stopped"))
            }
        };

        if let Ok(CliToCoordinatorResponse::CliAndDefaultDaemonOnSameMachine(result)) = &mut result
        {
            if result.cli.is_none() {
                // fill cli IP address in reply
                result.cli = peer_addr.map(|s| s.ip());
            }
        }

        let reply =
            result.unwrap_or_else(|err| CliToCoordinatorResponse::Error(format!("{err:?}")));
        match transport.send(&reply).await {
            Ok(()) => {}
            Err(err) => match err.kind() {
                ErrorKind::UnexpectedEof => {
                    tracing::debug!("Control connection closed while trying to send reply");
                    break;
                }
                err => {
                    let err = eyre!(err).wrap_err("failed to send reply");
                    tracing::error!("{err}");
                    break;
                }
            },
        }
    }
}
