use std::{
    net::{Shutdown, SocketAddr, TcpStream},
    sync::Arc,
};

use dora_core::{config::NodeId, uhlc::HLC};
use dora_message::{
    DataflowId,
    daemon_to_node::DaemonReply,
    node_to_daemon::{DaemonRequest, Timestamped},
};

use crate::daemon_connection::DaemonChannel;
use futures::channel::oneshot;

/// Interrupt the blocking read if the event stream and its waiters go away.
struct CloseOnDrop(TcpStream);

impl Drop for CloseOnDrop {
    fn drop(&mut self) {
        let _ = self.0.shutdown(Shutdown::Both);
    }
}

pub(super) async fn wait(
    addr: SocketAddr,
    dataflow_id: DataflowId,
    node_id: NodeId,
    clock: Arc<HLC>,
) {
    let (connected, socket_guard) = oneshot::channel();
    let (done, reply) = oneshot::channel();
    let listen = move || -> eyre::Result<DaemonReply> {
        let socket = TcpStream::connect(addr)?;
        if connected.send(CloseOnDrop(socket.try_clone()?)).is_err() {
            return Ok(DaemonReply::Empty);
        }
        let mut channel = DaemonChannel::Tcp(socket);
        channel.register(dataflow_id, node_id, clock.new_timestamp())?;
        channel.request(&Timestamped {
            inner: DaemonRequest::WaitForManualStop,
            timestamp: clock.new_timestamp(),
        })
    };
    let thread = std::thread::Builder::new()
        .name("dora-manual-stop".into())
        .spawn(move || {
            let _ = done.send(listen());
        });
    match thread {
        Ok(_) => {
            let _guard = socket_guard.await;
            match reply.await {
                Ok(Ok(DaemonReply::Result(Ok(())))) => return,
                Ok(Ok(DaemonReply::Result(Err(err)))) => {
                    tracing::debug!(%err, "manual stop wait rejected")
                }
                Ok(Err(err)) => tracing::debug!(%err, "manual stop connection unavailable"),
                _ => tracing::debug!("manual stop connection ended without a stop"),
            }
        }
        Err(err) => tracing::debug!(%err, "could not start manual stop listener"),
    }
    // Older daemons reject the new request. The ordinary Stop event remains usable.
    std::future::pending().await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::EventStream;
    use dora_message::daemon_to_node::{DaemonCommunication, NodeEvent};
    use futures::FutureExt;
    use std::time::Duration;
    use tokio::{
        io::{AsyncReadExt, AsyncWriteExt},
        net::{TcpListener, TcpStream},
        time::timeout,
    };

    async fn request(socket: &mut TcpStream) -> DaemonRequest {
        let len = socket.read_u64_le().await.unwrap();
        assert!(len <= dora_message::MAX_MESSAGE_BYTES as u64);
        let mut bytes = vec![0; len as usize];
        socket.read_exact(&mut bytes).await.unwrap();
        dora_message::decode::<Timestamped<DaemonRequest>>(&bytes)
            .unwrap()
            .inner
    }

    async fn reply(socket: &mut TcpStream, reply: DaemonReply) {
        let bytes = dora_message::encode(&reply).unwrap();
        socket.write_u64_le(bytes.len() as u64).await.unwrap();
        socket.write_all(&bytes).await.unwrap();
    }

    async fn registered(listener: &TcpListener) -> TcpStream {
        let (mut socket, _) = listener.accept().await.unwrap();
        assert!(matches!(
            request(&mut socket).await,
            DaemonRequest::Register(_)
        ));
        reply(&mut socket, DaemonReply::Result(Ok(()))).await;
        socket
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn manual_stop_uses_event_channel_when_control_request_is_rejected() {
        let listener = TcpListener::bind(("127.0.0.1", 0)).await.unwrap();
        let socket_addr = listener.local_addr().unwrap();
        let (rejected, rejection) = oneshot::channel();
        let (send_stop, release) = oneshot::channel();
        let deadline = Duration::from_secs(5);
        let daemon = tokio::spawn(timeout(deadline, async move {
            let mut events = registered(&listener).await;
            assert!(matches!(
                request(&mut events).await,
                DaemonRequest::Subscribe
            ));
            reply(&mut events, DaemonReply::Result(Ok(()))).await;
            let mut close = registered(&listener).await;
            let mut control = registered(&listener).await;
            assert!(matches!(
                request(&mut control).await,
                DaemonRequest::WaitForManualStop
            ));
            reply(
                &mut control,
                DaemonReply::Result(Err("unsupported request".into())),
            )
            .await;
            // The client has processed the rejection when it closes this socket.
            assert_eq!(control.read(&mut [0]).await.unwrap(), 0);
            rejected.send(()).unwrap();
            release.await.unwrap();

            assert!(matches!(
                request(&mut events).await,
                DaemonRequest::NextEvent
            ));
            let stop = Timestamped {
                inner: NodeEvent::Stop,
                timestamp: HLC::default().new_timestamp(),
            };
            reply(&mut events, DaemonReply::NextEvents(vec![stop])).await;
            assert!(matches!(
                request(&mut events).await,
                DaemonRequest::NextEvent
            ));
            reply(&mut events, DaemonReply::NextEvents(vec![])).await;
            assert!(matches!(
                request(&mut close).await,
                DaemonRequest::EventStreamDropped
            ));
            reply(&mut close, DaemonReply::Result(Ok(()))).await;
        }));

        let events = EventStream::init(
            DataflowId::nil(),
            &"client".parse().unwrap(),
            &DaemonCommunication::Tcp { socket_addr }.into(),
            Default::default(),
            &Default::default(),
            Arc::default(),
            None,
            None,
        )
        .unwrap();
        let mut stopped = Box::pin(events.manual_stop());
        timeout(deadline, async {
            tokio::select! {
                biased;
                () = &mut stopped => panic!("control rejection must not signal Stop"),
                result = rejection => result.unwrap(),
            }
            assert!(stopped.as_mut().now_or_never().is_none());
            send_stop.send(()).unwrap();
            stopped.await;
        })
        .await
        .expect("the ordinary Stop must wake the waiter after rejection");
        drop(events);
        daemon.await.unwrap().unwrap();
    }
}
