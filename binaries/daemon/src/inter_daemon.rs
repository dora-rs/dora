use crate::tcp_utils::{tcp_receive, tcp_send};
use dora_core::daemon_messages::{InterDaemonEvent, Timestamped};
use eyre::{Context, ContextCompat};
use std::{
    collections::BTreeMap,
    io::ErrorKind,
    net::{Ipv4Addr, SocketAddr},
};
use tokio::net::{TcpListener, TcpStream};

pub struct InterDaemonConnection {
    socket: SocketAddr,
    connection: Option<TcpStream>,
}

impl InterDaemonConnection {
    pub fn new(socket: SocketAddr) -> Self {
        Self {
            socket,
            connection: None,
        }
    }

    #[tracing::instrument(skip(self), fields(%self.socket))]
    async fn connect(&mut self) -> eyre::Result<&mut TcpStream> {
        match &mut self.connection {
            Some(c) => Ok(c),
            entry @ None => {
                let connection = TcpStream::connect(self.socket)
                    .await
                    .wrap_err("failed to connect")?;
                connection
                    .set_nodelay(true)
                    .wrap_err("failed to set nodelay")?;
                Ok(entry.insert(connection))
            }
        }
    }

    pub fn socket(&self) -> SocketAddr {
        self.socket
    }
}

#[tracing::instrument(skip(inter_daemon_connections))]
pub async fn send_inter_daemon_event(
    target_machines: &[String],
    inter_daemon_connections: &mut BTreeMap<String, InterDaemonConnection>,
    event: &Timestamped<InterDaemonEvent>,
) -> eyre::Result<()> {
    let message = bincode::serialize(event).wrap_err("failed to serialize InterDaemonEvent")?;
    for target_machine in target_machines {
        let connection = inter_daemon_connections
            .get_mut(target_machine)
            .wrap_err_with(|| format!("unknown target machine `{target_machine}`"))?
            .connect()
            .await
            .wrap_err_with(|| format!("failed to connect to machine `{target_machine}`"))?;
        tcp_send(connection, &message)
            .await
            .wrap_err_with(|| format!("failed to send event to machine `{target_machine}`"))?;
    }

    Ok(())
}

pub async fn spawn_listener_loop(
    machine_id: String,
    events_tx: flume::Sender<Timestamped<InterDaemonEvent>>,
) -> eyre::Result<SocketAddr> {
    let localhost = Ipv4Addr::new(127, 0, 0, 1);
    let socket = match TcpListener::bind((localhost, 0)).await {
        Ok(socket) => socket,
        Err(err) => {
            return Err(eyre::Report::new(err).wrap_err("failed to create local TCP listener"))
        }
    };
    let socket_addr = socket
        .local_addr()
        .wrap_err("failed to get local addr of socket")?;

    tokio::spawn(async move {
        listener_loop(socket, events_tx).await;
        tracing::debug!("inter-daemon listener loop finished for machine `{machine_id}`");
    });

    Ok(socket_addr)
}

async fn listener_loop(
    listener: TcpListener,
    events_tx: flume::Sender<Timestamped<InterDaemonEvent>>,
) {
    loop {
        match listener
            .accept()
            .await
            .wrap_err("failed to accept new connection")
        {
            Err(err) => {
                tracing::info!("{err}");
            }
            Ok((connection, _)) => {
                tokio::spawn(handle_connection_loop(connection, events_tx.clone()));
            }
        }
    }
}

async fn handle_connection_loop(
    mut connection: TcpStream,
    events_tx: flume::Sender<Timestamped<InterDaemonEvent>>,
) {
    if let Err(err) = connection.set_nodelay(true) {
        tracing::warn!("failed to set nodelay for connection: {err}");
    }

    loop {
        match receive_message(&mut connection).await {
            Ok(Some(message)) => {
                if events_tx.send_async(message).await.is_err() {
                    break;
                }
            }
            Ok(None) => break,
            Err(err) => {
                tracing::warn!("{err:?}");
                break;
            }
        }
    }
}

async fn receive_message(
    connection: &mut TcpStream,
) -> eyre::Result<Option<Timestamped<InterDaemonEvent>>> {
    let raw = match tcp_receive(connection).await {
        Ok(raw) => raw,
        Err(err) => match err.kind() {
            ErrorKind::UnexpectedEof
            | ErrorKind::ConnectionAborted
            | ErrorKind::ConnectionReset => return Ok(None),
            _other => {
                return Err(err)
                    .context("unexpected I/O error while trying to receive InterDaemonEvent")
            }
        },
    };
    bincode::deserialize(&raw)
        .wrap_err("failed to deserialize DaemonRequest")
        .map(Some)
}
