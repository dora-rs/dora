use dora_core::daemon_messages::{DaemonReply, DaemonRequest, Timestamped};
use eyre::{eyre, Context};
use std::{
    io::{Read, Write},
    net::TcpStream,
};

pub fn request(
    connection: &mut TcpStream,
    request: &Timestamped<DaemonRequest>,
) -> eyre::Result<DaemonReply> {
    send_message(connection, request)?;
    if request.inner.expects_tcp_reply() {
        receive_reply(connection)
            .and_then(|reply| reply.ok_or_else(|| eyre!("server disconnected unexpectedly")))
    } else {
        Ok(DaemonReply::Empty)
    }
}

fn send_message(
    connection: &mut TcpStream,
    message: &Timestamped<DaemonRequest>,
) -> eyre::Result<()> {
    let serialized = bincode::serialize(&message).wrap_err("failed to serialize DaemonRequest")?;
    tcp_send(connection, &serialized).wrap_err("failed to send DaemonRequest")?;
    Ok(())
}

fn receive_reply(connection: &mut TcpStream) -> eyre::Result<Option<DaemonReply>> {
    let raw = match tcp_receive(connection) {
        Ok(raw) => raw,
        Err(err) => match err.kind() {
            std::io::ErrorKind::UnexpectedEof | std::io::ErrorKind::ConnectionAborted => {
                return Ok(None)
            }
            other => {
                return Err(err).with_context(|| {
                    format!(
                        "unexpected I/O error (kind {other:?}) while trying to receive DaemonReply"
                    )
                })
            }
        },
    };
    bincode::deserialize(&raw)
        .wrap_err("failed to deserialize DaemonReply")
        .map(Some)
}

fn tcp_send(connection: &mut (impl Write + Unpin), message: &[u8]) -> std::io::Result<()> {
    let len_raw = (message.len() as u64).to_le_bytes();
    connection.write_all(&len_raw)?;
    connection.write_all(message)?;
    connection.flush()?;
    Ok(())
}

fn tcp_receive(connection: &mut (impl Read + Unpin)) -> std::io::Result<Vec<u8>> {
    let reply_len = {
        let mut raw = [0; 8];
        connection.read_exact(&mut raw)?;
        u64::from_le_bytes(raw) as usize
    };
    let mut reply = vec![0; reply_len];
    connection.read_exact(&mut reply)?;
    Ok(reply)
}
