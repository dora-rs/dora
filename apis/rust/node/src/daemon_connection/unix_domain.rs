use dora_message::{
    daemon_to_node::DaemonReply,
    node_to_daemon::{DaemonRequest, Timestamped},
};
use eyre::{eyre, Context};
use std::{
    io::{Read, Write},
    os::unix::net::UnixStream,
};

enum Serializer {
    Bincode,
    SerdeJson,
}
pub fn request(
    connection: &mut UnixStream,
    request: &Timestamped<DaemonRequest>,
) -> eyre::Result<DaemonReply> {
    send_message(connection, request)?;
    if request.inner.expects_tcp_bincode_reply() {
        receive_reply(connection, Serializer::Bincode)
            .and_then(|reply| reply.ok_or_else(|| eyre!("server disconnected unexpectedly")))
    // Use serde json for message with variable length
    } else if request.inner.expects_tcp_json_reply() {
        receive_reply(connection, Serializer::SerdeJson)
            .and_then(|reply| reply.ok_or_else(|| eyre!("server disconnected unexpectedly")))
    } else {
        Ok(DaemonReply::Empty)
    }
}

fn send_message(
    connection: &mut UnixStream,
    message: &Timestamped<DaemonRequest>,
) -> eyre::Result<()> {
    let serialized = bincode::serialize(&message).wrap_err("failed to serialize DaemonRequest")?;
    stream_send(connection, &serialized).wrap_err("failed to send DaemonRequest")?;
    Ok(())
}

fn receive_reply(
    connection: &mut UnixStream,
    serializer: Serializer,
) -> eyre::Result<Option<DaemonReply>> {
    let raw = match stream_receive(connection) {
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
    match serializer {
        Serializer::Bincode => bincode::deserialize(&raw)
            .wrap_err("failed to deserialize DaemonReply")
            .map(Some),
        Serializer::SerdeJson => serde_json::from_slice(&raw)
            .wrap_err("failed to deserialize DaemonReply")
            .map(Some),
    }
}

fn stream_send(connection: &mut (impl Write + Unpin), message: &[u8]) -> std::io::Result<()> {
    let len_raw = (message.len() as u64).to_le_bytes();
    connection.write_all(&len_raw)?;
    connection.write_all(message)?;
    connection.flush()?;
    Ok(())
}

fn stream_receive(connection: &mut (impl Read + Unpin)) -> std::io::Result<Vec<u8>> {
    let reply_len = {
        let mut raw = [0; 8];
        connection.read_exact(&mut raw)?;
        u64::from_le_bytes(raw) as usize
    };
    let mut reply = vec![0; reply_len];
    connection.read_exact(&mut reply)?;
    Ok(reply)
}
