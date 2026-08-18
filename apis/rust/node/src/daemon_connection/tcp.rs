use dora_message::{
    daemon_to_node::DaemonReply,
    node_to_daemon::{DaemonRequest, Timestamped},
};
use eyre::{Context, eyre};
use std::{
    io::{Read, Write},
    net::TcpStream,
};

enum Serializer {
    Bincode,
    SerdeJson,
}
pub fn request(
    connection: &mut TcpStream,
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
    connection: &mut TcpStream,
    message: &Timestamped<DaemonRequest>,
) -> eyre::Result<()> {
    let serialized = bincode::serialize(&message).wrap_err("failed to serialize DaemonRequest")?;
    tcp_send(connection, &serialized).wrap_err("failed to send DaemonRequest")?;
    Ok(())
}

fn receive_reply(
    connection: &mut TcpStream,
    serializer: Serializer,
) -> eyre::Result<Option<DaemonReply>> {
    let raw =
        match tcp_receive(connection) {
            Ok(raw) => raw,
            Err(err) => match err.kind() {
                std::io::ErrorKind::UnexpectedEof | std::io::ErrorKind::ConnectionAborted => {
                    return Ok(None);
                }
                other => return Err(err).with_context(|| {
                    format!(
                        "unexpected I/O error (kind {other:?}) while trying to receive DaemonReply"
                    )
                }),
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

/// Payload size (excluding the 8-byte header) at or below which `tcp_send`
/// coalesces the header and body into one buffer. Chosen to cover normal
/// control messages and small data outputs while keeping the extra copy
/// trivial; larger payloads are written header-then-body without copying.
const COALESCE_THRESHOLD: usize = 16 * 1024;

fn tcp_send(connection: &mut (impl Write + Unpin), message: &[u8]) -> std::io::Result<()> {
    if message.len() > dora_message::MAX_MESSAGE_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!(
                "outgoing message size {} exceeds maximum {}",
                message.len(),
                dora_message::MAX_MESSAGE_BYTES
            ),
        ));
    }
    let len_raw = (message.len() as u64).to_le_bytes();
    // For a small message, concatenate the 8-byte length header and the payload
    // into a single buffer and write it in one call. The connection has
    // `TCP_NODELAY` enabled (see `daemon_connection::connect`), so writing the
    // header separately would flush it as its own tiny TCP segment before the
    // body — an extra syscall and a runt segment on every request.
    //
    // Above the threshold we skip the coalescing copy and write the header and
    // body separately: this control channel also carries `SendMessage`/pinned-
    // memory payloads up to `MAX_MESSAGE_BYTES` (64 MiB) whenever an output
    // takes the daemon path instead of direct zenoh, and copying tens of MiB to
    // avoid one header segment is the wrong trade — for a large body the runt
    // header segment is negligible anyway. The wire format (8-byte LE length
    // prefix + body) is identical on both paths.
    if message.len() <= COALESCE_THRESHOLD {
        let mut buf = Vec::with_capacity(8 + message.len());
        buf.extend_from_slice(&len_raw);
        buf.extend_from_slice(message);
        connection.write_all(&buf)?;
    } else {
        connection.write_all(&len_raw)?;
        connection.write_all(message)?;
    }
    connection.flush()?;
    Ok(())
}

fn tcp_receive(connection: &mut (impl Read + Unpin)) -> std::io::Result<Vec<u8>> {
    let reply_len = {
        let mut raw = [0; 8];
        connection.read_exact(&mut raw)?;
        usize::try_from(u64::from_le_bytes(raw)).map_err(|_| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "message length overflows usize",
            )
        })?
    };
    if reply_len > dora_message::MAX_MESSAGE_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!(
                "message size {reply_len} exceeds maximum {}",
                dora_message::MAX_MESSAGE_BYTES
            ),
        ));
    }
    let mut reply = vec![0; reply_len];
    connection.read_exact(&mut reply)?;
    Ok(reply)
}

#[cfg(test)]
mod tests {
    use super::{tcp_receive, tcp_send};
    use std::io::Cursor;

    // Length-prefixed framing guards on the node side of the daemon<->node TCP
    // transport (dora-rs/dora#2027 verified test gap). A malformed/hostile frame
    // must error, not over-allocate or hang.

    #[test]
    fn framing_roundtrip() {
        let payload = b"hello dora framing".to_vec();
        let mut wire = Vec::new();
        tcp_send(&mut wire, &payload).expect("send");
        // 8-byte little-endian length prefix + payload.
        assert_eq!(wire.len(), 8 + payload.len());
        let got = tcp_receive(&mut Cursor::new(wire)).expect("receive");
        assert_eq!(got, payload);
    }

    #[test]
    fn empty_payload_roundtrips() {
        let mut wire = Vec::new();
        tcp_send(&mut wire, &[]).expect("send empty");
        let got = tcp_receive(&mut Cursor::new(wire)).expect("receive empty");
        assert!(got.is_empty());
    }

    #[test]
    fn large_payload_roundtrips_uncoalesced() {
        // A payload above COALESCE_THRESHOLD takes the header-then-body write
        // path (no coalescing copy). The wire layout must be identical: an
        // 8-byte length prefix followed by the body.
        let payload = vec![0xCD_u8; super::COALESCE_THRESHOLD + 1];
        let mut wire = Vec::new();
        tcp_send(&mut wire, &payload).expect("send large");
        assert_eq!(wire.len(), 8 + payload.len());
        let got = tcp_receive(&mut Cursor::new(wire)).expect("receive large");
        assert_eq!(got, payload);
    }

    #[test]
    fn receive_rejects_oversized_length_prefix() {
        // A hostile peer announces a body larger than MAX_MESSAGE_BYTES. The
        // guard must reject *before* allocating the body buffer; we therefore
        // supply only the 8-byte header (no body).
        let claimed = dora_message::MAX_MESSAGE_BYTES as u64 + 1;
        let header = claimed.to_le_bytes().to_vec();
        let err = tcp_receive(&mut Cursor::new(header)).expect_err("must reject oversized frame");
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidData);
        assert!(
            err.to_string().contains("exceeds maximum"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn receive_rejects_truncated_header() {
        // Fewer than 8 length bytes -> EOF, not a hang or panic.
        let err = tcp_receive(&mut Cursor::new(vec![1, 2, 3])).expect_err("truncated header");
        assert_eq!(err.kind(), std::io::ErrorKind::UnexpectedEof);
    }

    #[test]
    fn receive_rejects_truncated_body() {
        // Header claims 16 bytes but only 4 follow.
        let mut wire = 16u64.to_le_bytes().to_vec();
        wire.extend_from_slice(&[0xAB; 4]);
        let err = tcp_receive(&mut Cursor::new(wire)).expect_err("truncated body");
        assert_eq!(err.kind(), std::io::ErrorKind::UnexpectedEof);
    }
}
