use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

pub async fn socket_stream_send(
    connection: &mut (impl AsyncWrite + Unpin),
    message: &[u8],
) -> std::io::Result<()> {
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
    // Concatenate length header + payload into a single buffer to avoid
    // sending a tiny 8-byte TCP segment with TCP_NODELAY enabled.
    // Previously used two separate write_all calls (2-3 syscalls per message).
    let len_raw = (message.len() as u64).to_le_bytes();
    let mut buf = Vec::with_capacity(8 + message.len());
    buf.extend_from_slice(&len_raw);
    buf.extend_from_slice(message);
    connection.write_all(&buf).await?;
    Ok(())
}

pub async fn socket_stream_receive(
    connection: &mut (impl AsyncRead + Unpin),
) -> std::io::Result<Vec<u8>> {
    let timeout = dora_message::TCP_READ_TIMEOUT;
    let reply_len = {
        let mut raw = [0; 8];
        tokio::time::timeout(timeout, connection.read_exact(&mut raw))
            .await
            .map_err(|_| {
                std::io::Error::new(std::io::ErrorKind::TimedOut, "TCP read header timed out")
            })??;
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
    tokio::time::timeout(timeout, connection.read_exact(&mut reply))
        .await
        .map_err(|_| {
            std::io::Error::new(std::io::ErrorKind::TimedOut, "TCP read body timed out")
        })??;
    Ok(reply)
}

#[cfg(test)]
mod tests {
    use super::{socket_stream_receive, socket_stream_send};
    use tokio::io::{AsyncWriteExt, duplex};

    // Length-prefixed framing guards on the daemon side of the daemon<->node TCP
    // transport (dora-rs/dora#2027 verified test gap): DoS-sized frames must be
    // rejected before allocation, a stalled peer must time out, and well-formed
    // frames must round-trip.

    #[tokio::test]
    async fn framing_roundtrip() {
        let (mut a, mut b) = duplex(64 * 1024);
        let payload = b"hello dora framing".to_vec();
        socket_stream_send(&mut a, &payload).await.expect("send");
        let got = socket_stream_receive(&mut b).await.expect("receive");
        assert_eq!(got, payload);
    }

    #[tokio::test]
    async fn empty_payload_roundtrips() {
        let (mut a, mut b) = duplex(64);
        socket_stream_send(&mut a, &[]).await.expect("send empty");
        let got = socket_stream_receive(&mut b).await.expect("receive empty");
        assert!(got.is_empty());
    }

    #[tokio::test]
    async fn send_rejects_oversized_message() {
        let (mut a, _b) = duplex(64);
        let oversized = vec![0u8; dora_message::MAX_MESSAGE_BYTES + 1];
        let err = socket_stream_send(&mut a, &oversized)
            .await
            .expect_err("must reject oversized outgoing message");
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidData);
        assert!(err.to_string().contains("exceeds maximum"), "{err}");
    }

    #[tokio::test]
    async fn receive_rejects_oversized_length_prefix() {
        // Hostile peer announces a body larger than MAX_MESSAGE_BYTES. The guard
        // must reject from the 8-byte header alone, before allocating the body.
        let (mut a, mut b) = duplex(64);
        let claimed = dora_message::MAX_MESSAGE_BYTES as u64 + 1;
        a.write_all(&claimed.to_le_bytes())
            .await
            .expect("write hdr");
        let err = socket_stream_receive(&mut b)
            .await
            .expect_err("must reject oversized frame");
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidData);
        assert!(err.to_string().contains("exceeds maximum"), "{err}");
    }

    #[tokio::test(start_paused = true)]
    async fn receive_times_out_on_stalled_peer() {
        // Writer stays open but sends nothing: the header read must time out
        // (not hang). Paused time auto-advances past TCP_READ_TIMEOUT, so the
        // test is instant.
        let (_writer, mut reader) = duplex(64);
        let err = socket_stream_receive(&mut reader)
            .await
            .expect_err("must time out on a stalled peer");
        assert_eq!(err.kind(), std::io::ErrorKind::TimedOut);
    }
}
