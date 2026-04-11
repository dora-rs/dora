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
