use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::TcpStream,
};

pub async fn tcp_send(connection: &mut TcpStream, message: &[u8]) -> std::io::Result<()> {
    if message.len() > adora_message::MAX_MESSAGE_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!(
                "outgoing message size {} exceeds maximum {}",
                message.len(),
                adora_message::MAX_MESSAGE_BYTES
            ),
        ));
    }
    let len_raw = (message.len() as u64).to_le_bytes();
    connection.write_all(&len_raw).await?;
    connection.write_all(message).await?;
    connection.flush().await?;
    Ok(())
}

pub async fn tcp_receive(connection: &mut TcpStream) -> std::io::Result<Vec<u8>> {
    let timeout = adora_message::TCP_READ_TIMEOUT;
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
    if reply_len > adora_message::MAX_MESSAGE_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!(
                "message size {reply_len} exceeds maximum {}",
                adora_message::MAX_MESSAGE_BYTES
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
