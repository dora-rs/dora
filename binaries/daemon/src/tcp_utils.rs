use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

pub async fn tcp_send(
    connection: &mut (impl AsyncWrite + Unpin),
    message: &[u8],
) -> std::io::Result<()> {
    let len_raw = (message.len() as u64).to_le_bytes();
    connection.write_all(&len_raw).await?;
    connection.write_all(message).await?;
    connection.flush().await?;
    Ok(())
}

pub async fn tcp_receive(connection: &mut (impl AsyncRead + Unpin)) -> std::io::Result<Vec<u8>> {
    let reply_len = {
        let mut raw = [0; 8];
        connection.read_exact(&mut raw).await?;
        u64::from_le_bytes(raw) as usize
    };
    let mut reply = vec![0; reply_len];
    connection.read_exact(&mut reply).await?;
    Ok(reply)
}
