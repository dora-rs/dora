use dora_core::coordinator_messages::LogMessage;

use crate::tcp_utils::tcp_send;

pub struct LogSubscriber {
    pub level: log::LevelFilter,
    connection: tokio::net::TcpStream,
}

impl LogSubscriber {
    pub fn new(level: log::LevelFilter, connection: tokio::net::TcpStream) -> Self {
        Self { level, connection }
    }

    pub async fn send_message(&mut self, message: &LogMessage) -> eyre::Result<()> {
        if message.level > self.level {
            return Ok(());
        }
        let message = serde_json::to_vec(&message)?;
        tcp_send(&mut self.connection, &message).await?;
        Ok(())
    }
}
