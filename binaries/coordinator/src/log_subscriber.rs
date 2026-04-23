use dora_message::coordinator_to_cli::LogMessage;
use eyre::{Context, ContextCompat};

pub struct LogSubscriber {
    pub level: log::LevelFilter,
    sender: Option<tokio::sync::mpsc::Sender<String>>,
}

impl LogSubscriber {
    pub fn new(level: log::LevelFilter, sender: tokio::sync::mpsc::Sender<String>) -> Self {
        Self {
            level,
            sender: Some(sender),
        }
    }

    pub async fn send_message(&mut self, message: &LogMessage) -> eyre::Result<()> {
        match message.level {
            dora_core::build::LogLevelOrStdout::LogLevel(level) => {
                if level > self.level {
                    return Ok(());
                }
            }
            dora_core::build::LogLevelOrStdout::Stdout => {}
        }

        let sender = self.sender.as_ref().context("subscriber is closed")?;
        let json = serde_json::to_string(&dora_message::ws_protocol::WsEvent {
            event: "log".to_string(),
            payload: serde_json::to_value(message)?,
        })
        .context("failed to serialize log WsEvent")?;
        sender
            .send(json)
            .await
            .map_err(|_e| eyre::eyre!("WS log subscriber channel closed"))?;
        Ok(())
    }

    pub fn is_closed(&self) -> bool {
        match &self.sender {
            None => true,
            Some(sender) => sender.is_closed(),
        }
    }

    pub fn close(&mut self) {
        self.sender = None;
    }
}
