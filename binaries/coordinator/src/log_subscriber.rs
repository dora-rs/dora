use communication_layer_request_reply::{
    AsyncTransport, EncodedTransport, Protocol, transport::FramedTransport,
};
use dora_message::{cli_to_coordinator::CliToCoordinatorProtocol, coordinator_to_cli::LogMessage};
use eyre::{Context, ContextCompat};
use tokio::net::TcpStream;

pub struct LogSubscriber {
    pub level: log::LevelFilter,
    transport: Option<
        EncodedTransport<
            FramedTransport<TcpStream>,
            <CliToCoordinatorProtocol as Protocol>::Encoding,
            LogMessage,
            (),
        >,
    >,
}

impl LogSubscriber {
    pub fn new(level: log::LevelFilter, transport: FramedTransport<TcpStream>) -> Self {
        Self {
            level,
            transport: Some(EncodedTransport::new(transport, Default::default())),
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

        let connection = self.transport.as_mut().context("connection is closed")?;
        connection
            .send(message)
            .await
            .context("failed to send message")?;
        Ok(())
    }

    pub fn is_closed(&self) -> bool {
        self.transport.is_none()
    }

    pub fn close(&mut self) {
        self.transport = None;
    }
}
