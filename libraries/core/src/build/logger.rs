use std::future::Future;

use dora_message::common::LogLevel;

pub trait BuildLogger: Send {
    type Clone: BuildLogger + 'static;

    fn log_message(
        &mut self,
        level: LogLevel,
        message: impl Into<String> + Send,
    ) -> impl Future<Output = ()> + Send;

    fn try_clone(&self) -> impl Future<Output = eyre::Result<Self::Clone>> + Send;
}
