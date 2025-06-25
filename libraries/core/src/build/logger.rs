use std::future::Future;

pub use dora_message::common::LogLevelOrStdout;

pub trait BuildLogger: Send {
    type Clone: BuildLogger + 'static;

    fn log_message(
        &mut self,
        level: impl Into<LogLevelOrStdout> + Send,
        message: impl Into<String> + Send,
    ) -> impl Future<Output = ()> + Send;

    fn log_stdout(&mut self, message: impl Into<String> + Send) -> impl Future<Output = ()> + Send {
        self.log_message(LogLevelOrStdout::Stdout, message)
    }

    fn try_clone(&self) -> impl Future<Output = eyre::Result<Self::Clone>> + Send;
}
