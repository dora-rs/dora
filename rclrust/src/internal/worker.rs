use futures::{channel::mpsc, StreamExt};

use super::async_api::{self, JoinHandle};
use crate::{log::Logger, rclrust_debug};

#[derive(Debug)]
pub enum WorkerMessage<T> {
    Message(T),
    Terminate,
}

#[derive(Debug)]
pub struct ReceiveWorker<T> {
    tx: mpsc::Sender<WorkerMessage<T>>,
    _thread: Option<JoinHandle<()>>,
}

impl<T> ReceiveWorker<T> {
    pub fn new<F>(callback: F) -> Self
    where
        T: Send + 'static,
        F: Fn(T) + Send + 'static,
    {
        let (tx, mut rx) = mpsc::channel(10);

        let thread = async_api::spawn(async move {
            while let Some(message) = rx.next().await {
                match message {
                    WorkerMessage::Message(msg) => callback(msg),
                    WorkerMessage::Terminate => break,
                }
            }

            rclrust_debug!(Logger::new("rclrust"), "Channel is closed.");
        });

        Self {
            tx,
            _thread: Some(thread),
        }
    }

    pub fn clone_tx(&self) -> mpsc::Sender<WorkerMessage<T>> {
        self.tx.clone()
    }

    pub fn terminate(&mut self) {
        let _ = self.tx.try_send(WorkerMessage::Terminate);
    }
}

impl<T> Drop for ReceiveWorker<T> {
    fn drop(&mut self) {
        self.terminate();
    }
}
