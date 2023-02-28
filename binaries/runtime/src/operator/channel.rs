use super::IncomingEvent;
use futures::{
    future::{self, FusedFuture},
    FutureExt,
};
use std::collections::VecDeque;

pub fn channel(
    runtime: &tokio::runtime::Handle,
) -> (flume::Sender<IncomingEvent>, flume::Receiver<IncomingEvent>) {
    let (incoming_tx, incoming_rx) = flume::bounded(10);
    let (outgoing_tx, outgoing_rx) = flume::bounded(0);

    runtime.spawn(async {
        let mut buffer = InputBuffer::new();
        buffer.run(incoming_rx, outgoing_tx).await;
    });

    (incoming_tx, outgoing_rx)
}

struct InputBuffer {
    queue: VecDeque<IncomingEvent>,
}

impl InputBuffer {
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }

    pub async fn run(
        &mut self,
        incoming: flume::Receiver<IncomingEvent>,
        outgoing: flume::Sender<IncomingEvent>,
    ) {
        let mut send_out_buf = future::Fuse::terminated();
        loop {
            let next_incoming = incoming.recv_async();
            match future::select(next_incoming, send_out_buf).await {
                future::Either::Left((event, mut send_out)) => {
                    match event {
                        Ok(event) => {
                            // received a new event -> push it to the queue
                            self.queue.push_back(event);

                            // TODO: drop oldest events when queue becomes too full

                            // if outgoing queue is empty, fill it again
                            if send_out.is_terminated() {
                                send_out = self.send_next_queued(&outgoing);
                            }
                        }
                        Err(flume::RecvError::Disconnected) => {
                            // the incoming channel was closed -> exit if we sent out all events already
                            if send_out.is_terminated() && self.queue.is_empty() {
                                break;
                            }
                        }
                    }

                    // reassign the send_out future, which might be still in progress
                    send_out_buf = send_out;
                }
                future::Either::Right((send_result, _)) => match send_result {
                    Ok(()) => {
                        send_out_buf = self.send_next_queued(&outgoing);
                    }
                    Err(flume::SendError(_)) => break,
                },
            };
        }
    }

    fn send_next_queued<'a>(
        &mut self,
        outgoing: &'a flume::Sender<IncomingEvent>,
    ) -> future::Fuse<flume::r#async::SendFut<'a, IncomingEvent>> {
        if let Some(next) = self.queue.pop_front() {
            outgoing.send_async(next).fuse()
        } else {
            future::Fuse::terminated()
        }
    }
}

impl Default for InputBuffer {
    fn default() -> Self {
        Self::new()
    }
}
