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
    max_queue_len: usize,
}

impl InputBuffer {
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
            max_queue_len: 10,
        }
    }

    pub async fn run(
        &mut self,
        incoming: flume::Receiver<IncomingEvent>,
        outgoing: flume::Sender<IncomingEvent>,
    ) {
        let mut send_out_buf = future::Fuse::terminated();
        let mut incoming_closed = false;
        loop {
            let next_incoming = if incoming_closed {
                future::Fuse::terminated()
            } else {
                incoming.recv_async().fuse()
            };
            match future::select(next_incoming, send_out_buf).await {
                future::Either::Left((event, mut send_out)) => {
                    match event {
                        Ok(event) => {
                            // received a new event -> push it to the queue
                            self.add_event(event);

                            // TODO: drop oldest events when queue becomes too full

                            // if outgoing queue is empty, fill it again
                            if send_out.is_terminated() {
                                send_out = self.send_next_queued(&outgoing);
                            }
                        }
                        Err(flume::RecvError::Disconnected) => {
                            incoming_closed = true;
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

    fn add_event(&mut self, event: IncomingEvent) {
        self.queue.push_back(event);

        // drop oldest input events to maintain max queue length queue
        let input_event_count = self
            .queue
            .iter()
            .filter(|e| matches!(e, IncomingEvent::Input { .. }))
            .count();
        let drop_n = input_event_count.saturating_sub(self.max_queue_len);
        if drop_n > 0 {
            self.drop_oldest_inputs(drop_n);
        }
    }

    fn drop_oldest_inputs(&mut self, number: usize) {
        tracing::debug!("dropping {number} operator inputs because event queue is too full");
        for i in 0..number {
            // find index of oldest input event
            let index = self
                .queue
                .iter()
                .position(|e| matches!(e, IncomingEvent::Input { .. }))
                .unwrap_or_else(|| panic!("no input event found in drop iteration {i}"));

            // remove that event
            self.queue.remove(index);
        }
    }
}

impl Default for InputBuffer {
    fn default() -> Self {
        Self::new()
    }
}
