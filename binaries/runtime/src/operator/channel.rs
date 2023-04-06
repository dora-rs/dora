use dora_core::config::DataId;
use dora_node_api::Event;
use futures::{
    future::{self, FusedFuture},
    FutureExt,
};
use std::collections::{BTreeMap, VecDeque};

pub fn channel(
    runtime: &tokio::runtime::Handle,
    queue_sizes: BTreeMap<DataId, usize>,
) -> (flume::Sender<Event>, flume::Receiver<Event>) {
    let (incoming_tx, incoming_rx) = flume::bounded(10);
    let (outgoing_tx, outgoing_rx) = flume::bounded(0);

    runtime.spawn(async {
        let mut buffer = InputBuffer::new(queue_sizes);
        buffer.run(incoming_rx, outgoing_tx).await;
    });

    (incoming_tx, outgoing_rx)
}

struct InputBuffer {
    queue: VecDeque<Option<Event>>,
    queue_sizes: BTreeMap<DataId, usize>,
}

impl InputBuffer {
    pub fn new(queue_sizes: BTreeMap<DataId, usize>) -> Self {
        Self {
            queue: VecDeque::new(),
            queue_sizes,
        }
    }

    pub async fn run(&mut self, incoming: flume::Receiver<Event>, outgoing: flume::Sender<Event>) {
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

                            // if outgoing queue is empty, fill it again
                            if send_out.is_terminated() {
                                send_out = self.send_next_queued(&outgoing);
                            }
                        }
                        Err(flume::RecvError::Disconnected) => {
                            incoming_closed = true;
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
            if incoming_closed && send_out_buf.is_terminated() && self.queue.is_empty() {
                break;
            }
        }
    }

    fn send_next_queued<'a>(
        &mut self,
        outgoing: &'a flume::Sender<Event>,
    ) -> future::Fuse<flume::r#async::SendFut<'a, Event>> {
        loop {
            match self.queue.pop_front() {
                Some(Some(next)) => break outgoing.send_async(next).fuse(),
                Some(None) => {
                    // dropped event, try again with next one
                }
                None => break future::Fuse::terminated(),
            }
        }
    }

    fn add_event(&mut self, event: Event) {
        self.queue.push_back(Some(event));

        // drop oldest input events to maintain max queue length queue
        self.drop_oldest_inputs();
    }

    fn drop_oldest_inputs(&mut self) {
        let mut queue_size_remaining = self.queue_sizes.clone();
        let mut dropped = 0;

        // iterate over queued events, newest first
        for event in self.queue.iter_mut().rev() {
            let Some(Event::Input { id: input_id, .. }) = event.as_mut() else {
                continue;
            };
            match queue_size_remaining.get_mut(input_id) {
                Some(0) => {
                    dropped += 1;
                    *event = None;
                }
                Some(size_remaining) => {
                    *size_remaining = size_remaining.saturating_sub(1);
                }
                None => {
                    tracing::warn!("no queue size known for received operator input `{input_id}`");
                }
            }
        }

        if dropped > 0 {
            tracing::debug!("dropped {dropped} operator inputs because event queue was too full");
        }
    }
}
