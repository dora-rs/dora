use adora_core::config::DataId;
use adora_message::config::QueuePolicy;
use adora_node_api::Event;
use futures::{
    FutureExt,
    future::{self, FusedFuture},
};
use std::collections::{BTreeMap, VecDeque};

pub fn channel(
    runtime: &tokio::runtime::Handle,
    queue_sizes: BTreeMap<DataId, (usize, QueuePolicy)>,
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
    /// Pre-computed effective cap per input ID.
    effective_caps: BTreeMap<DataId, (usize, QueuePolicy)>,
}

impl InputBuffer {
    pub fn new(queue_sizes: BTreeMap<DataId, (usize, QueuePolicy)>) -> Self {
        let effective_caps = queue_sizes
            .into_iter()
            .map(|(id, (size, policy))| (id, (policy.effective_cap(size), policy)))
            .collect();
        Self {
            queue: VecDeque::new(),
            effective_caps,
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
        let mut remaining: BTreeMap<&DataId, usize> = self
            .effective_caps
            .iter()
            .map(|(k, (cap, _))| (k, *cap))
            .collect();
        let mut dropped = 0;

        // iterate over queued events, newest first
        for event in self.queue.iter_mut().rev() {
            let Some(Event::Input { id: input_id, .. }) = event.as_mut() else {
                continue;
            };
            match remaining.get_mut(input_id) {
                Some(0) => {
                    dropped += 1;
                    if let Some((_, QueuePolicy::Backpressure)) = self.effective_caps.get(input_id)
                    {
                        tracing::error!(
                            "backpressure input `{input_id}` hit hard cap, dropping oldest to prevent OOM"
                        );
                    }
                    *event = None;
                }
                Some(rem) => {
                    *rem = rem.saturating_sub(1);
                }
                None => {
                    tracing::warn!("no queue size known for received operator input `{input_id}`");
                }
            }
        }

        if dropped > 0 {
            tracing::warn!("dropped {dropped} operator inputs because event queue was too full");
        }
    }
}
