use crate::Event;
use communication_layer_pub_sub::{CommunicationLayer, Subscriber};
use dora_core::topics::{
    ZENOH_CONTROL_PREFIX, ZENOH_CONTROL_START_DATAFLOW, ZENOH_CONTROL_STOP_ALL,
};
use eyre::{eyre, WrapErr};
use futures::{Stream, StreamExt};
use std::path::Path;
use tokio_stream::wrappers::ReceiverStream;

pub(crate) fn control_events() -> impl Stream<Item = Event> {
    let (tx, rx) = tokio::sync::mpsc::channel(2);

    tokio::task::spawn_blocking(move || {
        let result = subscribe_control_sync(tx.clone());
        match result {
            Ok(()) => {}
            Err(error) => {
                let _ = tx.blocking_send(Event::ControlChannelError(error));
            }
        }
    });

    ReceiverStream::new(rx).chain(futures::stream::iter(std::iter::from_fn(|| {
        tracing::info!("control channel closed");
        None
    })))
}

fn subscribe_control_sync(tx: tokio::sync::mpsc::Sender<Event>) -> eyre::Result<()> {
    let mut zenoh_control_session =
        communication_layer_pub_sub::zenoh::ZenohCommunicationLayer::init(
            Default::default(),
            ZENOH_CONTROL_PREFIX.into(),
        )
        .map_err(|err| eyre!(err))
        .wrap_err("failed to open zenoh control session")?;

    let start_dataflow = zenoh_control_session
        .subscribe(ZENOH_CONTROL_START_DATAFLOW)
        .map_err(|err| eyre!(err))
        .wrap_err("failed to subscribe to start dataflow topic")?;
    let start_tx = tx.downgrade();
    let _start_dataflow_thread =
        std::thread::spawn(move || start_dataflow_handler(start_dataflow, start_tx));

    let stop_tx = tx;
    let stop = zenoh_control_session
        .subscribe(ZENOH_CONTROL_STOP_ALL)
        .map_err(|err| eyre!(err))
        .wrap_err("failed to subscribe to stop all topic")?;
    let _stop_thread = tokio::task::spawn_blocking(move || {
        stop_handler(stop, stop_tx);
    });

    Ok(())
}

fn stop_handler(mut stop: Box<dyn Subscriber>, stop_tx: tokio::sync::mpsc::Sender<Event>) {
    let send_result = match stop.recv().map_err(|err| eyre!(err)) {
        Ok(_) => stop_tx.blocking_send(Event::Stop),
        Err(err) => stop_tx.blocking_send(Event::ControlChannelError(
            err.wrap_err("failed to receive on control channel"),
        )),
    };
    let _ = send_result;
}

fn start_dataflow_handler(
    mut start_dataflow: Box<dyn Subscriber>,
    start_tx: tokio::sync::mpsc::WeakSender<Event>,
) {
    loop {
        let recv_result = start_dataflow.recv();
        let start_tx = match start_tx.upgrade() {
            Some(tx) => tx,
            None => {
                // control channel was closed after receiving stop message
                break;
            }
        };
        let message = match recv_result {
            Ok(Some(message)) => message,
            Ok(None) => break,
            Err(err) => {
                let send_result = start_tx.blocking_send(Event::ControlChannelError(
                    eyre!(err).wrap_err("failed to receive on start_dataflow topic"),
                ));
                match send_result {
                    Ok(()) => continue,
                    Err(_) => break,
                }
            }
        };
        let data = message.get();
        let path = match std::str::from_utf8(&data) {
            Ok(path) => Path::new(path),
            Err(err) => {
                let send_result = start_tx.blocking_send(Event::ParseError(
                    eyre!(err).wrap_err("failed to parse start_dataflow message"),
                ));
                match send_result {
                    Ok(()) => continue,
                    Err(_) => break,
                }
            }
        };
        let _ = start_tx.blocking_send(Event::StartDataflow {
            path: path.to_owned(),
        });
    }
}
