use crate::{
    config::{CommunicationConfig, DataId, InputMapping},
    BoxError,
};
use eyre::{eyre, Context};
pub use flume::Receiver;
use std::{
    collections::{BTreeMap, HashMap},
    mem, thread,
};

#[doc(hidden)]
pub const STOP_TOPIC: &str = "__dora_rs_internal__operator_stopped";

#[cfg(all(unix, feature = "iceoryx"))]
pub mod iceoryx;
#[cfg(feature = "zenoh")]
pub mod zenoh;

pub fn init(
    communication_config: &CommunicationConfig,
) -> eyre::Result<Box<dyn CommunicationLayer>> {
    match communication_config {
        #[cfg(feature = "zenoh")]
        CommunicationConfig::Zenoh {
            config: zenoh_config,
            prefix: zenoh_prefix,
        } => {
            let layer =
                zenoh::ZenohCommunicationLayer::init(zenoh_config.clone(), zenoh_prefix.clone())?;

            Ok(Box::new(layer))
        }
        #[cfg(not(feature = "zenoh"))]
        CommunicationConfig::Zenoh { .. } => {
            eyre::bail!(
                "cannot parse zenoh config because the compile-time `zenoh` feature \
                of `dora-node-api` was disabled"
            )
        }
        #[cfg(all(unix, feature = "iceoryx"))]
        CommunicationConfig::Iceoryx {
            app_name_prefix,
            topic_prefix,
        } => {
            let app_name_prefix = app_name_prefix.clone();
            let topic_prefix = topic_prefix.clone();
            let layer = iceoryx::IceoryxCommunicationLayer::init(app_name_prefix, topic_prefix)?;

            Ok(Box::new(layer))
        }
        #[cfg(not(all(unix, feature = "iceoryx")))]
        CommunicationConfig::Iceoryx { .. } => {
            eyre::bail!(
                "cannot parse iceoryx config because the compile-time `iceoryx` feature \
                of `dora-node-api` was disabled"
            )
        }
    }
}

pub trait CommunicationLayer: Send + Sync {
    fn publisher(&mut self, topic: &str) -> Result<Box<dyn Publisher>, BoxError>;

    fn subscribe(&mut self, topic: &str) -> Result<Box<dyn Subscriber>, BoxError>;

    fn subscribe_all(
        &mut self,
        inputs: &BTreeMap<DataId, InputMapping>,
    ) -> eyre::Result<flume::Receiver<Input>> {
        let (inputs_tx, inputs_rx) = flume::bounded(10);
        for (input, mapping) in inputs {
            let topic = mapping.to_string();
            let mut sub = self
                .subscribe(&topic)
                .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;

            let input_id = input.to_owned();
            let sender = inputs_tx.clone();
            thread::spawn(move || loop {
                match sub.recv().transpose() {
                    None => break,
                    Some(value) => {
                        let input = value.map(|data| Input {
                            id: input_id.clone(),
                            data,
                        });
                        match sender.send(input) {
                            Ok(()) => {}
                            Err(flume::SendError(_)) => break,
                        }
                    }
                }
            });
        }
        mem::drop(inputs_tx);

        let (stop_tx, stop_rx) = flume::bounded(10);
        let mut sources: HashMap<_, _> = inputs
            .values()
            .map(|v| (v.source().to_owned(), v.operator().to_owned()))
            .collect();
        for (source, operator) in &sources {
            let topic = match operator {
                Some(operator) => format!("{source}/{operator}/{STOP_TOPIC}"),
                None => format!("{source}/{STOP_TOPIC}"),
            };
            let mut sub = self
                .subscribe(&topic)
                .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;

            let source_id = source.to_owned();
            let sender = stop_tx.clone();
            thread::spawn(move || loop {
                match sub.recv().transpose() {
                    None => break,
                    Some(value) => {
                        let input = value.map(|_| source_id.clone());
                        match sender.send(input) {
                            Ok(()) => {}
                            Err(flume::SendError(_)) => break,
                        }
                    }
                }
            });
        }
        mem::drop(stop_tx);

        let (combined_tx, combined) = flume::bounded(1);
        thread::spawn(move || loop {
            let selector = flume::Selector::new()
                .recv(&inputs_rx, |v| match v {
                    Ok(Ok(value)) => InputEvent::Input(value),
                    Ok(Err(err)) => InputEvent::Error(err),
                    Err(flume::RecvError::Disconnected) => InputEvent::Error(BoxError(
                        eyre!("input stream was disconnected unexpectedly").into(),
                    )),
                })
                .recv(&stop_rx, |v| match v {
                    Ok(Ok(stopped_source)) => {
                        sources.remove(&stopped_source);
                        InputEvent::InputClosed {
                            number_of_remaining_sources: sources.len(),
                        }
                    }
                    Ok(Err(err)) => InputEvent::Error(err),
                    Err(flume::RecvError::Disconnected) => InputEvent::Error(BoxError(
                        eyre!("stop stream was disconnected unexpectedly").into(),
                    )),
                });
            match selector.wait() {
                InputEvent::Input(input) => match combined_tx.send(input) {
                    Ok(()) => {}
                    Err(flume::SendError(_)) => break,
                },
                InputEvent::InputClosed {
                    number_of_remaining_sources,
                } => {
                    if number_of_remaining_sources == 0 {
                        break;
                    }
                }
                InputEvent::Error(err) => panic!("{err}"),
            }
        });

        Ok(combined)
    }
}

pub trait Publisher: Send + Sync {
    fn publish(&self, data: &[u8]) -> Result<(), BoxError>;

    fn boxed_clone(&self) -> Box<dyn Publisher>;
}

pub trait Subscriber: Send + Sync {
    fn recv(&mut self) -> Result<Option<Vec<u8>>, BoxError>;
}

enum InputEvent {
    Input(Input),
    InputClosed { number_of_remaining_sources: usize },
    Error(BoxError),
}

#[derive(Debug)]
pub struct Input {
    pub id: DataId,
    pub data: Vec<u8>,
}
