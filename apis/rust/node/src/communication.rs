use communication_layer_pub_sub::ReceivedSample;
pub use communication_layer_pub_sub::{CommunicationLayer, Publisher, Subscriber};
use dora_message::Metadata;

use crate::{
    config::{CommunicationConfig, DataId, InputMapping, NodeId, OperatorId},
    BoxError,
};
use eyre::Context;
use std::{
    borrow::Cow,
    collections::{BTreeMap, HashSet},
    mem,
    ops::Deref,
    thread,
};

#[doc(hidden)]
pub const STOP_TOPIC: &str = "__dora_rs_internal__operator_stopped";

pub fn init(
    communication_config: &CommunicationConfig,
) -> eyre::Result<Box<dyn CommunicationLayer>> {
    match communication_config {
        #[cfg(feature = "zenoh")]
        CommunicationConfig::Zenoh {
            config: zenoh_config,
            prefix: zenoh_prefix,
        } => {
            let layer = communication_layer_pub_sub::zenoh::ZenohCommunicationLayer::init(
                zenoh_config.deref().clone(),
                zenoh_prefix.clone(),
            )
            .map_err(|err| eyre::eyre!(err))?;

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
            let app_name = format!("{app_name_prefix}-{}", uuid::Uuid::new_v4());
            let instance_name = topic_prefix.clone();
            let layer = communication_layer_pub_sub::iceoryx::IceoryxCommunicationLayer::init(
                app_name,
                "dora".into(),
                instance_name,
            )
            .map_err(|err| eyre::eyre!(err))?;

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

pub fn subscribe_all(
    communication: &mut dyn CommunicationLayer,
    inputs: &BTreeMap<DataId, InputMapping>,
) -> eyre::Result<flume::Receiver<Input>> {
    let (inputs_tx, inputs_rx) = flume::bounded(10);
    for (input, mapping) in inputs {
        let topic = mapping.to_string();
        let mut sub = communication
            .subscribe(&topic)
            .map_err(|err| eyre::eyre!(err))
            .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;

        let input_id = input.to_owned();
        let sender = inputs_tx.clone();
        thread::spawn(move || loop {
            let event = match sub.recv().transpose() {
                None => break,
                Some(Ok(sample)) => {
                    let mut raw: &[u8] = &sample.get();
                    let full_len = raw.len();
                    match Metadata::deserialize(&mut raw).with_context(|| {
                        format!("failed to deserialize metadata for `{input_id}` message")
                    }) {
                        Ok(metadata) => InputEvent::Input(Input {
                            id: input_id.clone(),
                            metadata,
                            data: Data {
                                offset: full_len - raw.len(),
                                sample,
                            },
                        }),
                        Err(err) => InputEvent::ParseMessageError(err.into()),
                    }
                }
                Some(Err(err)) => InputEvent::Error(err),
            };
            match sender.send(event) {
                Ok(()) => {}
                Err(flume::SendError(_)) => break,
            }
        });
    }

    let mut sources: HashSet<_> = inputs
        .values()
        .map(|v| (v.source().to_owned(), v.operator().to_owned()))
        .collect();
    for (source, operator) in &sources {
        let topic = match operator {
            Some(operator) => format!("{source}/{operator}/{STOP_TOPIC}"),
            None => format!("{source}/{STOP_TOPIC}"),
        };
        let mut sub = communication
            .subscribe(&topic)
            .map_err(|err| eyre::eyre!(err))
            .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;

        let source = source.to_owned();
        let operator = operator.clone();
        let sender = inputs_tx.clone();
        thread::spawn(move || loop {
            let event = match sub.recv().transpose() {
                None => break,
                Some(Ok(_)) => InputEvent::SourceClosed {
                    source: source.clone(),
                    operator: operator.clone(),
                },
                Some(Err(err)) => InputEvent::Error(err),
            };
            match sender.send(event) {
                Ok(()) => {}
                Err(flume::SendError(_)) => break,
            }
        });
    }
    mem::drop(inputs_tx);

    let (combined_tx, combined) = flume::bounded(1);
    thread::spawn(move || loop {
        match inputs_rx.recv() {
            Ok(InputEvent::Input(message)) => match combined_tx.send(message) {
                Ok(()) => {}
                Err(flume::SendError(_)) => break,
            },
            Ok(InputEvent::SourceClosed { source, operator }) => {
                sources.remove(&(source, operator));
                if sources.is_empty() {
                    break;
                }
            }
            Ok(InputEvent::ParseMessageError(err)) => {
                tracing::warn!("{err}");
            }
            Ok(InputEvent::Error(err)) => panic!("{err}"),
            Err(_) => break,
        }
    });

    Ok(combined)
}

enum InputEvent {
    Input(Input),
    SourceClosed {
        source: NodeId,
        operator: Option<OperatorId>,
    },
    Error(BoxError),
    ParseMessageError(BoxError),
}

pub struct Input {
    pub id: DataId,
    pub metadata: Metadata<'static>,
    pub data: Data,
}

impl Input {
    pub fn data(&self) -> Cow<[u8]> {
        self.data.get()
    }

    pub fn metadata(&self) -> &Metadata {
        &self.metadata
    }
}

pub struct Data {
    sample: Box<dyn ReceivedSample>,
    offset: usize,
}

impl Data {
    fn get(&self) -> Cow<[u8]> {
        match self.sample.get() {
            std::borrow::Cow::Borrowed(data) => Cow::Borrowed(&data[self.offset..]),
            std::borrow::Cow::Owned(mut data) => {
                // TODO avoid copy caused by moving the remaining elements to the front
                data.drain(..self.offset);
                Cow::Owned(data)
            }
        }
    }
}
