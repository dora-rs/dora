use dora_common::descriptor::{OperatorConfig, OperatorSource};
use dora_node_api::config::DataId;
use eyre::{eyre, Context};
use std::any::Any;
use tokio::sync::mpsc::{self, Sender};

mod shared_lib;

pub struct Operator {
    operator_task: Sender<OperatorInput>,
    config: OperatorConfig,
}

impl Operator {
    pub async fn init(
        operator_config: OperatorConfig,
        events_tx: Sender<OperatorEvent>,
    ) -> eyre::Result<Self> {
        let (operator_task, operator_rx) = mpsc::channel(10);

        match &operator_config.source {
            OperatorSource::SharedLibrary(path) => {
                shared_lib::spawn(path, events_tx, operator_rx).wrap_err_with(|| {
                    format!(
                        "failed ot spawn shared library operator for {}",
                        operator_config.id
                    )
                })?;
            }
            OperatorSource::Python(path) => {
                eprintln!("WARNING: Python operators are not supported yet");
            }
            OperatorSource::Wasm(path) => {
                eprintln!("WARNING: WASM operators are not supported yet");
            }
        }
        Ok(Self {
            operator_task,
            config: operator_config,
        })
    }

    pub fn handle_input(&mut self, id: DataId, value: Vec<u8>) -> eyre::Result<()> {
        self.operator_task
            .try_send(OperatorInput { id, value })
            .map_err(|err| match err {
                tokio::sync::mpsc::error::TrySendError::Closed(_) => eyre!("operator crashed"),
                tokio::sync::mpsc::error::TrySendError::Full(_) => eyre!("operator queue full"),
            })
    }

    /// Get a reference to the operator's config.
    #[must_use]
    pub fn config(&self) -> &OperatorConfig {
        &self.config
    }
}

pub enum OperatorEvent {
    Output { id: DataId, value: Vec<u8> },
    Error(eyre::Error),
    Panic(Box<dyn Any + Send>),
    EndOfInput,
}

pub struct OperatorInput {
    id: DataId,
    value: Vec<u8>,
}
