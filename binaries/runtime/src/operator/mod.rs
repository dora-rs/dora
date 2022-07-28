use dora_core::descriptor::{OperatorDefinition, OperatorSource};
use dora_node_api::config::DataId;
use eyre::{eyre, Context};
use std::any::Any;
use tokio::sync::mpsc::{self, Sender};

mod python;
mod shared_lib;

pub struct Operator {
    operator_task: Option<Sender<OperatorInput>>,
    definition: OperatorDefinition,
}

impl Operator {
    pub async fn init(
        operator_definition: OperatorDefinition,
        events_tx: Sender<OperatorEvent>,
    ) -> eyre::Result<Self> {
        let (operator_task, operator_rx) = mpsc::channel(10);

        match &operator_definition.config.source {
            OperatorSource::SharedLibrary(path) => {
                shared_lib::spawn(path, events_tx, operator_rx).wrap_err_with(|| {
                    format!(
                        "failed ot spawn shared library operator for {}",
                        operator_definition.id
                    )
                })?;
            }
            OperatorSource::Python(path) => {
                python::spawn(path, events_tx, operator_rx).wrap_err_with(|| {
                    format!(
                        "failed ot spawn Python operator for {}",
                        operator_definition.id
                    )
                })?;
            }
            OperatorSource::Wasm(path) => {
                eprintln!("WARNING: WASM operators are not supported yet");
            }
        }
        Ok(Self {
            operator_task: Some(operator_task),
            definition: operator_definition,
        })
    }

    pub fn handle_input(&mut self, id: DataId, value: Vec<u8>) -> eyre::Result<()> {
        self.operator_task
            .as_mut()
            .ok_or_else(|| {
                eyre!(
                    "input channel for {} was already closed",
                    self.definition.id
                )
            })?
            .try_send(OperatorInput { id, value })
            .map_err(|err| match err {
                tokio::sync::mpsc::error::TrySendError::Closed(_) => eyre!("operator crashed"),
                tokio::sync::mpsc::error::TrySendError::Full(_) => eyre!("operator queue full"),
            })
    }

    pub fn close_input_stream(&mut self) {
        self.operator_task = None;
    }

    /// Get a reference to the operator's definition.
    #[must_use]
    pub fn definition(&self) -> &OperatorDefinition {
        &self.definition
    }
}

pub enum OperatorEvent {
    Output { id: DataId, value: Vec<u8> },
    Error(eyre::Error),
    Panic(Box<dyn Any + Send>),
    Finished,
}

pub struct OperatorInput {
    id: DataId,
    value: Vec<u8>,
}
