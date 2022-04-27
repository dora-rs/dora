use dora_api::config::DataId;
use dora_common::descriptor::{OperatorConfig, OperatorSource};
use eyre::eyre;
use tokio::sync::mpsc::{self, Sender};

mod shared_lib;

pub struct Operator {
    operator_task: Sender<OperatorInput>,
}

impl Operator {
    pub async fn init(
        operator_config: &OperatorConfig,
        events_tx: Sender<OperatorEvent>,
    ) -> eyre::Result<Self> {
        let (operator_task, operator_rx) = mpsc::channel(10);

        match &operator_config.source {
            OperatorSource::SharedLibrary(path) => {
                let todo =
                    "init shared library operator at `path` with `events_tx` and `operator_rx`";
                eprintln!("WARNING: shared library operators are not supported yet");
            }
            OperatorSource::Python(path) => {
                eprintln!("WARNING: Python operators are not supported yet");
            }
            OperatorSource::Wasm(path) => {
                eprintln!("WARNING: WASM operators are not supported yet");
            }
        }
        Ok(Self { operator_task })
    }

    pub fn handle_input(&mut self, id: DataId, value: Vec<u8>) -> eyre::Result<()> {
        self.operator_task
            .try_send(OperatorInput { id, value })
            .map_err(|err| match err {
                tokio::sync::mpsc::error::TrySendError::Closed(_) => eyre!("operator crashed"),
                tokio::sync::mpsc::error::TrySendError::Full(_) => eyre!("operator queue full"),
            })
    }
}

pub enum OperatorEvent {}

pub struct OperatorInput {
    id: DataId,
    value: Vec<u8>,
}
