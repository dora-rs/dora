use config::{CommunicationConfig, DataId, OperatorConfig};
use eyre::WrapErr;
use futures::{stream::FuturesUnordered, SinkExt, StreamExt};
use futures_concurrency::Merge;
use std::collections::HashSet;
use zenoh::prelude::SplitBuffer;

pub mod config;

const STOP_TOPIC: &str = "__dora_rs_internal__operator_stopped";

pub struct DoraOperator {
    operator_config: OperatorConfig,
    communication_config: CommunicationConfig,
    zenoh: zenoh::Session,
}

impl DoraOperator {
    pub async fn init_from_args() -> eyre::Result<Self> {
        let operator_config = {
            let raw = std::env::var("OPERATOR_CONFIG")
                .wrap_err("env variable OPERATOR_CONFIG must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
        };
        let communication_config = {
            let raw = std::env::var("COMMUNICATION_CONFIG")
                .wrap_err("env variable COMMUNICATION_CONFIG must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize communication config")?
        };
        Self::init(operator_config, communication_config).await
    }

    pub async fn init(
        operator_config: OperatorConfig,
        communication_config: CommunicationConfig,
    ) -> eyre::Result<Self> {
        let zenoh = zenoh::open(communication_config.zenoh_config.clone())
            .await
            .map_err(BoxError)
            .wrap_err("failed to create zenoh session")?;

        Ok(Self {
            operator_config,
            communication_config,
            zenoh,
        })
    }

    pub async fn inputs(&self) -> eyre::Result<impl futures::Stream<Item = Input> + '_> {
        let prefix = &self.communication_config.zenoh_prefix;

        let mut streams = Vec::new();
        for (input, config::InputMapping { source, output }) in &self.operator_config.inputs {
            let topic = format!("{prefix}/{source}/{output}");
            let sub = self
                .zenoh
                .subscribe(&topic)
                .await
                .map_err(BoxError)
                .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;
            streams.push(sub.map(|s| Input {
                id: input.clone(),
                data: s.value.payload.contiguous().into_owned(),
            }))
        }

        let stop_messages = FuturesUnordered::new();
        let sources: HashSet<_> = self
            .operator_config
            .inputs
            .values()
            .map(|v| &v.source)
            .collect();
        for source in &sources {
            let topic = format!("{prefix}/{source}/{STOP_TOPIC}");
            let sub = self
                .zenoh
                .subscribe(&topic)
                .await
                .map_err(BoxError)
                .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;
            stop_messages.push(sub.into_future());
        }
        let finished = Box::pin(stop_messages.all(|_| async { true }));

        Ok(streams.merge().take_until(finished))
    }

    pub async fn send_output(&self, output_id: &DataId, data: &[u8]) -> eyre::Result<()> {
        if !self.operator_config.outputs.contains(output_id) {
            eyre::bail!("unknown output");
        }

        let prefix = &self.communication_config.zenoh_prefix;
        let self_id = &self.operator_config.id;

        let topic = format!("{prefix}/{self_id}/{output_id}");
        let mut publisher = self
            .zenoh
            .publish(&topic)
            .await
            .map_err(BoxError)
            .wrap_err_with(|| format!("failed to create publisher for output {output_id}"))?;
        SinkExt::send(&mut publisher, data)
            .await
            .map_err(BoxError)
            .wrap_err_with(|| format!("failed to send data for output {output_id}"))?;
        Ok(())
    }
}

impl Drop for DoraOperator {
    fn drop(&mut self) {
        use zenoh::prelude::ZFuture;

        let prefix = &self.communication_config.zenoh_prefix;
        let self_id = &self.operator_config.id;
        let topic = format!("{prefix}/{self_id}/{STOP_TOPIC}");
        let result = self
            .zenoh
            .put(&topic, Vec::new())
            .wait()
            .map_err(BoxError)
            .wrap_err_with(|| format!("failed to send stop message for source `{self_id}`"));
        if let Err(err) = result {
            tracing::error!("{err}")
        }
    }
}

pub struct Input {
    pub id: DataId,
    pub data: Vec<u8>,
}

struct BoxError(Box<dyn std::error::Error + Send + Sync + 'static>);

impl std::fmt::Debug for BoxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Debug::fmt(&self.0, f)
    }
}

impl std::fmt::Display for BoxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(&self.0, f)
    }
}

impl std::error::Error for BoxError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        self.0.source()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run<F, O>(future: F) -> O
    where
        F: std::future::Future<Output = O>,
    {
        let rt = tokio::runtime::Builder::new_current_thread()
            .build()
            .unwrap();
        rt.block_on(future)
    }

    #[test]
    fn no_op_operator() {
        let operator_config = config::OperatorConfig {
            id: uuid::Uuid::new_v4().to_string().into(),
            inputs: Default::default(),
            outputs: Default::default(),
        };
        let communication_config = config::CommunicationConfig {
            zenoh_config: Default::default(),
            zenoh_prefix: format!("/{}", uuid::Uuid::new_v4()),
        };

        run(async {
            let operator = DoraOperator::init(operator_config, communication_config)
                .await
                .unwrap();
            let mut inputs = operator.inputs().await.unwrap();
            assert!(inputs.next().await.is_none());
        });
    }
}
