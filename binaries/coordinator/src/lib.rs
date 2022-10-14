use crate::run::spawn_dataflow;
use dora_core::topics::{
    StartDataflowResult, StopDataflowResult, ZENOH_CONTROL_DESTROY, ZENOH_CONTROL_START,
    ZENOH_CONTROL_STOP,
};
use dora_node_api::{communication, config::CommunicationConfig};
use eyre::{bail, eyre, WrapErr};
use futures::StreamExt;
use futures_concurrency::stream::Merge;
use run::{await_tasks, SpawnedDataflow};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};
use tokio_stream::wrappers::ReceiverStream;
use uuid::Uuid;
use zenoh::prelude::Sample;

mod control;
mod run;

#[derive(Debug, Clone, clap::Parser)]
#[clap(about = "Dora coordinator")]
pub struct Args {
    #[clap(long)]
    pub runtime: Option<PathBuf>,
    #[clap(long)]
    pub run_dataflow: Option<PathBuf>,
}

pub async fn run(args: Args) -> eyre::Result<()> {
    let Args {
        runtime,
        run_dataflow,
    } = args;

    let runtime_path = runtime.unwrap_or_else(|| {
        std::env::args()
            .next()
            .map(PathBuf::from)
            .unwrap_or_default()
            .with_file_name("dora-runtime")
    });

    match run_dataflow {
        Some(path) => {
            // start the given dataflow directly
            run::run_dataflow(&path, &runtime_path)
                .await
                .wrap_err_with(|| format!("failed to run dataflow at {}", path.display()))?;
        }
        None => {
            // start in daemon mode
            start(&runtime_path).await?;
        }
    }

    Ok(())
}

async fn start(runtime_path: &Path) -> eyre::Result<()> {
    let (dataflow_events_tx, dataflow_events) = tokio::sync::mpsc::channel(2);
    let mut dataflow_events_tx = Some(dataflow_events_tx);
    let dataflow_error_events = ReceiverStream::new(dataflow_events);

    let (control_events, control_events_abort) = futures::stream::abortable(
        control::control_events()
            .await
            .wrap_err("failed to create control events")?,
    );

    let mut events = (dataflow_error_events, control_events).merge();

    let mut running_dataflows = HashMap::new();

    while let Some(event) = events.next().await {
        match event {
            Event::Dataflow { uuid, event } => match event {
                DataflowEvent::Finished { result } => {
                    running_dataflows.remove(&uuid);
                    match result {
                        Ok(()) => {
                            tracing::info!("dataflow `{uuid}` finished successully");
                        }
                        Err(err) => {
                            let err = err.wrap_err(format!("error occured in dataflow `{uuid}`"));
                            tracing::error!("{err:?}");
                        }
                    }
                }
            },

            Event::Control(query) => match query.key_selector().as_str() {
                ZENOH_CONTROL_START => {
                    let dataflow_path = query.value_selector();
                    let result =
                        start_dataflow(Path::new(dataflow_path), runtime_path, &dataflow_events_tx)
                            .await;
                    let reply = match result {
                        Ok((uuid, communication_config)) => {
                            running_dataflows.insert(uuid, communication_config);
                            StartDataflowResult::Ok {
                                uuid: uuid.to_string(),
                            }
                        }
                        Err(err) => {
                            tracing::error!("{err:?}");
                            StartDataflowResult::Error(format!("{err:?}"))
                        }
                    };
                    let _ = query
                        .reply_async(Sample::new("", serde_json::to_string(&reply).unwrap()))
                        .await;
                }
                ZENOH_CONTROL_DESTROY => {
                    tracing::info!("Received stop command");

                    control_events_abort.abort();

                    // ensure that no new dataflows can be started
                    dataflow_events_tx = None;

                    query.reply_async(Sample::new("", "")).await;
                }
                _ => {
                    query.reply_async(Sample::new("error", "invalid")).await;
                }
            },
        }
    }

    tracing::info!("stopped");

    Ok(())
}

async fn start_dataflow(
    path: &Path,
    runtime_path: &Path,
    dataflow_events_tx: &Option<tokio::sync::mpsc::Sender<Event>>,
) -> eyre::Result<(Uuid, CommunicationConfig)> {
    let runtime_path = runtime_path.to_owned();
    let dataflow_events_tx = match dataflow_events_tx {
        Some(channel) => channel.clone(),
        None => bail!("cannot start new dataflow after receiving stop command"),
    };
    let SpawnedDataflow {
        uuid,
        communication_config,
        tasks,
    } = spawn_dataflow(&runtime_path, &path).await?;
    let path = path.to_owned();
    let task = async move {
        let result = await_tasks(tasks)
            .await
            .wrap_err_with(|| format!("failed to run dataflow at {}", path.display()));

        let _ = dataflow_events_tx
            .send(Event::Dataflow {
                uuid,
                event: DataflowEvent::Finished { result },
            })
            .await;
    };
    tokio::spawn(task);
    Ok((uuid, communication_config))
}

enum Event {
    Dataflow { uuid: Uuid, event: DataflowEvent },
    Control(zenoh::queryable::Query),
}

enum DataflowEvent {
    Finished { result: eyre::Result<()> },
}
