use crate::{run::spawn_dataflow, tcp_utils::tcp_send};
use control::ControlEvent;
use dora_core::{
    config::CommunicationConfig,
    coordinator_messages::RegisterResult,
    topics::{
        control_socket_addr, ControlRequest, DataflowId, ListDataflowResult, StartDataflowResult,
        StopDataflowResult, DORA_COORDINATOR_PORT_DEFAULT,
    },
};
use eyre::{bail, WrapErr};
use futures::StreamExt;
use futures_concurrency::stream::Merge;
use run::{await_tasks, SpawnedDataflow};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};
use tokio::net::TcpStream;
use tokio_stream::wrappers::{ReceiverStream, TcpListenerStream};
use uuid::Uuid;

mod control;
mod listener;
mod run;
mod tcp_utils;

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
    let listener = listener::create_listener(DORA_COORDINATOR_PORT_DEFAULT).await?;
    let new_daemon_connections = TcpListenerStream::new(listener).map(|c| {
        c.map(Event::NewDaemonConnection)
            .wrap_err("failed to open connection")
            .unwrap_or_else(Event::DaemonConnectError)
    });

    let (dataflow_events_tx, dataflow_events) = tokio::sync::mpsc::channel(2);
    let mut dataflow_events_tx = Some(dataflow_events_tx);
    let dataflow_events = ReceiverStream::new(dataflow_events);

    let (daemon_events_tx, daemon_events) = tokio::sync::mpsc::channel(2);
    let daemon_events = ReceiverStream::new(daemon_events);

    let (control_events, control_events_abort) = futures::stream::abortable(
        control::control_events(control_socket_addr())
            .await
            .wrap_err("failed to create control events")?,
    );

    let mut events = (
        new_daemon_connections,
        daemon_events,
        dataflow_events,
        control_events,
    )
        .merge();

    let mut running_dataflows = HashMap::new();
    let mut daemon_connections = HashMap::new();

    while let Some(event) = events.next().await {
        match event {
            Event::NewDaemonConnection(connection) => {
                let events_tx = daemon_events_tx.clone();
                tokio::spawn(listener::handle_connection(connection, events_tx));
            }
            Event::DaemonConnectError(err) => {
                tracing::warn!("{:?}", err.wrap_err("failed to connect to dora-daemon"));
            }
            Event::Daemon(event) => match event {
                DaemonEvent::Register {
                    machine_id,
                    mut connection,
                } => match daemon_connections.entry(machine_id) {
                    std::collections::hash_map::Entry::Vacant(entry) => {
                        let reply = RegisterResult::Ok;
                        if tcp_send(&mut connection, &serde_json::to_vec(&reply)?)
                            .await
                            .is_ok()
                        {
                            entry.insert(connection);
                        }
                    }
                    std::collections::hash_map::Entry::Occupied(entry) => {
                        let reply = RegisterResult::Err(format!(
                            "there is already a daemon connection for machine `{}`",
                            entry.key()
                        ));
                        let _ = tcp_send(&mut connection, &serde_json::to_vec(&reply)?).await;
                    }
                },
            },
            Event::Dataflow { uuid, event } => match event {
                DataflowEvent::Finished { result } => {
                    running_dataflows.remove(&uuid);
                    match result {
                        Ok(()) => {
                            tracing::info!("dataflow `{uuid}` finished successfully");
                        }
                        Err(err) => {
                            let err = err.wrap_err(format!("error occured in dataflow `{uuid}`"));
                            tracing::error!("{err:?}");
                        }
                    }
                }
            },

            Event::Control(event) => match event {
                ControlEvent::IncomingRequest {
                    request,
                    reply_sender,
                } => {
                    let reply = match request {
                        ControlRequest::Start {
                            dataflow_path,
                            name,
                        } => {
                            let inner = async {
                                if let Some(name) = name.as_deref() {
                                    // check that name is unique
                                    if running_dataflows
                                        .values()
                                        .any(|d: &RunningDataflow| d.name.as_deref() == Some(name))
                                    {
                                        bail!("there is already a running dataflow with name `{name}`");
                                    }
                                }
                                let dataflow = start_dataflow(
                                    &dataflow_path,
                                    name,
                                    runtime_path,
                                    &dataflow_events_tx,
                                )
                                .await?;
                                Ok(dataflow)
                            };
                            let reply = match inner.await {
                                Ok(dataflow) => {
                                    let uuid = dataflow.uuid;
                                    running_dataflows.insert(uuid, dataflow);
                                    StartDataflowResult::Ok { uuid }
                                }
                                Err(err) => {
                                    tracing::error!("{err:?}");
                                    StartDataflowResult::Error(format!("{err:?}"))
                                }
                            };
                            serde_json::to_vec(&reply).unwrap()
                        }
                        ControlRequest::Stop { dataflow_uuid } => {
                            let stop = async {
                                stop_dataflow(&running_dataflows, dataflow_uuid).await?;
                                Result::<_, eyre::Report>::Ok(())
                            };
                            let reply = match stop.await {
                                Ok(()) => StopDataflowResult::Ok,
                                Err(err) => StopDataflowResult::Error(format!("{err:?}")),
                            };

                            serde_json::to_vec(&reply).unwrap()
                        }
                        ControlRequest::StopByName { name } => {
                            let stop = async {
                                let uuids: Vec<_> = running_dataflows
                                    .iter()
                                    .filter(|(_, v)| v.name.as_deref() == Some(name.as_str()))
                                    .map(|(k, _)| k)
                                    .copied()
                                    .collect();
                                let dataflow_uuid = if uuids.is_empty() {
                                    bail!("no running dataflow with name `{name}`");
                                } else if let [uuid] = uuids.as_slice() {
                                    *uuid
                                } else {
                                    bail!("multiple dataflows found with name `{name}`");
                                };

                                stop_dataflow(&running_dataflows, dataflow_uuid).await?;
                                Result::<_, eyre::Report>::Ok(())
                            };
                            let reply = match stop.await {
                                Ok(()) => StopDataflowResult::Ok,
                                Err(err) => StopDataflowResult::Error(format!("{err:?}")),
                            };

                            serde_json::to_vec(&reply).unwrap()
                        }
                        ControlRequest::Destroy => {
                            tracing::info!("Received destroy command");

                            control_events_abort.abort();

                            // ensure that no new dataflows can be started
                            dataflow_events_tx = None;

                            // stop all running dataflows
                            for &uuid in running_dataflows.keys() {
                                stop_dataflow(&running_dataflows, uuid).await?;
                            }

                            b"ok".as_slice().into()
                        }
                        ControlRequest::List => {
                            let mut dataflows: Vec<_> = running_dataflows.values().collect();
                            dataflows.sort();

                            let reply = ListDataflowResult::Ok {
                                dataflows: dataflows
                                    .into_iter()
                                    .map(|d| DataflowId {
                                        uuid: d.uuid,
                                        name: d.name.clone(),
                                    })
                                    .collect(),
                            };

                            serde_json::to_vec(&reply).unwrap()
                        }
                    };
                    let _ = reply_sender.send(reply);
                }
                ControlEvent::Error(err) => tracing::error!("{err:?}"),
            },
        }
    }

    tracing::info!("stopped");

    Ok(())
}

struct RunningDataflow {
    name: Option<String>,
    uuid: Uuid,
    communication_config: CommunicationConfig,
}

impl PartialEq for RunningDataflow {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.uuid == other.uuid
    }
}

impl Eq for RunningDataflow {}

impl PartialOrd for RunningDataflow {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        match self.name.partial_cmp(&other.name) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        self.uuid.partial_cmp(&other.uuid)
    }
}

impl Ord for RunningDataflow {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match self.name.cmp(&other.name) {
            core::cmp::Ordering::Equal => {}
            ord => return ord,
        }
        self.uuid.cmp(&other.uuid)
    }
}

async fn stop_dataflow(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    uuid: Uuid,
) -> eyre::Result<()> {
    let communication_config = match running_dataflows.get(&uuid) {
        Some(dataflow) => dataflow.communication_config.clone(),
        None => bail!("No running dataflow found with UUID `{uuid}`"),
    };

    todo!();
    // let mut communication =
    //     tokio::task::spawn_blocking(move || communication::init(&communication_config))
    //         .await
    //         .wrap_err("failed to join communication layer init task")?
    //         .wrap_err("failed to init communication layer")?;
    // tracing::info!("sending stop message to dataflow `{uuid}`");
    // let manual_stop_publisher = manual_stop_publisher(communication.as_mut())?;
    // tokio::task::spawn_blocking(move || manual_stop_publisher())
    //     .await
    //     .wrap_err("failed to join stop publish task")?
    //     .map_err(|err| eyre!(err))
    //     .wrap_err("failed to send stop message")?;
    Ok(())
}

async fn start_dataflow(
    path: &Path,
    name: Option<String>,
    runtime_path: &Path,
    dataflow_events_tx: &Option<tokio::sync::mpsc::Sender<Event>>,
) -> eyre::Result<RunningDataflow> {
    // TODO: send Spawn message to daemon

    let runtime_path = runtime_path.to_owned();
    let dataflow_events_tx = match dataflow_events_tx {
        Some(channel) => channel.clone(),
        None => bail!("cannot start new dataflow after receiving stop command"),
    };
    let SpawnedDataflow {
        uuid,
        communication_config,
        tasks,
    } = spawn_dataflow(&runtime_path, path).await?;
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
    Ok(RunningDataflow {
        uuid,
        name,
        communication_config,
    })
}

pub enum Event {
    NewDaemonConnection(TcpStream),
    DaemonConnectError(eyre::Report),
    Dataflow { uuid: Uuid, event: DataflowEvent },
    Control(ControlEvent),
    Daemon(DaemonEvent),
}

pub enum DataflowEvent {
    Finished { result: eyre::Result<()> },
}

pub enum DaemonEvent {
    Register {
        machine_id: String,
        connection: TcpStream,
    },
}
