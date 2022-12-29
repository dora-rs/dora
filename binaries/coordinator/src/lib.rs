use crate::{
    run::spawn_dataflow,
    tcp_utils::{tcp_receive, tcp_send},
};
use control::ControlEvent;
use dora_core::{
    config::CommunicationConfig,
    coordinator_messages::RegisterResult,
    daemon_messages::{DaemonCoordinatorEvent, DaemonCoordinatorReply},
    topics::{
        control_socket_addr, ControlRequest, DataflowId, ListDataflowResult, StartDataflowResult,
        StopDataflowResult, DORA_COORDINATOR_PORT_DEFAULT,
    },
};
use eyre::{bail, eyre, ContextCompat, WrapErr};
use futures::StreamExt;
use futures_concurrency::stream::Merge;
use run::SpawnedDataflow;
use std::{
    collections::{BTreeSet, HashMap},
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
}

pub async fn run(args: Args) -> eyre::Result<()> {
    let Args { runtime } = args;

    let runtime_path = runtime.unwrap_or_else(|| {
        std::env::args()
            .next()
            .map(PathBuf::from)
            .unwrap_or_default()
            .with_file_name("dora-runtime")
    });

    // start in daemon mode
    start(&runtime_path).await?;

    Ok(())
}

async fn start(runtime_path: &Path) -> eyre::Result<()> {
    let listener = listener::create_listener(DORA_COORDINATOR_PORT_DEFAULT).await?;
    let new_daemon_connections = TcpListenerStream::new(listener).map(|c| {
        c.map(Event::NewDaemonConnection)
            .wrap_err("failed to open connection")
            .unwrap_or_else(Event::DaemonConnectError)
    });

    let (daemon_events_tx, daemon_events) = tokio::sync::mpsc::channel(2);
    let daemon_events = ReceiverStream::new(daemon_events);

    let (control_events, control_events_abort) = futures::stream::abortable(
        control::control_events(control_socket_addr())
            .await
            .wrap_err("failed to create control events")?,
    );

    let mut events = (new_daemon_connections, daemon_events, control_events).merge();

    let mut running_dataflows: HashMap<Uuid, RunningDataflow> = HashMap::new();
    let mut daemon_connections: HashMap<_, TcpStream> = HashMap::new();

    while let Some(event) = events.next().await {
        tracing::trace!("Handling event {event:?}");
        match event {
            Event::NewDaemonConnection(connection) => {
                let events_tx = daemon_events_tx.clone();
                tokio::spawn(listener::handle_connection(connection, events_tx));
            }
            Event::DaemonConnectError(err) => {
                tracing::warn!("{:?}", err.wrap_err("failed to connect to dora-daemon"));
            }
            Event::Daemon(event) => {
                match event {
                    DaemonEvent::Register {
                        machine_id,
                        mut connection,
                    } => {
                        let reply = RegisterResult::Ok;
                        match tcp_send(&mut connection, &serde_json::to_vec(&reply)?).await {
                            Ok(()) => {
                                let previous =
                                    daemon_connections.insert(machine_id.clone(), connection);
                                if let Some(_previous) = previous {
                                    tracing::info!("closing previous connection `{machine_id}` on new register");
                                }
                            }
                            Err(err) => {
                                tracing::warn!("failed to register daemon connection for machine `{machine_id}`: {err}");
                            }
                        }
                    }
                }
            }
            Event::Dataflow { uuid, event } => match event {
                DataflowEvent::DataflowFinishedOnMachine { machine_id, result } => {
                    match running_dataflows.entry(uuid) {
                        std::collections::hash_map::Entry::Occupied(mut entry) => {
                            entry.get_mut().machines.remove(&machine_id);
                            match result {
                                Ok(()) => {
                                    tracing::info!("dataflow `{uuid}` finished successfully on machine `{machine_id}`");
                                }
                                Err(err) => {
                                    let err =
                                        err.wrap_err(format!("error occured in dataflow `{uuid}` on machine `{machine_id}`"));
                                    tracing::error!("{err:?}");
                                }
                            }
                            if entry.get_mut().machines.is_empty() {
                                entry.remove();
                                tracing::info!("dataflow `{uuid}` finished");
                            }
                        }
                        std::collections::hash_map::Entry::Vacant(_) => {
                            tracing::warn!("dataflow not running on DataflowFinishedOnMachine");
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
                                    &mut daemon_connections,
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
                                stop_dataflow(
                                    &running_dataflows,
                                    dataflow_uuid,
                                    &mut daemon_connections,
                                )
                                .await?;
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

                                stop_dataflow(
                                    &running_dataflows,
                                    dataflow_uuid,
                                    &mut daemon_connections,
                                )
                                .await?;
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

                            // stop all running dataflows
                            for &uuid in running_dataflows.keys() {
                                stop_dataflow(&running_dataflows, uuid, &mut daemon_connections)
                                    .await?;
                            }

                            b"ok".as_slice().into()
                        }
                        ControlRequest::List => {
                            let mut dataflows: Vec<_> = running_dataflows.values().collect();
                            dataflows.sort_by_key(|d| (&d.name, d.uuid));

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
                        ControlRequest::DaemonConnected => {
                            let running = !daemon_connections.is_empty();
                            serde_json::to_vec(&running).unwrap()
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
    /// The IDs of the machines that the dataflow is running on.
    machines: BTreeSet<String>,
}

impl PartialEq for RunningDataflow {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.uuid == other.uuid && self.machines == other.machines
    }
}

impl Eq for RunningDataflow {}

async fn stop_dataflow(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    uuid: Uuid,
    daemon_connections: &mut HashMap<String, TcpStream>,
) -> eyre::Result<()> {
    let Some(dataflow) = running_dataflows.get(&uuid) else {
        bail!("No running dataflow found with UUID `{uuid}`")
    };
    let message = serde_json::to_vec(&DaemonCoordinatorEvent::StopDataflow { dataflow_id: uuid })?;

    for machine_id in &dataflow.machines {
        let daemon_connection = daemon_connections
            .get_mut(machine_id)
            .wrap_err("no daemon connection")?; // TODO: take from dataflow spec
        tcp_send(daemon_connection, &message)
            .await
            .wrap_err("failed to send stop message to daemon")?;

        // wait for reply
        let reply_raw = tcp_receive(daemon_connection)
            .await
            .wrap_err("failed to receive stop reply from daemon")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize stop reply from daemon")?
        {
            DaemonCoordinatorReply::StopResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to stop dataflow")?,
            _ => bail!("unexpected reply"),
        }
    }
    tracing::info!("successfully stoped dataflow `{uuid}`");

    Ok(())
}

async fn start_dataflow(
    path: &Path,
    name: Option<String>,
    runtime_path: &Path,
    daemon_connections: &mut HashMap<String, TcpStream>,
) -> eyre::Result<RunningDataflow> {
    let runtime_path = runtime_path.to_owned();

    let SpawnedDataflow {
        uuid,
        communication_config,
        machines,
    } = spawn_dataflow(&runtime_path, path, daemon_connections).await?;
    Ok(RunningDataflow {
        uuid,
        name,
        communication_config,
        machines,
    })
}

#[derive(Debug)]
pub enum Event {
    NewDaemonConnection(TcpStream),
    DaemonConnectError(eyre::Report),
    Dataflow { uuid: Uuid, event: DataflowEvent },
    Control(ControlEvent),
    Daemon(DaemonEvent),
}

#[derive(Debug)]
pub enum DataflowEvent {
    DataflowFinishedOnMachine {
        machine_id: String,
        result: eyre::Result<()>,
    },
}

#[derive(Debug)]
pub enum DaemonEvent {
    Register {
        machine_id: String,
        connection: TcpStream,
    },
}
