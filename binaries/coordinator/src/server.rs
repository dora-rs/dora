use std::{collections::BTreeSet, sync::Arc, time::Duration};

use dora_core::config::{NodeId, OperatorId};
use dora_message::{
    BuildId,
    cli_to_coordinator::{BuildRequest, CliControl, StartRequest},
    common::DaemonId,
    coordinator_to_cli::{
        CheckDataflowReply, DataflowIdAndName, DataflowInfo, DataflowList, DataflowListEntry,
        DataflowResult, DataflowStatus, NodeInfo, NodeMetricsInfo, StopDataflowReply, VersionInfo,
    },
    tarpc::context::Context,
};
use eyre::eyre;
use petname::petname;
use tokio::sync::oneshot;
use uuid::Uuid;

use crate::{
    build_dataflow, dataflow_result, handle_destroy, reload_dataflow, resolve_name, retrieve_logs,
    start_dataflow, state::CoordinatorState, stop_dataflow,
};

/// Helper to convert eyre errors to strings for tarpc.
fn err_to_string(err: eyre::Report) -> String {
    format!("{err:?}")
}

/// Shared logic for stopping a dataflow by UUID.
async fn stop_dataflow_impl(
    state: &CoordinatorState,
    dataflow_uuid: Uuid,
    grace_duration: Option<Duration>,
    force: bool,
) -> Result<StopDataflowReply, String> {
    if let Some(result) = state.dataflow_results.get(&dataflow_uuid) {
        let reply = StopDataflowReply {
            uuid: dataflow_uuid,
            result: dataflow_result(result.value(), dataflow_uuid, &state.clock),
        };
        return Ok(reply);
    }

    let (tx, rx) = oneshot::channel();
    let mut dataflow = stop_dataflow(
        &state.running_dataflows,
        dataflow_uuid,
        &state.daemon_connections,
        state.clock.new_timestamp(),
        grace_duration,
        force,
    )
    .await
    .map_err(err_to_string)?;

    dataflow.stop_reply_senders.push(tx);
    // Drop the DashMap RefMut so the event loop can access the entry.
    drop(dataflow);

    match rx.await {
        Ok(Ok(reply)) => Ok(reply),
        Ok(Err(err)) => Err(err_to_string(err)),
        Err(_) => Err("coordinator dropped the reply sender".to_string()),
    }
}

#[derive(Clone)]
pub(crate) struct ControlServer {
    pub(crate) state: Arc<CoordinatorState>,
    pub(crate) client_ip: Option<std::net::IpAddr>,
}

impl CliControl for ControlServer {
    async fn build(self, _context: Context, request: BuildRequest) -> Result<BuildId, String> {
        // assign a random build id
        let build_id = BuildId::generate();

        let result = build_dataflow(
            request,
            build_id,
            &self.state.clock,
            &self.state.daemon_connections,
        )
        .await;
        match result {
            Ok(build) => {
                self.state.running_builds.insert(build_id, build);
                Ok(build_id)
            }
            Err(err) => Err(err_to_string(err)),
        }
    }

    async fn wait_for_build(self, _context: Context, build_id: BuildId) -> Result<(), String> {
        let (tx, rx) = oneshot::channel();
        if let Some(mut build) = self.state.running_builds.get_mut(&build_id) {
            build.build_result.register(tx);
        } else if let Some(mut result) = self.state.finished_builds.get_mut(&build_id) {
            result.register(tx);
        } else {
            return Err(format!("unknown build id {build_id}"));
        }

        match rx.await {
            Ok(Ok(build_finished)) => build_finished.result,
            Ok(Err(err)) => Err(err_to_string(err)),
            Err(_) => Err("coordinator dropped the reply sender".to_string()),
        }
    }

    async fn start(self, _context: Context, request: StartRequest) -> Result<Uuid, String> {
        let StartRequest {
            build_id,
            session_id,
            dataflow,
            name,
            local_working_dir,
            uv,
            write_events_to,
        } = request;

        let name = name.or_else(|| petname(2, "-"));

        if let Some(name) = name.as_deref() {
            // check that name is unique
            if self
                .state
                .running_dataflows
                .iter()
                .any(|d| d.value().name.as_deref() == Some(name))
            {
                return Err(format!(
                    "there is already a running dataflow with name `{name}`"
                ));
            }
        }
        let uuid = start_dataflow(
            build_id,
            session_id,
            dataflow,
            local_working_dir,
            name,
            &self.state.daemon_connections,
            &self.state.clock,
            &self.state.running_dataflows,
            uv,
            write_events_to,
        )
        .await
        .map_err(err_to_string)?;

        Ok(uuid)
    }

    async fn wait_for_spawn(self, _context: Context, dataflow_id: Uuid) -> Result<(), String> {
        let (tx, rx) = oneshot::channel();
        if let Some(mut dataflow) = self.state.running_dataflows.get_mut(&dataflow_id) {
            dataflow.spawn_result.register(tx);
        } else {
            return Err(format!("unknown dataflow {dataflow_id}"));
        }

        match rx.await {
            Ok(Ok(_uuid)) => Ok(()),
            Ok(Err(err)) => Err(err_to_string(err)),
            Err(_) => Err("coordinator dropped the reply sender".to_string()),
        }
    }

    async fn reload(
        self,
        _context: Context,
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> Result<Uuid, String> {
        reload_dataflow(
            &self.state.running_dataflows,
            dataflow_id,
            node_id,
            operator_id,
            &self.state.daemon_connections,
            self.state.clock.new_timestamp(),
        )
        .await
        .map_err(err_to_string)?;
        Ok(dataflow_id)
    }

    async fn check(
        self,
        _context: Context,
        dataflow_uuid: Uuid,
    ) -> Result<CheckDataflowReply, String> {
        let status = match self.state.running_dataflows.get(&dataflow_uuid) {
            Some(_) => CheckDataflowReply::Running {
                uuid: dataflow_uuid,
            },
            None => CheckDataflowReply::Stopped {
                uuid: dataflow_uuid,
                result: self
                    .state
                    .dataflow_results
                    .get(&dataflow_uuid)
                    .map(|r| dataflow_result(r.value(), dataflow_uuid, &self.state.clock))
                    .unwrap_or_else(|| {
                        DataflowResult::ok_empty(dataflow_uuid, self.state.clock.new_timestamp())
                    }),
            },
        };
        Ok(status)
    }

    async fn stop(
        self,
        _context: Context,
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<StopDataflowReply, String> {
        stop_dataflow_impl(&self.state, dataflow_uuid, grace_duration, force).await
    }

    async fn stop_by_name(
        self,
        _context: Context,
        name: String,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<StopDataflowReply, String> {
        let dataflow_uuid = resolve_name(
            name,
            &self.state.running_dataflows,
            &self.state.archived_dataflows,
        )
        .map_err(err_to_string)?;

        stop_dataflow_impl(&self.state, dataflow_uuid, grace_duration, force).await
    }

    async fn logs(
        self,
        _context: Context,
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
        tail: Option<usize>,
    ) -> Result<Vec<u8>, String> {
        let dataflow_uuid = if let Some(uuid) = uuid {
            Ok(uuid)
        } else if let Some(name) = name {
            resolve_name(
                name,
                &self.state.running_dataflows,
                &self.state.archived_dataflows,
            )
        } else {
            Err(eyre!("No uuid"))
        }
        .map_err(err_to_string)?;

        retrieve_logs(
            &self.state.running_dataflows,
            &self.state.archived_dataflows,
            dataflow_uuid,
            node.into(),
            &self.state.daemon_connections,
            self.state.clock.new_timestamp(),
            tail,
        )
        .await
        .map_err(err_to_string)
    }

    async fn destroy(self, _context: Context) -> Result<(), String> {
        tracing::info!("Received destroy command");

        handle_destroy(&self.state).await.map_err(err_to_string)
    }

    async fn list(self, _context: Context) -> Result<DataflowList, String> {
        // Convert to owned entries immediately to release DashMap locks.
        let running: Vec<_> = self
            .state
            .running_dataflows
            .iter()
            .map(|d| DataflowListEntry {
                id: DataflowIdAndName {
                    uuid: d.value().uuid,
                    name: d.value().name.clone(),
                },
                status: DataflowStatus::Running,
            })
            .collect();
        let finished_failed: Vec<_> = self
            .state
            .dataflow_results
            .iter()
            .map(|r| {
                let uuid = *r.key();
                let name = self
                    .state
                    .archived_dataflows
                    .get(&uuid)
                    .and_then(|d| d.name.clone());
                let id = DataflowIdAndName { uuid, name };
                let status = if r.value().values().all(|r| r.is_ok()) {
                    DataflowStatus::Finished
                } else {
                    DataflowStatus::Failed
                };
                DataflowListEntry { id, status }
            })
            .collect();

        let sort_key = |e: &DataflowListEntry| (e.id.name.clone(), e.id.uuid);
        let mut running = running;
        let mut finished_failed = finished_failed;
        running.sort_by_key(sort_key);
        finished_failed.sort_by_key(sort_key);

        running.extend(finished_failed);
        Ok(DataflowList(running))
    }

    async fn info(self, _context: Context, dataflow_uuid: Uuid) -> Result<DataflowInfo, String> {
        if let Some(dataflow) = self.state.running_dataflows.get(&dataflow_uuid) {
            Ok(DataflowInfo {
                uuid: dataflow.uuid,
                name: dataflow.name.clone(),
                descriptor: dataflow.descriptor.clone(),
            })
        } else {
            Err(format!("No running dataflow with uuid `{dataflow_uuid}`"))
        }
    }

    async fn daemon_connected(self, _context: Context) -> Result<bool, String> {
        Ok(!self.state.daemon_connections.is_empty())
    }

    async fn connected_machines(self, _context: Context) -> Result<BTreeSet<DaemonId>, String> {
        Ok(self.state.daemon_connections.keys().collect())
    }

    async fn cli_and_default_daemon_on_same_machine(
        self,
        _context: Context,
    ) -> Result<bool, String> {
        let Some(cli_ip) = self.client_ip else {
            return Ok(false);
        };
        let Some(default_id) = self.state.daemon_connections.unnamed().next() else {
            return Ok(false);
        };
        let Some(stream) = self.state.daemon_connections.get_stream(&default_id) else {
            return Ok(false);
        };
        let stream = stream.lock().await;
        let same = stream
            .peer_addr()
            .ok()
            .map(|addr| addr.ip() == cli_ip)
            .unwrap_or(false);
        Ok(same)
    }

    async fn get_version(self, _context: Context) -> VersionInfo {
        VersionInfo {
            coordinator_version: env!("CARGO_PKG_VERSION").to_string(),
            message_format_version: dora_message::VERSION.to_string(),
        }
    }

    async fn get_node_info(self, _context: Context) -> Result<Vec<NodeInfo>, String> {
        let mut node_infos = Vec::new();
        for r in self.state.running_dataflows.iter() {
            let dataflow = r.value();
            for (node_id, _node) in &dataflow.nodes {
                // Get the specific daemon this node is running on
                if let Some(daemon_id) = dataflow.node_to_daemon.get(node_id) {
                    // Get metrics if available
                    let metrics = dataflow.node_metrics.get(node_id).map(|m| {
                        NodeMetricsInfo {
                            pid: m.pid,
                            cpu_usage: m.cpu_usage,
                            // Use 1000 for MB (megabytes) instead of 1024 (mebibytes)
                            memory_mb: m.memory_bytes as f64 / 1000.0 / 1000.0,
                            disk_read_mb_s: m.disk_read_bytes.map(|b| b as f64 / 1000.0 / 1000.0),
                            disk_write_mb_s: m.disk_write_bytes.map(|b| b as f64 / 1000.0 / 1000.0),
                            start_time: m.start_time,
                        }
                    });

                    node_infos.push(NodeInfo {
                        dataflow_id: dataflow.uuid,
                        dataflow_name: dataflow.name.clone(),
                        node_id: node_id.clone(),
                        daemon_id: daemon_id.clone(),
                        metrics,
                    });
                }
            }
        }
        Ok(node_infos)
    }
}
