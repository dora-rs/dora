use std::{collections::BTreeSet, sync::Arc, time::Duration};

use async_trait::async_trait;
use dora_message::{
    BuildId,
    cli_to_coordinator::{
        BuildReq, CliAndDefaultDaemonIps, CliToCoordinator, DataflowInfo, CheckResp,
        DataflowStopped, StartReq, WaitForBuildResp,
    },
    common::DaemonId,
    coordinator_to_cli::{
        DataflowIdAndName, DataflowListEntry, DataflowResult, DataflowStatus, NodeInfo,
    },
    id::{NodeId, OperatorId},
};
use eyre::{ Result, bail, eyre};
use petname::petname;
use tokio::sync::{ RwLock};
use uuid::Uuid;

use crate::{Coordinator, dataflow_result, resolve_name};

pub struct CliRequestHandler(pub Arc<RwLock<Coordinator>>);

#[async_trait]
impl CliToCoordinator for CliRequestHandler {
    async fn build(self, req: BuildReq) -> Result<BuildId> {
        let BuildReq {
            session_id,
            dataflow,
            git_sources,
            prev_git_sources,
            local_working_dir,
            uv,
        } = req;
        // assign a random build id
        let build_id = BuildId::generate();

        let mut this = self.0.write().await;
        this.build_dataflow(
            build_id,
            session_id,
            dataflow,
            git_sources,
            prev_git_sources,
            local_working_dir,
            uv,
        )
        .await
        .map(|build| {
            this.running_builds.insert(build_id, build);
            build_id
        })
    }

    async fn wait_for_build(self, build_id: BuildId) -> Result<WaitForBuildResp> {
        let result = {
            let this = self.0.read().await;
            this.running_builds.get(&build_id).map_or_else(
                || this.finished_builds.get(&build_id).cloned(),
                |build| Some(build.build_result.clone()),
            )
        };
        result
            .ok_or_else(|| eyre!("unknown build id {build_id}"))?
            .wait()
            .await
    }

    async fn start(self, req: StartReq) -> Result<Uuid> {
        let StartReq {
            build_id,
            session_id,
            dataflow,
            name,
            local_working_dir,
            uv,
            write_events_to,
        } = req;
        let name = name.or_else(|| petname(2, "-"));

        let mut this = self.0.write().await;
        if let Some(name) = name.as_deref() {
            // check that name is unique
            if this
                .running_dataflows
                .values()
                .any(|d| d.name.as_deref() == Some(name))
            {
                bail!("there is already a running dataflow with name `{name}`");
            }
        }
        this.start_dataflow(
            build_id,
            session_id,
            dataflow,
            local_working_dir,
            name,
            uv,
            write_events_to,
        )
        .await
        .map(|dataflow| {
            let uuid = dataflow.uuid;
            this.running_dataflows.insert(uuid, dataflow);
            uuid
        })
    }

    async fn wait_for_spawn(self, dataflow_id: Uuid) -> Result<Uuid> {
        let result = self
            .0
            .read()
            .await
            .running_dataflows
            .get(&dataflow_id)
            .map(|d| d.spawn_result.clone());
        result
            .ok_or_else(|| eyre!("unknown dataflow {dataflow_id}"))?
            .wait()
            .await
    }

    async fn reload(
        self,
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> Result<Uuid> {
        self.0
            .write()
            .await
            .reload_dataflow(dataflow_id, node_id, operator_id)
            .await
            .map(|_| dataflow_id)
    }

    async fn check(self, dataflow_uuid: Uuid) -> Result<CheckResp> {
        let this = self.0.read().await;
        Ok(match this.running_dataflows.get(&dataflow_uuid) {
            Some(_) => CheckResp::Spawned {
                uuid: dataflow_uuid,
            },
            None => match this.dataflow_results.get(&dataflow_uuid) {
                Some(_) => CheckResp::Stopped {
                    uuid: dataflow_uuid,
                    result: this.dataflow_results.get(&dataflow_uuid).map_or_else(
                        || DataflowResult::ok_empty(dataflow_uuid, this.clock.new_timestamp()),
                        |r| dataflow_result(r, dataflow_uuid, &this.clock),
                    ),
                },
                None => bail!("no dataflow with UUID `{dataflow_uuid}` found"),
            },
        })
    }

    async fn stop(
        self,
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<DataflowStopped> {
        let mut this = self.0.write().await;
        if let Some(result) = this.dataflow_results.get(&dataflow_uuid) {
            return Ok(DataflowStopped {
                uuid: dataflow_uuid,
                result: dataflow_result(result, dataflow_uuid, &this.clock),
            });
        }

        let dataflow = this
            .stop_dataflow(dataflow_uuid, grace_duration, force)
            .await?;
        let result = dataflow.stop_reply_senders.clone();
        drop(this);
        result.wait().await
    }

    async fn stop_by_name(
        self,
        name: String,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<DataflowStopped> {
        let mut this = self.0.write().await;
        let dataflow_uuid = resolve_name(name, &this.running_dataflows, &this.archived_dataflows)?;
        if let Some(result) = this.dataflow_results.get(&dataflow_uuid) {
            return Ok(DataflowStopped {
                uuid: dataflow_uuid,
                result: dataflow_result(result, dataflow_uuid, &this.clock),
            });
        }

        let dataflow = this
            .stop_dataflow(dataflow_uuid, grace_duration, force)
            .await?;
        let result = dataflow.stop_reply_senders.clone();
        drop(this);
        result.wait().await
    }

    async fn logs(
        self,
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
        tail: Option<usize>,
    ) -> Result<Vec<u8>> {
        let mut this = self.0.write().await;
        let dataflow_uuid = if let Some(uuid) = uuid {
            uuid
        } else if let Some(name) = name {
            resolve_name(name, &this.running_dataflows, &this.archived_dataflows)?
        } else {
            bail!("No uuid")
        };
        this.retrieve_logs(dataflow_uuid, node.into(), tail).await
    }

    async fn destroy(self) -> Result<()> {
        tracing::info!("Received destroy command");
        self.0.write().await.handle_destroy().await
    }

    async fn list(self) -> Result<Vec<DataflowListEntry>> {
        let this = self.0.read().await;
        let mut dataflows: Vec<_> = this.running_dataflows.values().collect();
        dataflows.sort_by_key(|d| (&d.name, d.uuid));

        let running = dataflows.into_iter().map(|d| DataflowListEntry {
            id: DataflowIdAndName {
                uuid: d.uuid,
                name: d.name.clone(),
            },
            status: DataflowStatus::Running,
        });
        let finished_failed = this.dataflow_results.iter().map(|(&uuid, results)| {
            let name = this
                .archived_dataflows
                .get(&uuid)
                .and_then(|d| d.name.clone());
            let id = DataflowIdAndName { uuid, name };
            let status = if results.values().all(|r| r.is_ok()) {
                DataflowStatus::Finished
            } else {
                DataflowStatus::Failed
            };
            DataflowListEntry { id, status }
        });

        Ok(running.chain(finished_failed).collect())
    }

    async fn info(self, dataflow_uuid: Uuid) -> Result<DataflowInfo> {
        self.0
            .read()
            .await
            .running_dataflows
            .get(&dataflow_uuid)
            .ok_or_else(|| eyre!("No running dataflow with uuid `{dataflow_uuid}`"))
            .map(|dataflow| DataflowInfo {
                uuid: dataflow.uuid,
                name: dataflow.name.clone(),
                descriptor: dataflow.descriptor.clone(),
            })
    }

    async fn daemon_connected(self) -> Result<bool> {
        Ok(!self.0.read().await.daemon_connections.is_empty())
    }

    async fn connected_machines(self) -> Result<BTreeSet<DaemonId>> {
        Ok(self
            .0
            .read()
            .await
            .daemon_connections
            .keys()
            .cloned()
            .collect())
    }

    async fn log_subscribe(self, _dataflow_id: Uuid, _level: log::LevelFilter) -> Result<()> {
        bail!("LogSubscribe request should be handled separately")
    }
    async fn build_log_subscribe(self, _build_id: BuildId, _level: log::LevelFilter) -> Result<()> {
        bail!("BuildLogSubscribe request should be handled separately")
    }
    async fn cli_and_default_daemon_on_same_machine(self) -> Result<CliAndDefaultDaemonIps> {
        let this = self.0.read().await;
        let mut default_daemon_ip = None;
        if let Some(default_id) = this.daemon_connections.unnamed().next() {
            if let Some(connection) = this.daemon_connections.get(default_id) {
                if let Ok(addr) = connection.stream.peer_addr() {
                    default_daemon_ip = Some(addr.ip());
                }
            }
        }
        Ok(CliAndDefaultDaemonIps {
            default_daemon: default_daemon_ip,
            cli: None, // filled later
        })
    }

    async fn get_node_info(self) -> Result<Vec<NodeInfo>> {
        use dora_message::coordinator_to_cli::{NodeInfo, NodeMetricsInfo};

        let this = self.0.read().await;
        let mut node_infos = Vec::new();
        for dataflow in this.running_dataflows.values() {
            for node_id in dataflow.nodes.keys() {
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
