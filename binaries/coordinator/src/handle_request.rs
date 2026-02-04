use std::collections::BTreeSet;

use dora_message::{
    BuildId, Request,
    cli_to_coordinator::{
        BuildRequest, CheckRequest, CheckResponse, CliAndDefaultDaemonIps,
        CliAndDefaultDaemonOnSameMachineRequest, ConnectedMachinesRequest, DaemonConnectedRequest,
        DataflowBuildTriggered, DataflowInfo, DataflowReloaded, DataflowStartTriggered,
        DataflowStopped, DestroyRequest, GetNodeInfoRequest, InfoRequest, ListRequest, LogsRequest,
        ReloadRequest, StartRequest, StopByNameRequest, StopRequest,
    },
    common::DaemonId,
    coordinator_to_cli::{
        DataflowIdAndName, DataflowList, DataflowListEntry, DataflowResult, DataflowStatus,
        NodeInfo, NodeMetricsInfo,
    },
};
use petname::petname;

use crate::{
    Coordinator, build_dataflow, dataflow_result, reload_dataflow, resolve_name, retrieve_logs,
    start_dataflow, stop_dataflow,
};

pub trait HandleRequest<Req>
where
    Req: Request,
{
    async fn handle_request(&mut self, request: Req) -> <Req as Request>::Response;
}

impl HandleRequest<BuildRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        request: BuildRequest,
    ) -> <BuildRequest as Request>::Response {
        let BuildRequest {
            session_id,
            dataflow,
            git_sources,
            prev_git_sources,
            local_working_dir,
            uv,
        } = request;
        // assign a random build id
        let build_id = BuildId::generate();

        let result = build_dataflow(
            build_id,
            session_id,
            dataflow,
            git_sources,
            prev_git_sources,
            local_working_dir,
            &self.clock,
            uv,
            &mut self.daemon_connections,
        )
        .await;
        match result {
            Ok(build) => {
                self.running_builds.insert(build_id, build);
                Ok(DataflowBuildTriggered { build_id })
            }
            Err(err) => Err(format!("{err:?}")),
        }
    }
}

impl HandleRequest<StartRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        request: StartRequest,
    ) -> <StartRequest as Request>::Response {
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

        // check that name is unique
        if let Some(name) = name.as_deref() {
            if self
                .running_dataflows
                .values()
                .any(|d| d.name.as_deref() == Some(name))
            {
                return Err(format!(
                    "there is already a running dataflow with name `{name}`"
                ));
            }
        }

        let result = start_dataflow(
            build_id,
            session_id,
            dataflow,
            local_working_dir,
            name,
            &mut self.daemon_connections,
            &self.clock,
            uv,
            write_events_to,
        )
        .await;

        match result {
            Ok(dataflow) => {
                let uuid = dataflow.uuid;
                self.running_dataflows.insert(uuid, dataflow);
                Ok(DataflowStartTriggered { uuid })
            }
            Err(err) => Err(format!("{err:?}")),
        }
    }
}

impl HandleRequest<CheckRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        request: CheckRequest,
    ) -> <CheckRequest as Request>::Response {
        let CheckRequest { dataflow_uuid } = request;

        match self.running_dataflows.get(&dataflow_uuid) {
            Some(_) => CheckResponse::Running,
            None => CheckResponse::Stopped {
                result: self
                    .dataflow_results
                    .get(&dataflow_uuid)
                    .map(|r| dataflow_result(r, dataflow_uuid, &self.clock))
                    .unwrap_or_else(|| {
                        DataflowResult::ok_empty(dataflow_uuid, self.clock.new_timestamp())
                    }),
            },
        }
    }
}

impl HandleRequest<ReloadRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        request: ReloadRequest,
    ) -> <ReloadRequest as Request>::Response {
        let ReloadRequest {
            dataflow_id,
            node_id,
            operator_id,
        } = request;

        let result = reload_dataflow(
            &self.running_dataflows,
            dataflow_id,
            node_id,
            operator_id,
            &mut self.daemon_connections,
            self.clock.new_timestamp(),
        )
        .await;

        result
            .map(|()| DataflowReloaded { uuid: dataflow_id })
            .map_err(|err| format!("{err:?}"))
    }
}

impl HandleRequest<StopRequest> for Coordinator {
    async fn handle_request(&mut self, request: StopRequest) -> <StopRequest as Request>::Response {
        let StopRequest {
            dataflow_uuid,
            grace_duration,
            force,
        } = request;

        // Check if already finished
        if let Some(result) = self.dataflow_results.get(&dataflow_uuid) {
            return Ok(DataflowStopped {
                uuid: dataflow_uuid,
                result: dataflow_result(result, dataflow_uuid, &self.clock),
            });
        }

        let result = stop_dataflow(
            &mut self.running_dataflows,
            dataflow_uuid,
            &mut self.daemon_connections,
            self.clock.new_timestamp(),
            grace_duration,
            force,
        )
        .await;

        match result {
            Ok(_dataflow) => {
                // The actual result will be sent via stop_reply_senders when the dataflow finishes
                // For now, we need to handle this specially - this is a "pending" response
                // Return an error indicating the caller should wait
                Err("stop_pending".to_string())
            }
            Err(err) => Err(format!("{err:?}")),
        }
    }
}

impl HandleRequest<StopByNameRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        request: StopByNameRequest,
    ) -> <StopByNameRequest as Request>::Response {
        let StopByNameRequest {
            name,
            grace_duration,
            force,
        } = request;

        let dataflow_uuid =
            match resolve_name(name, &self.running_dataflows, &self.archived_dataflows) {
                Ok(uuid) => uuid,
                Err(err) => return Err(format!("{err:?}")),
            };

        // Check if already finished
        if let Some(result) = self.dataflow_results.get(&dataflow_uuid) {
            return Ok(DataflowStopped {
                uuid: dataflow_uuid,
                result: dataflow_result(result, dataflow_uuid, &self.clock),
            });
        }

        let result = stop_dataflow(
            &mut self.running_dataflows,
            dataflow_uuid,
            &mut self.daemon_connections,
            self.clock.new_timestamp(),
            grace_duration,
            force,
        )
        .await;

        match result {
            Ok(_dataflow) => {
                // The actual result will be sent via stop_reply_senders when the dataflow finishes
                Err("stop_pending".to_string())
            }
            Err(err) => Err(format!("{err:?}")),
        }
    }
}

impl HandleRequest<LogsRequest> for Coordinator {
    async fn handle_request(&mut self, request: LogsRequest) -> <LogsRequest as Request>::Response {
        let LogsRequest {
            uuid,
            name,
            node,
            tail,
        } = request;

        let dataflow_uuid = if let Some(uuid) = uuid {
            Ok(uuid)
        } else if let Some(name) = name {
            resolve_name(name, &self.running_dataflows, &self.archived_dataflows)
        } else {
            Err(eyre::eyre!("No uuid"))
        };

        match dataflow_uuid {
            Ok(uuid) => retrieve_logs(
                &self.running_dataflows,
                &self.archived_dataflows,
                uuid,
                node.into(),
                &mut self.daemon_connections,
                self.clock.new_timestamp(),
                tail,
            )
            .await
            .map_err(|err| format!("{err:?}")),
            Err(err) => Err(format!("{err:?}")),
        }
    }
}

impl HandleRequest<InfoRequest> for Coordinator {
    async fn handle_request(&mut self, request: InfoRequest) -> <InfoRequest as Request>::Response {
        let InfoRequest { dataflow_uuid } = request;

        if let Some(dataflow) = self.running_dataflows.get(&dataflow_uuid) {
            Ok(DataflowInfo {
                uuid: dataflow.uuid,
                name: dataflow.name.clone(),
                descriptor: dataflow.descriptor.clone(),
            })
        } else {
            Err(format!("No running dataflow with uuid `{dataflow_uuid}`"))
        }
    }
}

impl HandleRequest<DestroyRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        _request: DestroyRequest,
    ) -> <DestroyRequest as Request>::Response {
        tracing::info!("Received destroy command");

        // Note: This needs special handling because it requires abort_handle and daemon_events_tx
        // which are not part of Coordinator struct. For now, return an error indicating
        // this should be handled specially.
        Err("destroy_requires_special_handling".to_string())
    }
}

impl HandleRequest<ListRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        _request: ListRequest,
    ) -> <ListRequest as Request>::Response {
        let mut dataflows: Vec<_> = self.running_dataflows.values().collect();
        dataflows.sort_by_key(|d| (&d.name, d.uuid));

        let running = dataflows.into_iter().map(|d| DataflowListEntry {
            id: DataflowIdAndName {
                uuid: d.uuid,
                name: d.name.clone(),
            },
            status: DataflowStatus::Running,
        });

        let finished_failed = self.dataflow_results.iter().map(|(&uuid, results)| {
            let name = self
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

        DataflowList(running.chain(finished_failed).collect())
    }
}

impl HandleRequest<DaemonConnectedRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        _request: DaemonConnectedRequest,
    ) -> <DaemonConnectedRequest as Request>::Response {
        !self.daemon_connections.is_empty()
    }
}

impl HandleRequest<ConnectedMachinesRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        _request: ConnectedMachinesRequest,
    ) -> <ConnectedMachinesRequest as Request>::Response {
        self.daemon_connections
            .keys()
            .cloned()
            .collect::<BTreeSet<DaemonId>>()
    }
}

impl HandleRequest<CliAndDefaultDaemonOnSameMachineRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        _request: CliAndDefaultDaemonOnSameMachineRequest,
    ) -> <CliAndDefaultDaemonOnSameMachineRequest as Request>::Response {
        let mut default_daemon_ip = None;
        if let Some(default_id) = self.daemon_connections.unnamed().next() {
            if let Some(connection) = self.daemon_connections.get(default_id) {
                if let Ok(addr) = connection.stream.peer_addr() {
                    default_daemon_ip = Some(addr.ip());
                }
            }
        }
        CliAndDefaultDaemonIps {
            default_daemon: default_daemon_ip,
            cli: None, // filled later
        }
    }
}

impl HandleRequest<GetNodeInfoRequest> for Coordinator {
    async fn handle_request(
        &mut self,
        _request: GetNodeInfoRequest,
    ) -> <GetNodeInfoRequest as Request>::Response {
        let mut node_infos = Vec::new();
        for dataflow in self.running_dataflows.values() {
            for (node_id, _node) in &dataflow.nodes {
                // Get the specific daemon this node is running on
                if let Some(daemon_id) = dataflow.node_to_daemon.get(node_id) {
                    // Get metrics if available
                    let metrics = dataflow.node_metrics.get(node_id).map(|m| NodeMetricsInfo {
                        pid: m.pid,
                        cpu_usage: m.cpu_usage,
                        // Use 1000 for MB (megabytes) instead of 1024 (mebibytes)
                        memory_mb: m.memory_bytes as f64 / 1000.0 / 1000.0,
                        disk_read_mb_s: m.disk_read_bytes.map(|b| b as f64 / 1000.0 / 1000.0),
                        disk_write_mb_s: m.disk_write_bytes.map(|b| b as f64 / 1000.0 / 1000.0),
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
        node_infos
    }
}
