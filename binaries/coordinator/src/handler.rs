use std::{collections::BTreeSet, time::Duration};

use async_trait::async_trait;
use dora_message::{
    BuildId,
    cli_to_coordinator::{
        BuildReq, CliAndDefaultDaemonIps, CliToCoordinator, StartReq, WaitForBuildResp,
    },
    common::DaemonId,
    coordinator_to_cli::{DataflowListEntry, NodeInfo},
    id::{NodeId, OperatorId},
};
use eyre::Result;
use uuid::Uuid;

pub struct CliRequestHandler;

#[async_trait]
impl CliToCoordinator for CliRequestHandler {
    async fn build(self, req: BuildReq) -> Result<BuildId> {
        // let BuildReq {
        //     session_id,
        //     dataflow,
        //     git_sources,
        //     prev_git_sources,
        //     local_working_dir,
        //     uv,
        // } = req;

        // // assign a random build id
        // let build_id = BuildId::generate();

        // let result = build_dataflow(
        //     build_id,
        //     session_id,
        //     dataflow,
        //     git_sources,
        //     prev_git_sources,
        //     local_working_dir,
        //     &self.clock,
        //     uv,
        //     &mut daemon_connections,
        // )
        // .await;
        // match result {
        //     Ok(build) => {
        //         running_builds.insert(build_id, build);
        //         Ok(build_id)
        //     }
        //     Err(err) => {
        //         let _ = reply_sender.send(Err(err));
        //     }
        // }
        todo!()
    }
    async fn wait_for_build(self, build_id: BuildId) -> Result<WaitForBuildResp> {
        todo!()
    }
    async fn start(self, req: StartReq) -> Result<Uuid> {
        todo!()
    }
    async fn wait_for_spawn(self, dataflow_id: Uuid) -> Result<Uuid> {
        todo!()
    }
    async fn reload(
        self,
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> Result<Uuid> {
        todo!()
    }
    async fn check(self, dataflow_uuid: Uuid) -> Result<Uuid> {
        todo!()
    }
    async fn stop(
        self,
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<()> {
        todo!()
    }
    async fn stop_by_name(
        self,
        name: String,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<()> {
        todo!()
    }
    async fn logs(
        self,
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
        tail: Option<usize>,
    ) -> Result<Vec<u8>> {
        todo!()
    }
    async fn destroy(self) -> Result<()> {
        todo!()
    }
    async fn list(self) -> Result<Vec<DataflowListEntry>> {
        todo!()
    }
    async fn info(self, dataflow_uuid: Uuid) -> Result<()> {
        todo!()
    }
    async fn daemon_connected(self) -> Result<bool> {
        todo!()
    }
    async fn connected_machines(self) -> Result<BTreeSet<DaemonId>> {
        todo!()
    }
    async fn log_subscribe(self, dataflow_id: Uuid, level: log::LevelFilter) -> Result<()> {
        todo!()
    }
    async fn build_log_subscribe(self, build_id: BuildId, level: log::LevelFilter) -> Result<()> {
        todo!()
    }
    async fn cli_and_default_daemon_on_same_machine(self) -> Result<CliAndDefaultDaemonIps> {
        todo!()
    }
    async fn get_node_info(self) -> Result<Vec<NodeInfo>> {
        todo!()
    }
}
