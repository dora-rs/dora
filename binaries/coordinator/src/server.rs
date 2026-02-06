use std::sync::Arc;

use dora_core::uhlc::HLC;
use dora_message::{
    BuildId,
    cli_to_coordinator::{BuildRequest, CliControl, ControlRequest, StartRequest},
    tarpc::context::Context,
};
use uuid::Uuid;

use crate::state::CoordinatorState;

use super::build_dataflow;

struct ControlServer {
    state: Arc<CoordinatorState>,
}

impl CliControl for ControlServer {
    async fn build(self, context: Context, request: BuildRequest) -> eyre::Result<BuildId> {
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
            Err(err) => Err(err),
        }
    }

    async fn wait_for_build(build_id: BuildId) -> eyre::Result<()> {
        todo!()
    }

    async fn start(request: StartRequest) -> eyre::Result<Uuid> {
        todo!()
    }

    async fn wait_for_spawn(dataflow_id: Uuid) -> eyre::Result<()> {
        todo!()
    }
}
