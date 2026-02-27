use adora_core::descriptor::Descriptor;
use adora_message::{
    BuildId,
    cli_to_coordinator::ControlRequest,
    common::{GitSource, LogMessage},
    coordinator_to_cli::ControlRequestReply,
    id::NodeId,
};
use eyre::{Context, bail};
use std::collections::BTreeMap;

use crate::{
    output::{LogOutputConfig, print_log_message},
    session::DataflowSession,
    ws_client::WsSession,
};

pub fn build_distributed_dataflow(
    session: &WsSession,
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    local_working_dir: Option<std::path::PathBuf>,
    uv: bool,
) -> eyre::Result<BuildId> {
    let build_id = {
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Build {
                    session_id: dataflow_session.session_id,
                    dataflow,
                    git_sources: git_sources.clone(),
                    prev_git_sources: dataflow_session.git_sources.clone(),
                    local_working_dir,
                    uv,
                })
                .unwrap(),
            )
            .wrap_err("failed to send start dataflow message")?;

        let result: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match result {
            ControlRequestReply::DataflowBuildTriggered { build_id } => {
                println!("dataflow build triggered: {build_id}");
                build_id
            }
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected start dataflow reply: {other:?}"),
        }
    };
    Ok(build_id)
}

pub fn wait_until_dataflow_built(
    build_id: BuildId,
    session: &WsSession,
    log_level: log::LevelFilter,
) -> eyre::Result<BuildId> {
    // subscribe to log messages
    let log_rx = session.subscribe_logs(
        &serde_json::to_vec(&ControlRequest::BuildLogSubscribe {
            build_id,
            level: log_level,
        })
        .wrap_err("failed to serialize message")?,
    )?;
    std::thread::spawn(move || {
        while let Ok(Ok(raw)) = log_rx.recv() {
            let parsed: eyre::Result<LogMessage> =
                serde_json::from_slice(&raw).context("failed to parse log message");
            match parsed {
                Ok(log_message) => {
                    let config = LogOutputConfig {
                        print_daemon_name: true,
                        ..LogOutputConfig::default()
                    };
                    print_log_message(log_message, &config);
                }
                Err(err) => {
                    tracing::warn!("failed to parse log message: {err:?}")
                }
            }
        }
    });

    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::WaitForBuild { build_id }).unwrap())
        .wrap_err("failed to send WaitForBuild message")?;

    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowBuildFinished { build_id, result } => match result {
            Ok(()) => {
                println!("dataflow build finished successfully");
                Ok(build_id)
            }
            Err(err) => bail!("{err}"),
        },
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    }
}
