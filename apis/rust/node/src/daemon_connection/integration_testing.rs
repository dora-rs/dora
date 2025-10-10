use std::{fs::File, path::PathBuf};

use colored::Colorize;
use dora_message::{
    common::Timestamped, daemon_to_node::DaemonReply, node_to_daemon::DaemonRequest,
};
use eyre::Context;

use crate::event_stream::data_to_arrow_array;

pub struct IntegrationTestingEvents {
    input_file: File,
    output_file: File,
    stopped: bool,
}

impl IntegrationTestingEvents {
    pub fn new(input_file_path: PathBuf, output_file_path: PathBuf) -> eyre::Result<Self> {
        let input_file = File::open(&input_file_path)
            .with_context(|| format!("failed to open {}", input_file_path.display()))?;
        let output_file = File::create(&output_file_path)
            .with_context(|| format!("failed to create {}", output_file_path.display()))?;

        Ok(Self {
            input_file,
            output_file,
            stopped: false,
        })
    }

    pub fn request(
        &mut self,
        request: &Timestamped<DaemonRequest>,
    ) -> Result<DaemonReply, eyre::Error> {
        let reply = match &request.inner {
            DaemonRequest::Register(_) => DaemonReply::Result(Ok(())),
            DaemonRequest::Subscribe => DaemonReply::Result(Ok(())),
            DaemonRequest::SubscribeDrop => DaemonReply::Result(Ok(())),
            DaemonRequest::NextEvent { .. } => todo!("read next event from input_file"),
            DaemonRequest::SendMessage {
                output_id,
                metadata,
                data,
            } => {
                let (drop_tx, drop_rx) = flume::unbounded();
                let array = data_to_arrow_array(data.clone(), metadata, drop_tx);
                // integration testing deosn't use shared memory -> no drop tokens
                let _ = drop_rx;

                todo!("serialize array and write it to output_file");
                DaemonReply::Empty
            }
            DaemonRequest::CloseOutputs(data_ids) => {
                println!("{} {data_ids:?}", "node reports closed outputs".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::OutputsDone => {
                println!("{}", "node reports OutputsDone".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::ReportDropTokens { drop_tokens } => {
                println!("{} {drop_tokens:?}", "node reports drop tokens".blue());
                DaemonReply::Empty
            }
            DaemonRequest::NextFinishedDropTokens => {
                // interactive nodes don't use shared memory -> no drop tokens
                DaemonReply::NextDropEvents(vec![])
            }
            DaemonRequest::EventStreamDropped => {
                println!("{}", "node reports EventStreamDropped".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::NodeConfig { .. } => {
                eyre::bail!("unexpected NodeConfig in interactive mode")
            }
        };
        Ok(reply)
    }
}
