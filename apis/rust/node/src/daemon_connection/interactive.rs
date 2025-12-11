use std::{io::stdout, time::Duration};

use colored::Colorize;
use dora_core::{metadata::ArrowTypeInfoExt, uhlc::HLC};
use dora_message::{
    common::{DataMessage, Timestamped},
    daemon_to_node::{DaemonReply, NodeEvent},
    metadata::{ArrowTypeInfo, Metadata},
    node_to_daemon::DaemonRequest,
};

use crate::{
    arrow_utils::{copy_array_into_sample, required_data_size},
    daemon_connection::json_to_arrow::read_json_bytes_as_arrow,
    event_stream::data_to_arrow_array,
};

#[derive(Default)]
pub struct InteractiveEvents {
    stopped: bool,
}

impl InteractiveEvents {
    pub fn request(
        &mut self,
        request: &Timestamped<DaemonRequest>,
    ) -> Result<DaemonReply, eyre::Error> {
        let reply = match &request.inner {
            DaemonRequest::Register(_) => DaemonReply::Result(Ok(())),
            DaemonRequest::Subscribe => DaemonReply::Result(Ok(())),
            DaemonRequest::SubscribeDrop => DaemonReply::Result(Ok(())),
            DaemonRequest::NextEvent { .. } => {
                let events = if let Some(event) = self.next_event()? {
                    let event = Timestamped {
                        inner: event,
                        timestamp: HLC::default().new_timestamp(),
                    };
                    vec![event]
                } else {
                    vec![]
                };
                DaemonReply::NextEvents(events)
            }
            DaemonRequest::SendMessage {
                output_id,
                metadata,
                data,
            } => {
                let (drop_tx, drop_rx) = flume::unbounded();
                let array = data_to_arrow_array(data.clone(), metadata, drop_tx);
                // interactive nodes don't use shared memory -> no drop tokens
                let _ = drop_rx;

                let array_display = match array {
                    Err(err) => format!("<error>: {err:?}"),
                    Ok(data) => format!("{data:?}"),
                };
                println!(
                    "{} {} with data: {array_display}",
                    "node sends output".yellow(),
                    output_id.bright_blue().bold()
                );
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

    fn next_event(&mut self) -> eyre::Result<Option<NodeEvent>> {
        std::thread::sleep(Duration::from_millis(500));
        println!("{}", "Node asks for next input".blue());
        if self.stopped {
            println!(
                "{}",
                "event channel was stopped -> returning empty event list".green()
            );
            return Ok(None);
        }
        let stdout_lock = stdout().lock();
        let id = inquire::Text::new("Input ID")
            .with_help_message("empty input ID to stop")
            .prompt()?;
        std::mem::drop(stdout_lock);
        let event = if id.is_empty() {
            println!("{}", "given input ID is empty -> stopping".blue());
            self.stopped = true;
            NodeEvent::Stop
        } else {
            let id = id.into();
            let (data, type_info) = loop {
                let stdout_lock = stdout().lock();
                let data = inquire::Text::new("Data")
                    .with_help_message("optional, String or JSON, esc to skip")
                    .prompt_skippable()?;
                std::mem::drop(stdout_lock);
                let typed_data = if let Some(data) = data {
                    // input is JSON data
                    let array = match read_json_bytes_as_arrow(data.as_bytes()) {
                        Ok(array) => array,
                        Err(err) => {
                            eprintln!("{}", format!("{err}").red());
                            continue;
                        }
                    };

                    let total_len = required_data_size(&array);
                    let mut buf = vec![0; total_len];
                    let type_info = copy_array_into_sample(buf.as_mut_slice(), &array);

                    (Some(buf), type_info)
                } else {
                    (None, ArrowTypeInfo::empty())
                };
                break typed_data;
            };

            NodeEvent::Input {
                id,
                metadata: Metadata::new(HLC::default().new_timestamp(), type_info),
                data: data.map(|d| DataMessage::Vec(aligned_vec::AVec::from_slice(1, &d))),
            }
        };
        Ok(Some(event))
    }
}
