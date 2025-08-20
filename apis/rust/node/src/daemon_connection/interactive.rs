use std::{io::stdout, time::Duration};

use dora_core::{metadata::ArrowTypeInfoExt, uhlc::HLC};
use dora_message::{
    common::{DataMessage, Timestamped},
    daemon_to_node::{DaemonReply, NodeEvent},
    metadata::{ArrowTypeInfo, Metadata},
    node_to_daemon::DaemonRequest,
};

pub struct InteractiveEvents {
    stopped: bool,
}

impl Default for InteractiveEvents {
    fn default() -> Self {
        Self { stopped: false }
    }
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
            DaemonRequest::NextEvent { drop_tokens } => {
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
            } => todo!(),
            DaemonRequest::CloseOutputs(data_ids) => {
                println!("node reports closed outputs {data_ids:?}");
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::OutputsDone => {
                println!("node reports OutputsDone");
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::ReportDropTokens { drop_tokens } => {
                println!("node reports drop tokens {drop_tokens:?}");
                DaemonReply::Empty
            }
            DaemonRequest::NextFinishedDropTokens => DaemonReply::NextDropEvents(vec![]), // TODO
            DaemonRequest::EventStreamDropped => {
                println!("node reports EventStreamDropped");
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::NodeConfig { node_id } => todo!(),
        };
        Ok(reply)
    }

    fn next_event(&mut self) -> eyre::Result<Option<NodeEvent>> {
        std::thread::sleep(Duration::from_millis(500));
        println!("Node asks for next input");
        if self.stopped {
            println!("event channel was stopped -> returning empty event list");
            return Ok(None);
        }
        let stdout_lock = stdout().lock();
        let id = inquire::Text::new("Input ID")
            .with_help_message("empty input ID to stop")
            .prompt()?;
        std::mem::drop(stdout_lock);
        let event = if id.is_empty() {
            println!("given input ID is empty -> stopping");
            self.stopped = true;
            NodeEvent::Stop
        } else {
            let id = id.into();
            let stdout_lock = stdout().lock();
            let data = inquire::Text::new("Data")
                .with_help_message("optional")
                .prompt_skippable()?;
            std::mem::drop(stdout_lock);
            let type_info = data
                .as_ref()
                .map(|d| ArrowTypeInfo::byte_array(d.len()))
                .unwrap_or_else(ArrowTypeInfo::empty);
            NodeEvent::Input {
                id,
                metadata: Metadata::new(HLC::default().new_timestamp(), type_info),
                data: data
                    .map(|d| DataMessage::Vec(aligned_vec::AVec::from_slice(1, d.as_bytes()))),
            }
        };
        Ok(Some(event))
    }
}
