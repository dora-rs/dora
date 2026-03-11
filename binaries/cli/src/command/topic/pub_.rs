use clap::Args;

use crate::{
    command::{Executable, default_tracing, topic::selector::DataflowSelector},
    common::CoordinatorOptions,
    ws_client::WsSession,
};
use adora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::ControlRequestReply,
    id::{DataId, NodeId},
};
use eyre::{Context, bail};

/// Publish a message to a topic for debugging and testing.
///
/// Sends JSON data to a topic via the coordinator's Zenoh session.
/// Useful for injecting test data into a running dataflow.
///
/// The dataflow must have debug mode enabled:
///
/// ```yaml
/// _unstable_debug:
///   publish_all_messages_to_zenoh: true
/// ```
///
/// Examples:
///
/// Publish JSON data to a topic:
///   adora topic pub -d my-dataflow sensor/reading '{"temperature": 25.5}'
///
/// Publish from a file:
///   adora topic pub -d my-dataflow sensor/reading --file data.json
///
/// Publish multiple times:
///   adora topic pub -d my-dataflow sensor/reading '42' --count 10
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Pub {
    #[clap(flatten)]
    selector: DataflowSelector,

    /// Topic to publish to (format: node_id/output_id)
    #[clap(value_name = "TOPIC")]
    topic: String,

    /// JSON data to publish
    #[clap(value_name = "DATA", required_unless_present = "file")]
    data: Option<String>,

    /// Read data from a JSON file instead of command line
    #[clap(long, conflicts_with = "data")]
    file: Option<String>,

    /// Number of messages to publish
    #[clap(long, default_value_t = 1, value_parser = clap::value_parser!(u32).range(1..))]
    count: u32,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Pub {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let session = self.coordinator.connect()?;
        let (dataflow_id, _descriptor) = self.selector.resolve(&session)?;

        // Parse topic as node_id/output_id
        let (node_id, output_id) = parse_topic(&self.topic)?;

        // Get data from args or file
        let data_json = if let Some(data) = self.data {
            data
        } else if let Some(path) = self.file {
            std::fs::read_to_string(&path)
                .with_context(|| format!("failed to read file: {path}"))?
        } else {
            bail!("either DATA or --file must be provided");
        };

        // Validate JSON
        serde_json::from_str::<serde_json::Value>(&data_json).wrap_err(
            "invalid JSON data\n\n  \
             hint: data must be valid JSON, e.g. '{\"key\": 42}' or '\"hello\"'",
        )?;

        for i in 0..self.count {
            publish(&session, dataflow_id, &node_id, &output_id, &data_json)?;
            if self.count > 1 {
                eprintln!("Published message {}/{}", i + 1, self.count);
            }
        }

        if self.count == 1 {
            eprintln!("Published to {}", self.topic);
        }

        Ok(())
    }
}

fn parse_topic(topic: &str) -> eyre::Result<(NodeId, DataId)> {
    let parts: Vec<&str> = topic.splitn(2, '/').collect();
    if parts.len() != 2 || parts[0].is_empty() || parts[1].is_empty() {
        bail!("invalid topic format: expected 'node_id/output_id', got '{topic}'");
    }
    Ok((parts[0].to_string().into(), parts[1].to_string().into()))
}

fn publish(
    session: &WsSession,
    dataflow_id: uuid::Uuid,
    node_id: &NodeId,
    output_id: &DataId,
    data_json: &str,
) -> eyre::Result<()> {
    let reply_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::TopicPublish {
                dataflow_id,
                node_id: node_id.clone(),
                output_id: output_id.clone(),
                data_json: data_json.to_string(),
            })
            .unwrap(),
        )
        .wrap_err("failed to send TopicPublish request")?;

    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;

    match reply {
        ControlRequestReply::TopicPublished => Ok(()),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected reply: {other:?}"),
    }
}
