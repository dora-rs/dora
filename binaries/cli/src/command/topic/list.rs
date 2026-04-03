use std::{collections::BTreeMap, io::Write};

use clap::Args;
use dora_core::{config::InputMapping, topics::zenoh_output_publish_topic};
use dora_message::id::{DataId, NodeId};
use serde::Serialize;
use tabwriter::TabWriter;

use crate::{
    command::{Executable, default_tracing, topic::selector::DataflowSelector},
    common::CoordinatorOptions,
    formatting::OutputFormat,
};

/// Print the outputs defined in a dataflow descriptor.
///
/// Examples:
///
/// List topics in a dataflow:
///   dora topic list -d my-dataflow
///
/// List topics as JSON:
///   dora topic list -d my-dataflow --format json
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct List {
    #[clap(flatten)]
    selector: DataflowSelector,

    /// Output format
    #[clap(long, value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    pub format: OutputFormat,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}
impl Executable for List {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        list(self.coordinator, self.selector, self.format).await
    }
}

#[derive(Serialize)]
struct OutputEntry {
    node: NodeId,
    name: DataId,
    subscribers: Vec<String>,
    zenoh_key: String,
}

async fn list(
    coordinator: CoordinatorOptions,
    selector: DataflowSelector,
    format: OutputFormat,
) -> eyre::Result<()> {
    let client = coordinator.connect_rpc().await?;
    let (dataflow_id, descriptor) = selector.resolve(&client).await?;

    let mut subscribers = BTreeMap::<(&NodeId, &DataId), Vec<(&NodeId, &DataId)>>::new();
    for node in &descriptor.nodes {
        for (input_id, input) in &node.inputs {
            if let InputMapping::User(user) = &input.mapping {
                subscribers
                    .entry((&user.source, &user.output))
                    .or_default()
                    .push((&node.id, input_id));
            }
        }
    }

    let mut entries = Vec::new();
    for node in &descriptor.nodes {
        for output in &node.outputs {
            entries.push(OutputEntry {
                node: node.id.clone(),
                name: output.clone(),
                subscribers: subscribers
                    .remove(&(&node.id, output))
                    .unwrap_or_default()
                    .into_iter()
                    .map(|(node, data)| format!("{node}/{data}"))
                    .collect(),
                zenoh_key: zenoh_output_publish_topic(dataflow_id, &node.id, output),
            });
        }
    }

    match format {
        OutputFormat::Table => {
            let mut tw = TabWriter::new(std::io::stdout().lock());
            tw.write_all(b"Node\tName\tZenoh Key\tSubscribers\n")?;
            for entry in entries {
                tw.write_all(
                    format!(
                        "{}\t{}\t{}\t{}\n",
                        entry.node,
                        entry.name,
                        entry.zenoh_key,
                        entry.subscribers.join(", ")
                    )
                    .as_bytes(),
                )?;
            }
            tw.flush()?;
        }
        OutputFormat::Json => {
            for entry in entries {
                println!("{}", serde_json::to_string(&entry)?);
            }
        }
    }

    Ok(())
}
