use std::{collections::BTreeMap, io::Write};

use clap::Args;
use dora_core::config::InputMapping;
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
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        list(self.coordinator, self.selector, self.format)
    }
}

#[derive(Serialize)]
struct OutputEntry {
    node: NodeId,
    name: DataId,
    subscribers: Vec<String>,
}

fn list(
    coordinator: CoordinatorOptions,
    selector: DataflowSelector,
    format: OutputFormat,
) -> eyre::Result<()> {
    let mut session = coordinator.connect()?;
    let (_dataflow_id, descriptor) = selector.resolve(session.as_mut())?;

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
            });
        }
    }

    match format {
        OutputFormat::Table => {
            let mut tw = TabWriter::new(std::io::stdout().lock());
            tw.write_all(b"Node\tName\tSubscribers\n")?;
            for entry in entries {
                tw.write_all(
                    format!(
                        "{}\t{}\t{}\n",
                        entry.node,
                        entry.name,
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
