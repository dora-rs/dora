use dora_api::config::{CommunicationConfig, InputMapping, OperatorConfig};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Descriptor {
    pub communication: CommunicationConfig,
    pub operators: Vec<Operator>,
}

impl Descriptor {
    pub fn sources(&self) -> impl Iterator<Item = &OperatorConfig> {
        self.operators
            .iter()
            .map(|o| &o.config)
            .filter(|o| o.inputs.is_empty())
    }

    pub fn sinks(&self) -> impl Iterator<Item = &OperatorConfig> {
        self.operators
            .iter()
            .map(|o| &o.config)
            .filter(|o| o.outputs.is_empty())
    }

    pub fn actions(&self) -> impl Iterator<Item = &OperatorConfig> {
        self.operators
            .iter()
            .map(|o| &o.config)
            .filter(|o| !o.inputs.is_empty() && !o.outputs.is_empty())
    }

    pub fn visualize_as_mermaid(&self) -> eyre::Result<String> {
        let mut flowchart = "flowchart TB\n".to_owned();
        for source in self.sources() {
            let id = &source.id;
            flowchart.push_str(&format!("  {id}[\\{id}/]\n"));
        }
        for action in self.actions() {
            let id = &action.id;
            flowchart.push_str(&format!("  {id}\n"));
        }
        for sink in self.sinks() {
            let id = &sink.id;
            flowchart.push_str(&format!("  {id}[/{id}\\]\n"));
        }

        let operators: HashMap<_, _> = self
            .operators
            .iter()
            .map(|o| (&o.config.id, &o.config))
            .collect();

        for operator in &self.operators {
            let operator = &operator.config;
            let id = &operator.id;
            for (input_id, InputMapping { source, output }) in &operator.inputs {
                if operators
                    .get(source)
                    .map(|source| source.outputs.contains(output))
                    .unwrap_or(false)
                {
                    let data = if output == input_id {
                        format!("{output}")
                    } else {
                        format!("{output} as {input_id}")
                    };
                    flowchart.push_str(&format!("  {source} -- {data} --> {id}\n"));
                } else {
                    flowchart.push_str(&format!("  missing>missing] -- {input_id} --> {id}\n"));
                }
            }
        }

        Ok(flowchart)
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Operator {
    #[serde(flatten)]
    pub config: OperatorConfig,
    pub run: String,
}
