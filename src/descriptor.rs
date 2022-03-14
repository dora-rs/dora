use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap, HashSet},
    hash::Hash,
};

#[derive(Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Descriptor {
    #[serde(default)]
    sources: HashSet<Source>,
    #[serde(default)]
    sinks: HashSet<Sink>,
    #[serde(default)]
    operators: HashSet<Operator>,
}

impl Descriptor {
    pub fn visualize_as_mermaid(&self) -> eyre::Result<String> {
        let mut flowchart = "flowchart TB\n".to_owned();
        for source in &self.sources {
            let id = &source.id;
            flowchart.push_str(&format!("  {id}[\\{id}/]\n"));
        }
        for operator in &self.operators {
            let id = &operator.id;
            flowchart.push_str(&format!("  {id}\n"));
        }
        for sink in &self.sinks {
            let id = &sink.id;
            flowchart.push_str(&format!("  {id}[/{id}\\]\n"));
        }

        let mut expected_inputs: HashMap<_, BTreeSet<_>> = HashMap::new();
        for operator in &self.operators {
            for input in operator.inputs.values().into_iter() {
                expected_inputs
                    .entry(input.to_owned())
                    .or_default()
                    .insert(&operator.id);
            }
        }
        for sink in &self.sinks {
            for input in sink.inputs.values().into_iter() {
                expected_inputs
                    .entry(input.to_owned())
                    .or_default()
                    .insert(&sink.id);
            }
        }

        for source in &self.sources {
            for output in source.outputs.values().into_iter() {
            let targets = expected_inputs.remove(output).unwrap_or_default();
            let id = &source.id;
            for target in targets {
                flowchart.push_str(&format!("  {id} -- {output} --> {target}\n"));
            }

            }
        }

        for operator in &self.operators {
            let id = &operator.id;
            for output in operator.outputs.values().into_iter() {
                let targets = expected_inputs.remove(output).unwrap_or_default();
                for target in targets {
                    flowchart.push_str(&format!("  {id} -- {output} --> {target}\n"));
                }
            }
        }

        for (output, targets) in expected_inputs.drain() {
            for target in targets {
                flowchart.push_str(&format!("  missing>missing] -- {output} --> {target}\n"));
            }
        }

        Ok(flowchart)
    }
}

#[derive(Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Source {
    id: String,
    outputs: BTreeMap<String, String>,
}

#[derive(Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Sink {
    id: String,
    inputs: BTreeMap<String, String>,
}

#[derive(Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Operator {
    id: String,
    inputs: BTreeMap<String, String>,
    outputs: BTreeMap<String, String>,
}
