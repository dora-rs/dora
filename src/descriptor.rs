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
            let label = &source.label;
            flowchart.push_str(&format!("  {label}[\\{label}/]\n"));
        }
        for operator in &self.operators {
            let label = &operator.label;
            flowchart.push_str(&format!("  {label}\n"));
        }
        for sink in &self.sinks {
            let label = &sink.label;
            flowchart.push_str(&format!("  {label}[/{label}\\]\n"));
        }

        let mut expected_inputs: HashMap<_, BTreeSet<_>> = HashMap::new();
        for operator in &self.operators {
            for input in operator.inputs.values().into_iter() {
                expected_inputs
                    .entry(input.to_owned())
                    .or_default()
                    .insert(&operator.label);
            }
        }
        for sink in &self.sinks {
            for input in sink.inputs.values().into_iter() {
                expected_inputs
                    .entry(input.to_owned())
                    .or_default()
                    .insert(&sink.label);
            }
        }

        for source in &self.sources {
            for output in source.outputs.values().into_iter() {
            let targets = expected_inputs.remove(output).unwrap_or_default();
            let label = &source.label;
            for target in targets {
                flowchart.push_str(&format!("  {label} -- {output} --> {target}\n"));
            }

            }
        }

        for operator in &self.operators {
            let label = &operator.label;
            for output in operator.outputs.values().into_iter() {
                let targets = expected_inputs.remove(output).unwrap_or_default();
                for target in targets {
                    flowchart.push_str(&format!("  {label} -- {output} --> {target}\n"));
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
    label: String,
    outputs: BTreeMap<String, String>,
}

#[derive(Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Sink {
    label: String,
    inputs: BTreeMap<String, String>,
}

#[derive(Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Operator {
    label: String,
    inputs: BTreeMap<String, String>,
    outputs: BTreeMap<String, String>,
}
