use clap::ValueEnum;
use dora_message::{common::NodeErrorCause, coordinator_to_cli::DataflowResult};

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum OutputFormat {
    Table,
    Json,
}

impl std::fmt::Display for OutputFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OutputFormat::Table => write!(f, "table"),
            OutputFormat::Json => write!(f, "json"),
        }
    }
}

pub struct FormatDataflowError<'a>(pub &'a DataflowResult);

impl std::fmt::Display for FormatDataflowError<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f)?;
        let failed = self
            .0
            .node_results
            .iter()
            .filter_map(|(id, r)| r.as_ref().err().map(|e| (id, e)));
        let total_failed = failed.clone().count();

        let mut non_cascading: Vec<_> = failed
            .clone()
            .filter(|(_, e)| !matches!(e.cause, NodeErrorCause::Cascading { .. }))
            .collect();
        non_cascading.sort_by_key(|(_, e)| e.timestamp);
        // print every non-cascading (root-cause) error; if there are none, fall
        // back to the earliest cascading error
        let hidden = if !non_cascading.is_empty() {
            let printed = non_cascading.len();
            for (id, err) in non_cascading {
                writeln!(f, "Node `{id}` failed: {err}")?;
            }
            total_failed - printed
        } else {
            // no non-cascading errors -> print earliest cascading
            let mut all: Vec<_> = failed.collect();
            all.sort_by_key(|(_, e)| e.timestamp);
            if let Some((id, err)) = all.first() {
                write!(f, "Node `{id}` failed: {err}")?;
                total_failed - 1
            } else {
                write!(f, "unknown error")?;
                0
            }
        };

        if hidden >= 1 {
            let noun = if hidden == 1 { "error" } else { "errors" };
            write!(
                f,
                "\n\nThere {} {hidden} more consequential {noun}. Check the `out/{}` folder for full details.",
                if hidden == 1 { "is" } else { "are" },
                self.0.uuid
            )?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_message::{
        common::{NodeError, NodeExitStatus},
        id::NodeId,
        uhlc::HLC,
    };
    use std::collections::BTreeMap;
    use uuid::Uuid;

    fn cascading_error(hlc: &HLC, caused_by: &str) -> NodeError {
        NodeError {
            timestamp: hlc.new_timestamp(),
            cause: NodeErrorCause::Cascading {
                caused_by_node: caused_by.to_string().into(),
            },
            exit_status: NodeExitStatus::ExitCode(1),
        }
    }

    #[test]
    fn all_cascading_two_failures_reports_the_hidden_one() {
        let hlc = HLC::default();
        let mut node_results: BTreeMap<NodeId, Result<(), NodeError>> = BTreeMap::new();
        node_results.insert("a".to_string().into(), Err(cascading_error(&hlc, "root")));
        node_results.insert("b".to_string().into(), Err(cascading_error(&hlc, "root")));

        let result = DataflowResult {
            uuid: Uuid::nil(),
            timestamp: hlc.new_timestamp(),
            node_results,
        };

        let rendered = FormatDataflowError(&result).to_string();

        // Exactly one of the two failures is shown inline...
        assert_eq!(
            rendered.matches("failed:").count(),
            1,
            "expected a single inline failure, got:\n{rendered}"
        );
        // ...but the user must still be told the second failure exists and
        // pointed at the `out/` folder (this is the case #2736 dropped).
        assert!(
            rendered.contains("1 more consequential error")
                && !rendered.contains("consequential errors"),
            "expected a singular summary line, got:\n{rendered}"
        );
        assert!(
            rendered.contains("out/"),
            "expected a pointer to the out/ folder, got:\n{rendered}"
        );
    }

    #[test]
    fn all_cascading_three_failures_uses_plural_summary() {
        let hlc = HLC::default();
        let mut node_results: BTreeMap<NodeId, Result<(), NodeError>> = BTreeMap::new();
        node_results.insert("a".to_string().into(), Err(cascading_error(&hlc, "root")));
        node_results.insert("b".to_string().into(), Err(cascading_error(&hlc, "root")));
        node_results.insert("c".to_string().into(), Err(cascading_error(&hlc, "root")));

        let result = DataflowResult {
            uuid: Uuid::nil(),
            timestamp: hlc.new_timestamp(),
            node_results,
        };

        let rendered = FormatDataflowError(&result).to_string();
        assert!(
            rendered.contains("2 more consequential errors"),
            "expected a plural summary line, got:\n{rendered}"
        );
    }
}
