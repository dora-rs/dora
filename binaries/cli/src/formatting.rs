use dora_core::topics::{DataflowResult, NodeErrorCause};

pub struct FormatDataflowError<'a>(pub &'a DataflowResult);

impl std::fmt::Display for FormatDataflowError<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
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
        // try to print earliest non-cascading error
        if !non_cascading.is_empty() {
            for (id, err) in non_cascading {
                writeln!(f, "Node `{id}` failed: {err}")?;
            }
        } else {
            // no non-cascading errors -> print earliest cascading
            let mut all: Vec<_> = failed.collect();
            all.sort_by_key(|(_, e)| e.timestamp);
            if let Some((id, err)) = all.first() {
                write!(f, "Node `{id}` failed: {err}")?;
            } else {
                write!(f, "unknown error")?;
            }
        }

        if total_failed > 1 {
            write!(
                f,
                "\n\nThere are {} more errors. Check the `out/{}` folder for full details.",
                total_failed - 1,
                self.0.uuid
            )?;
        }

        Ok(())
    }
}
