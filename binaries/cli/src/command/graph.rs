use super::Executable;
use dora_core::descriptor::{Descriptor, DescriptorExt};
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, NodeStatus},
    id::NodeId,
};
use eyre::Context;
use std::{
    collections::HashMap,
    fs::File,
    io::Write,
    path::{Path, PathBuf},
};

const MERMAID_TEMPLATE: &str = include_str!("graph/mermaid-template.html");

#[derive(Debug, clap::Args)]
/// Generate a visualization of the given graph using mermaid.js. Use --open to open browser.
pub struct Graph {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    dataflow: PathBuf,
    /// Visualize the dataflow as a Mermaid diagram (instead of HTML)
    #[clap(long, action)]
    mermaid: bool,
    /// Open the HTML visualization in the browser
    #[clap(long, action)]
    open: bool,
    /// Live mode with auto-refresh
    #[clap(long, action)]
    live: bool,
    /// Output file path
    #[clap(long, short = 'o')]
    output: Option<PathBuf>,
}

impl Executable for Graph {
    fn execute(self) -> eyre::Result<()> {
        create(
            self.dataflow,
            self.mermaid,
            self.open,
            self.output,
            self.live,
        )
    }
}

fn create(
    dataflow: std::path::PathBuf,
    mermaid: bool,
    open: bool,
    output: Option<PathBuf>,
    live: bool,
) -> eyre::Result<()> {
    if mermaid {
        let visualized = visualize_as_mermaid(&dataflow)?;
        println!("{visualized}");
        println!(
            "Paste the above output on https://mermaid.live/ or in a \
            ```mermaid code block on GitHub to display it."
        );
    } else {
        let mut html = visualize_as_html(&dataflow)?;
        if live {
            html = html.replace(
                "<head>",
                "<head>\n    <meta http-equiv=\"refresh\" content=\"2\">",
            );
        }

        let path = output.unwrap_or_else(|| {
            let working_dir = std::env::current_dir().unwrap();
            let graph_filename = match dataflow.file_stem().and_then(|n| n.to_str()) {
                Some(name) => format!("{name}-graph"),
                None => "graph".into(),
            };
            let mut extra = 0;
            loop {
                let adjusted_file_name = if extra == 0 {
                    format!("{graph_filename}.html")
                } else {
                    format!("{graph_filename}.{extra}.html")
                };
                let p = working_dir.join(&adjusted_file_name);
                if p.exists() {
                    extra += 1;
                } else {
                    break p;
                }
            }
        });

        let mut file = File::create(&path).context("failed to create graph HTML file")?;
        file.write_all(html.as_bytes())?;

        println!(
            "View graph by opening the following in your browser:\n  file://{}",
            path.display()
        );

        if open {
            webbrowser::open(path.as_os_str().to_str().unwrap())?;
        }
    }
    Ok(())
}

pub fn visualize_as_html(dataflow: &Path) -> eyre::Result<String> {
    let mermaid = visualize_as_mermaid(dataflow)?;
    Ok(MERMAID_TEMPLATE.replacen("____insert____", &mermaid, 1))
}

pub fn visualize_as_mermaid(dataflow: &Path) -> eyre::Result<String> {
    let descriptor = Descriptor::blocking_read(dataflow)
        .with_context(|| format!("failed to read dataflow at `{}`", dataflow.display()))?;
    let visualized = descriptor
        .visualize_as_mermaid()
        .context("failed to visualize descriptor")?;

    Ok(visualized)
}
