use clap::{CommandFactory, ValueEnum};
use clap_complete::Shell;

use crate::command::Executable;
use sysinfo;

#[derive(Debug, clap::Args)]
#[command(after_help = r#"
USAGE:
  eval $(dora completion)              # Auto-detect shell
  eval $(dora completion <SHELL>)      # Specify shell explicitly

PERSIST COMPLETION:

  Bash:
    echo 'eval "$(dora completion bash)"' >> ~/.bashrc
    # Then restart shell

  Zsh:
    echo 'eval "$(dora completion zsh)"' >> ~/.zshrc
    # Then restart shell

    # If you get 'command not found: compdef', add this before the eval line:
    autoload -Uz compinit
    compinit

  Fish:
    # Add to ~/.config/fish/config.fish
    if status is-interactive
        eval "$(dora completion fish)"
    end
    # Then restart shell

"#)]
pub struct Completion {
    /// The shell to generate the script for
    #[arg(value_enum)]
    shell: Option<Shell>,
}
impl Executable for Completion {
    fn execute(self) -> eyre::Result<()> {
        let shell = if let Some(sh) = self.shell {
            sh
        } else {
            get_shell().map_err(|e| {
                eyre::eyre!(
                    "Please specify the shell via parameter. Unable to get the current shell. ({})",
                    e
                )
            })?
        };
        clap_complete::generate(
            shell,
            &mut crate::Args::command(),
            "dora",
            &mut std::io::stdout(),
        );
        Ok(())
    }
}

fn get_shell() -> eyre::Result<Shell> {
    let pid =
        sysinfo::get_current_pid().map_err(|_| eyre::eyre!("Unable to get the current PID"))?;
    let system = sysinfo::System::new_all();
    let process = system
        .process(pid)
        .ok_or(eyre::eyre!("Unable to get the current process"))?;
    let parent_pid = process
        .parent()
        .ok_or(eyre::eyre!("Unable to get the parent process PID"))?;
    let parent_process = system
        .process(parent_pid)
        .ok_or(eyre::eyre!("Unable to get the parent process"))?;
    let current_shell = Shell::from_str(
        parent_process
            .name()
            .to_str()
            .ok_or(eyre::eyre!("Unable to get the shell name"))?,
        true,
    )
    .map_err(|_| eyre::eyre!("Unable to get the shell type"))?;
    Ok(current_shell)
}
