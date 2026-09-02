//! Pins the `dora` command surface, which dora 1.x freezes.
//!
//! `docs/api-rust.md` puts `dora-cli` inside the 1.0 guarantee: "the `dora`
//! command, its subcommands, and the dataflow YAML schema". Nothing checked
//! that. `cargo-semver-checks` cannot see a CLI -- the crate's Rust lib target
//! is internal, and a deleted subcommand or a flag that grew a required value
//! is invisible to it -- so a break here would have reached users' scripts
//! with no gate having failed.
//!
//! The snapshot this test maintains is what makes the surface checkable
//! *without* building the CLI: `scripts/qa/breaking-changes.sh` diffs the
//! checked-in file against the one released with the baseline tag, which is
//! why that gate can run on every PR in seconds. This test is the other half
//! of the deal -- it keeps the file honest, so the cheap diff is comparing the
//! real surface rather than a stale copy of it.
//!
//! Adding a command or a flag is fine; the snapshot just has to be updated in
//! the same commit, which is what puts the change in front of a reviewer:
//!
//! ```text
//! UPDATE_CLI_SURFACE=1 cargo test -p dora-cli --test cli_surface
//! ```

use std::{collections::BTreeSet, path::PathBuf};

use clap::{Command, CommandFactory};
use dora_cli::Args;

const SNAPSHOT: &str = "cli-surface.txt";

const HEADER: &str = "\
# The `dora` command surface, frozen for dora 1.x (docs/api-rust.md).
#
# Generated -- do not edit by hand:
#     UPDATE_CLI_SURFACE=1 cargo test -p dora-cli --test cli_surface
#
# One line per command, then one per argument. Hidden commands are included:
# `dora daemon` and `dora coordinator` are undocumented but scripted against.
# Removing any line is a breaking change; adding one is not.
";

/// Every line of the surface, sorted so the file diffs cleanly.
fn surface() -> String {
    let mut lines = BTreeSet::new();
    walk(&Args::command(), "dora", &mut lines);
    let body: Vec<_> = lines.into_iter().collect();
    format!("{HEADER}\n{}\n", body.join("\n"))
}

fn walk(command: &Command, path: &str, lines: &mut BTreeSet<String>) {
    lines.insert(path.to_owned());
    for alias in command.get_all_aliases() {
        lines.insert(format!("{path} | alias: {alias}"));
    }
    for arg in command.get_arguments() {
        lines.insert(format!("{path} | {}", describe(arg)));
    }
    for sub in command.get_subcommands() {
        walk(sub, &format!("{path} {}", sub.get_name()), lines);
    }
}

/// One argument, in the terms a caller's script depends on: how it is spelled,
/// whether it takes a value, and whether it may be omitted.
fn describe(arg: &clap::Arg) -> String {
    let mut spelling = Vec::new();
    if let Some(short) = arg.get_short() {
        spelling.push(format!("-{short}"));
    }
    if let Some(long) = arg.get_long() {
        spelling.push(format!("--{long}"));
    }
    let mut item = if spelling.is_empty() {
        format!("<{}>", arg.get_id())
    } else {
        spelling.join("/")
    };
    // An option that starts or stops taking a value breaks every existing
    // invocation, so the value is part of the identity, not a detail. Read it
    // off the action rather than `get_num_args`, which is only populated once
    // clap has built the command -- and `Args::command()` hands back an
    // unbuilt one, so every option would look like a bare flag.
    if arg.get_action().takes_values() && arg.get_long().is_some() {
        item.push_str(" <value>");
    }
    if arg.is_required_set() {
        item.push_str(" (required)");
    }
    item
}

fn snapshot_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(SNAPSHOT)
}

#[test]
fn cli_surface_snapshot_is_current() {
    let path = snapshot_path();
    let current = surface();

    if std::env::var_os("UPDATE_CLI_SURFACE").is_some() {
        std::fs::write(&path, &current).expect("failed to write the CLI surface snapshot");
        return;
    }

    let recorded = std::fs::read_to_string(&path).unwrap_or_else(|error| {
        panic!(
            "cannot read {}: {error}\n\
             Generate it with: UPDATE_CLI_SURFACE=1 cargo test -p dora-cli --test cli_surface",
            path.display()
        )
    });

    if recorded != current {
        let recorded_lines: BTreeSet<_> = recorded.lines().filter(|l| keep(l)).collect();
        let current_lines: BTreeSet<_> = current.lines().filter(|l| keep(l)).collect();
        let removed: Vec<_> = recorded_lines.difference(&current_lines).collect();
        let added: Vec<_> = current_lines.difference(&recorded_lines).collect();
        panic!(
            "the `dora` command surface changed.\n\n\
             removed (breaking -- dora 1.x froze these):\n  {}\n\n\
             added (fine, but record it):\n  {}\n\n\
             Update the snapshot in this commit so the change is reviewed:\n  \
             UPDATE_CLI_SURFACE=1 cargo test -p dora-cli --test cli_surface",
            if removed.is_empty() {
                "(none)".to_owned()
            } else {
                removed
                    .iter()
                    .map(|l| l.to_string())
                    .collect::<Vec<_>>()
                    .join("\n  ")
            },
            if added.is_empty() {
                "(none)".to_owned()
            } else {
                added
                    .iter()
                    .map(|l| l.to_string())
                    .collect::<Vec<_>>()
                    .join("\n  ")
            },
        );
    }
}

fn keep(line: &str) -> bool {
    !line.trim().is_empty() && !line.trim_start().starts_with('#')
}

/// The snapshot is only worth anything if it covers the whole tree, so guard
/// the two ways it could quietly shrink to nothing.
#[test]
fn snapshot_covers_the_command_tree() {
    let text = surface();
    for command in [
        "dora build",
        "dora start",
        "dora run",
        "dora up",
        "dora daemon",
    ] {
        assert!(
            text.lines().any(|line| line == command),
            "`{command}` is missing from the generated surface"
        );
    }
    assert!(
        text.lines().filter(|l| keep(l)).count() > 100,
        "the generated surface is implausibly small: the walk is not reaching the tree"
    );
}
