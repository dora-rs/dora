use anyhow::{Context, bail};
use clap::{Parser, Subcommand};
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Parser)]
#[command(name = "xtask", about = "Dora task runner")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Stage C node library files from build output to prefix directory
    StageCNode {
        /// Cargo build target directory (e.g., target/release)
        target_dir: PathBuf,

        /// Staging output directory (e.g., dora-c-libraries-x86_64-unknown-linux-gnu)
        prefix: PathBuf,
    },
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::StageCNode { target_dir, prefix } => {
            stage_c_node(&target_dir, &prefix)?;
        }
    }

    Ok(())
}

fn stage_c_node(target_dir: &Path, prefix: &Path) -> anyhow::Result<()> {
    let staging = target_dir.join("dora-node-api-c");

    if !staging.exists() {
        bail!(
            "Staging directory not found: {}\n\
             Ensure dora-node-api-c has been built (cargo build -p dora-node-api-c).",
            staging.display()
        );
    }

    let cmake_src = staging.join("lib/cmake/dora-node-api-c");
    let header_src = staging.join("include/node_api.h");

    if !cmake_src.exists() {
        bail!("CMake config directory not found: {}", cmake_src.display());
    }
    if !header_src.exists() {
        bail!("Header file not found: {}", header_src.display());
    }

    println!("Staging C node library files...");
    println!("  TARGET_DIR: {}", target_dir.display());
    println!("  STAGING:    {}", staging.display());
    println!("  PREFIX:     {}", prefix.display());

    let lib_cmake_dir = prefix.join("lib/cmake");
    let include_dir = prefix.join("include");
    fs::create_dir_all(&lib_cmake_dir)?;
    fs::create_dir_all(&include_dir)?;

    copy_dir_contents(&cmake_src, &lib_cmake_dir.join("dora-node-api-c"))
        .context("Failed to copy cmake config files")?;

    fs::copy(&header_src, include_dir.join("node_api.h")).context("Failed to copy header file")?;

    copy_library(target_dir, prefix)?;

    println!("Staged files:");
    if let Ok(entries) = fs::read_dir(prefix.join("lib")) {
        for entry in entries.filter_map(|e| e.ok()) {
            println!("  lib/{}", entry.file_name().to_string_lossy());
        }
    }
    if let Ok(entries) = fs::read_dir(&include_dir) {
        for entry in entries.filter_map(|e| e.ok()) {
            println!("  include/{}", entry.file_name().to_string_lossy());
        }
    }
    let cmake_dir = lib_cmake_dir.join("dora-node-api-c");
    if let Ok(entries) = fs::read_dir(&cmake_dir) {
        for entry in entries.filter_map(|e| e.ok()) {
            println!(
                "  lib/cmake/dora-node-api-c/{}",
                entry.file_name().to_string_lossy()
            );
        }
    }

    Ok(())
}

fn copy_dir_contents(src: &Path, dst: &Path) -> anyhow::Result<()> {
    fs::create_dir_all(dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let file_type = entry.file_type()?;
        let dst_path = dst.join(entry.file_name());

        if file_type.is_dir() {
            copy_dir_contents(&entry.path(), &dst_path)?;
        } else {
            fs::copy(entry.path(), &dst_path)?;
        }
    }
    Ok(())
}

fn copy_library(target_dir: &Path, prefix: &Path) -> anyhow::Result<()> {
    #[cfg(unix)]
    let lib_name = "libdora_node_api_c.a";

    #[cfg(windows)]
    let lib_name = "dora_node_api_c.lib";

    #[cfg(not(any(unix, windows)))]
    let lib_name = "libdora_node_api_c.a";

    let lib_src = target_dir.join(lib_name);
    let lib_dst = prefix.join("lib").join(lib_name);

    if lib_src.exists() {
        fs::copy(&lib_src, &lib_dst).context(format!(
            "Failed to copy library {} -> {}",
            lib_src.display(),
            lib_dst.display()
        ))?;
    } else {
        eprintln!("WARNING: Library file not found at {}", lib_src.display());
    }

    Ok(())
}
