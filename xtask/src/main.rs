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
    /// Stage library files from build output to prefix directory
    Stage {
        /// Crate name (e.g., dora-node-api-c, dora-operator-api-c, dora-node-api-cxx, dora-operator-api-cxx)
        crate_name: String,

        /// Cargo build target directory (e.g., target/release)
        target_dir: PathBuf,

        /// Staging output directory (e.g., dora-c-libraries-x86_64-unknown-linux-gnu)
        prefix: PathBuf,
    },
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Stage {
            crate_name,
            target_dir,
            prefix,
        } => {
            stage_library(&crate_name, &target_dir, &prefix)?;
        }
    }

    Ok(())
}

fn is_cxx_crate(crate_name: &str) -> bool {
    crate_name.ends_with("-cxx")
}

fn stage_library(crate_name: &str, target_dir: &Path, prefix: &Path) -> anyhow::Result<()> {
    if is_cxx_crate(crate_name) {
        stage_cxx_crate(crate_name, target_dir, prefix)
    } else {
        stage_c_crate(crate_name, target_dir, prefix)
    }
}

fn stage_c_crate(crate_name: &str, target_dir: &Path, prefix: &Path) -> anyhow::Result<()> {
    let staging = target_dir.join(crate_name);

    if !staging.exists() {
        bail!(
            "Staging directory not found: {}\n\
             Ensure {crate_name} has been built (cargo build -p {crate_name}).",
            staging.display()
        );
    }

    let cmake_src = staging.join("lib/cmake").join(crate_name);
    let include_src = staging.join("include");

    if !cmake_src.exists() {
        bail!("CMake config directory not found: {}", cmake_src.display());
    }
    if !include_src.exists() {
        bail!("Include directory not found: {}", include_src.display());
    }

    println!("Staging {crate_name} (C crate)...");
    println!("  TARGET_DIR: {}", target_dir.display());
    println!("  STAGING:    {}", staging.display());
    println!("  PREFIX:     {}", prefix.display());

    let lib_cmake_dir = prefix.join("lib/cmake");
    let include_dir = prefix.join("include");
    fs::create_dir_all(&lib_cmake_dir)?;
    fs::create_dir_all(&include_dir)?;

    copy_dir_contents(&cmake_src, &lib_cmake_dir.join(crate_name))
        .with_context(|| format!("Failed to copy cmake config for {crate_name}"))?;

    copy_dir_contents(&include_src, &include_dir)
        .with_context(|| format!("Failed to copy headers for {crate_name}"))?;

    copy_library(crate_name, target_dir, prefix)?;

    print_staged_files(prefix, crate_name);

    Ok(())
}

fn stage_cxx_crate(crate_name: &str, target_dir: &Path, prefix: &Path) -> anyhow::Result<()> {
    let workspace_root = target_dir
        .parent()
        .and_then(Path::parent)
        .unwrap_or(target_dir);
    let cxxbridge_dir = workspace_root.join("target/cxxbridge").join(crate_name);

    if !cxxbridge_dir.exists() {
        bail!(
            "Cxxbridge directory not found: {}\n\
             Ensure {crate_name} has been built with host cargo (cargo build -p {crate_name}).",
            cxxbridge_dir.display()
        );
    }

    let cmake_src = cxxbridge_dir.join("lib/cmake").join(crate_name);
    let include_src = cxxbridge_dir.join("include");

    if !cmake_src.exists() {
        bail!("CMake config directory not found: {}", cmake_src.display());
    }
    if !include_src.exists() {
        bail!("Include directory not found: {}", include_src.display());
    }

    println!("Staging {crate_name} (C++ crate)...");
    println!("  CXXBRIDGE:  {}", cxxbridge_dir.display());
    println!("  TARGET_DIR: {}", target_dir.display());
    println!("  PREFIX:     {}", prefix.display());

    let lib_cmake_dir = prefix.join("lib/cmake");
    let include_dir = prefix.join("include");
    fs::create_dir_all(&lib_cmake_dir)?;
    fs::create_dir_all(&include_dir)?;

    copy_dir_contents(&cmake_src, &lib_cmake_dir.join(crate_name))
        .with_context(|| format!("Failed to copy cmake config for {crate_name}"))?;

    copy_dir_contents(&include_src, &include_dir)
        .with_context(|| format!("Failed to copy headers for {crate_name}"))?;

    copy_library(crate_name, target_dir, prefix)?;

    print_staged_files(prefix, crate_name);

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

fn copy_library(crate_name: &str, target_dir: &Path, prefix: &Path) -> anyhow::Result<()> {
    let lib_name = match crate_name {
        "dora-node-api-c" => {
            if cfg!(windows) {
                "dora_node_api_c.lib"
            } else {
                "libdora_node_api_c.a"
            }
        }
        "dora-operator-api-c" => {
            if cfg!(windows) {
                "dora_operator_api_c.lib"
            } else {
                "libdora_operator_api_c.a"
            }
        }
        "dora-node-api-cxx" => {
            if cfg!(windows) {
                "dora_node_api_cxx.lib"
            } else {
                "libdora_node_api_cxx.a"
            }
        }
        "dora-operator-api-cxx" => {
            if cfg!(windows) {
                "dora_operator_api_cxx.lib"
            } else {
                "libdora_operator_api_cxx.a"
            }
        }
        _ => {
            eprintln!("WARNING: Unknown crate {crate_name}, skipping library copy");
            return Ok(());
        }
    };

    let lib_src = target_dir.join(lib_name);
    let lib_dst = prefix.join("lib").join(lib_name);

    if lib_src.exists() {
        fs::copy(&lib_src, &lib_dst).with_context(|| {
            format!(
                "Failed to copy library {} -> {}",
                lib_src.display(),
                lib_dst.display()
            )
        })?;
    } else {
        eprintln!("WARNING: Library file not found at {}", lib_src.display());
    }

    Ok(())
}

fn print_staged_files(prefix: &Path, crate_name: &str) {
    println!("Staged files:");
    if let Ok(entries) = fs::read_dir(prefix.join("lib")) {
        for entry in entries.filter_map(|e| e.ok()) {
            println!("  lib/{}", entry.file_name().to_string_lossy());
        }
    }
    if let Ok(entries) = fs::read_dir(prefix.join("include")) {
        for entry in entries.filter_map(|e| e.ok()) {
            println!("  include/{}", entry.file_name().to_string_lossy());
        }
    }
    let cmake_dir = prefix.join("lib/cmake").join(crate_name);
    if let Ok(entries) = fs::read_dir(&cmake_dir) {
        for entry in entries.filter_map(|e| e.ok()) {
            println!(
                "  lib/cmake/{crate_name}/{}",
                entry.file_name().to_string_lossy()
            );
        }
    }
}
