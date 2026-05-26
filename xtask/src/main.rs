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

        /// Cargo target triple this prefix is being staged for
        /// (e.g., `aarch64-unknown-linux-gnu`). Required for cross-
        /// compiled cxx archives: the cxxbridge `build.rs` may run on
        /// the host (encoding `linux-x86_64` into the cmake config),
        /// so xtask rewrites `DORA_BUILT_TARGET` at stage time to
        /// match the actual archive triple. Without this, a consumer
        /// on aarch64 hits the cmake `FATAL_ERROR` target-match check
        /// even though the lib is correct for their platform.
        #[arg(long)]
        target: Option<String>,
    },
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Stage {
            crate_name,
            target_dir,
            prefix,
            target,
        } => {
            stage_library(&crate_name, &target_dir, &prefix, target.as_deref())?;
        }
    }

    Ok(())
}

fn is_cxx_crate(crate_name: &str) -> bool {
    crate_name.ends_with("-cxx")
}

fn stage_library(
    crate_name: &str,
    target_dir: &Path,
    prefix: &Path,
    target_triple: Option<&str>,
) -> anyhow::Result<()> {
    if is_cxx_crate(crate_name) {
        stage_cxx_crate(crate_name, target_dir, prefix)?;
    } else {
        stage_c_crate(crate_name, target_dir, prefix)?;
    }
    if let Some(triple) = target_triple {
        rewrite_built_target(prefix, crate_name, triple).with_context(|| {
            format!("Failed to rewrite DORA_BUILT_TARGET for {crate_name} ({triple})")
        })?;
    }
    Ok(())
}

/// Translate a cargo target triple into the `<os>-<arch>` shape the
/// cmake config template uses. Mirrors `compute_target()` in each
/// `build.rs`, but driven by an explicit string instead of cargo's
/// `CARGO_CFG_TARGET_*` (which encodes the *host* on cross-cxx builds).
///
/// Currently supports only the four targets in
/// `.github/workflows/publish-c-cpp-libraries.yml`. Two known caveats
/// for extending the matrix:
///   - **Android triples** (e.g. `aarch64-linux-android`) literally
///     contain `linux` and would silently map to `linux-<arch>` if
///     the Android check did not come first — the Bionic-vs-glibc
///     ABI gap would then only surface at the consumer's link step.
///     The check ordering below is load-bearing.
///   - **armv7** triples normalize to `arm` in `CARGO_CFG_TARGET_ARCH`
///     but the raw first triple component is `armv7`. Adding armv7
///     to the publish matrix needs both a lookup-table entry here
///     and a matching arch normaliser in `dora-api-config.cmake.in`
///     (today's `else → ${CMAKE_SYSTEM_PROCESSOR}` lets a raw
///     `armv7l` from `CMAKE_SYSTEM_PROCESSOR` flow through, which
///     won't match `armv7` either).
fn cargo_triple_to_cmake_target(triple: &str) -> anyhow::Result<String> {
    let arch = triple
        .split('-')
        .next()
        .ok_or_else(|| anyhow::anyhow!("empty target triple"))?;
    // Order matters: `android` must be checked before `linux` because
    // Android triples contain both substrings (`*-linux-android*`).
    let os = if triple.contains("android") {
        bail!("Android target `{triple}` not supported — Bionic libc differs from glibc/musl");
    } else if triple.contains("ios") {
        bail!("iOS target `{triple}` not supported — extend cmake template's platform map");
    } else if triple.contains("linux") {
        "linux"
    } else if triple.contains("darwin") {
        "macos"
    } else if triple.contains("windows") {
        "windows"
    } else {
        bail!("unsupported target triple `{triple}` — extend cargo_triple_to_cmake_target");
    };
    Ok(format!("{os}-{arch}"))
}

/// Patch the staged `<crate>Config.cmake` to encode the *archive's*
/// target rather than whatever the host build.rs baked in. See the
/// `--target` flag doc on `Stage`.
fn rewrite_built_target(prefix: &Path, crate_name: &str, triple: &str) -> anyhow::Result<()> {
    let expected = cargo_triple_to_cmake_target(triple)?;
    let config_path = prefix
        .join("lib/cmake")
        .join(crate_name)
        .join(format!("{crate_name}Config.cmake"));
    let contents = fs::read_to_string(&config_path)
        .with_context(|| format!("Failed to read {}", config_path.display()))?;
    let mut rewrote = false;
    let new_contents: String = contents
        .lines()
        .map(|line| {
            let trimmed = line.trim_start();
            if trimmed.starts_with("set(DORA_BUILT_TARGET") {
                rewrote = true;
                format!("set(DORA_BUILT_TARGET \"{expected}\")")
            } else {
                line.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");
    if !rewrote {
        bail!(
            "DORA_BUILT_TARGET line not found in {}; cmake template may have drifted",
            config_path.display()
        );
    }
    let final_contents = if contents.ends_with('\n') {
        format!("{new_contents}\n")
    } else {
        new_contents
    };
    fs::write(&config_path, final_contents)
        .with_context(|| format!("Failed to write {}", config_path.display()))?;
    println!("  REWROTE DORA_BUILT_TARGET → {expected} (from triple {triple})");
    Ok(())
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
    // The cxxbridge artefacts live next to the cargo `target/` directory,
    // but cargo lays them out as either `target/cxxbridge/<crate>/` (host
    // builds) or `target/<triple>/cxxbridge/<crate>/` (cross builds /
    // `--target` invocations). Locate whichever exists by walking up the
    // ancestors of `target_dir` (which is `target/release`,
    // `target/<triple>/release`, or similar) and checking each one's
    // `cxxbridge/<crate>` subdirectory.
    // Prefer a candidate whose `lib/cmake/<crate>` subdir is populated —
    // otherwise we may pick up a stale `cxxbridge/<crate>` from a previous
    // build that predates the cmake-emitting build.rs.
    let cxxbridge_dir = target_dir
        .ancestors()
        .find_map(|ancestor| {
            let candidate = ancestor.join("cxxbridge").join(crate_name);
            candidate
                .join("lib/cmake")
                .join(crate_name)
                .exists()
                .then_some(candidate)
        })
        .ok_or_else(|| {
            anyhow::anyhow!(
                "Cxxbridge directory with cmake config not found for {crate_name} in any ancestor of {}.\n\
                 Ensure {crate_name} has been built (cargo build -p {crate_name}).",
                target_dir.display()
            )
        })?;

    let cmake_src = cxxbridge_dir.join("lib/cmake").join(crate_name);
    let include_src = cxxbridge_dir.join("include");
    let src_src = cxxbridge_dir.join("src");

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
    let src_dir = prefix.join("src");
    fs::create_dir_all(&lib_cmake_dir)?;
    fs::create_dir_all(&include_dir)?;
    fs::create_dir_all(&src_dir)?;

    copy_dir_contents(&cmake_src, &lib_cmake_dir.join(crate_name))
        .with_context(|| format!("Failed to copy cmake config for {crate_name}"))?;

    copy_dir_contents(&include_src, &include_dir)
        .with_context(|| format!("Failed to copy headers for {crate_name}"))?;

    if src_src.exists() {
        copy_dir_contents(&src_src, &src_dir)
            .with_context(|| format!("Failed to copy cxxbridge source for {crate_name}"))?;
    }

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

    // Look for the static lib at the user-supplied target_dir first
    // (the common publish-workflow case where `cargo build --target X
    // --release` puts it at `target/X/release/`). On macOS host builds,
    // cargo writes the lib to `target/<profile>/` (no triple subdir)
    // even though the cxxbridge files end up under `target/<triple>/`,
    // so also try the sibling `target/<profile>/` path. The profile is
    // the last component of target_dir.
    let lib_dst = prefix.join("lib").join(lib_name);
    let primary = target_dir.join(lib_name);
    let host_fallback = target_dir
        .file_name()
        .and_then(|profile| {
            target_dir
                .ancestors()
                .nth(2)
                .map(|root| root.join(profile).join(lib_name))
        })
        .filter(|p| p != &primary);

    let lib_src = if primary.exists() {
        Some(primary)
    } else {
        host_fallback.filter(|p| p.exists())
    };

    if let Some(lib_src) = lib_src {
        fs::copy(&lib_src, &lib_dst).with_context(|| {
            format!(
                "Failed to copy library {} -> {}",
                lib_src.display(),
                lib_dst.display()
            )
        })?;
    } else {
        // Known crate but no static library — fail loudly. Returning Ok
        // here would let the publish workflow archive and upload a
        // find_package prefix whose imported cmake target points at a
        // non-existent .a/.lib, which only fails at the consumer's
        // link step.
        bail!(
            "Library {} not found at {} or sibling host-profile dir. \
             Run `cargo build -p {crate_name}` (or `--release`, matching the staged target_dir) first.",
            lib_name,
            target_dir.join(lib_name).display()
        );
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn fixture_staging_without_lib(tmp: &Path, crate_name: &str) -> PathBuf {
        let target_dir = tmp.join("target");
        let staging = target_dir.join(crate_name);
        fs::create_dir_all(staging.join("lib/cmake").join(crate_name)).unwrap();
        fs::create_dir_all(staging.join("include")).unwrap();
        fs::write(
            staging
                .join("lib/cmake")
                .join(crate_name)
                .join(format!("{crate_name}Config.cmake")),
            "# placeholder\n",
        )
        .unwrap();
        fs::write(
            staging.join("include").join("header.h"),
            "/* placeholder */\n",
        )
        .unwrap();
        target_dir
    }

    /// The publish workflow archives whatever `xtask stage` leaves in
    /// the prefix dir, so a missing static lib MUST be a hard error.
    /// Otherwise we ship a find_package archive whose imported cmake
    /// target points at a non-existent `.a`/`.lib` and the failure
    /// only surfaces at the consumer's link step.
    #[test]
    fn stage_c_crate_errors_when_static_lib_missing() {
        let tmp = tempdir();
        let target_dir = fixture_staging_without_lib(tmp.path(), "dora-node-api-c");
        let prefix = tmp.path().join("prefix");

        let err = stage_library("dora-node-api-c", &target_dir, &prefix, None)
            .expect_err("missing static lib must fail staging, not warn");
        let msg = err.to_string();
        assert!(
            msg.contains("libdora_node_api_c.a") || msg.contains("dora_node_api_c.lib"),
            "error must name the missing library: {msg}"
        );
    }

    /// Sanity: with the lib present at the expected location, staging
    /// succeeds. This guards against regressions that would turn the
    /// missing-lib check into a false positive.
    #[test]
    fn stage_c_crate_succeeds_when_lib_present() {
        let tmp = tempdir();
        let target_dir = fixture_staging_without_lib(tmp.path(), "dora-node-api-c");
        // Provide the lib at the path copy_library looks for first.
        let lib_name = if cfg!(windows) {
            "dora_node_api_c.lib"
        } else {
            "libdora_node_api_c.a"
        };
        fs::write(target_dir.join(lib_name), b"fake static lib\n").unwrap();
        let prefix = tmp.path().join("prefix");

        stage_library("dora-node-api-c", &target_dir, &prefix, None)
            .expect("staging should succeed when lib is present");
        assert!(prefix.join("lib").join(lib_name).exists());
    }

    /// Regression test for the cross-cxx target-encoding bug surfaced
    /// during /pr-review of #1875. When the cxxbridge `build.rs` runs
    /// on the host during a cross build (e.g., the publish workflow's
    /// `aarch64-unknown-linux-gnu` job), `DORA_BUILT_TARGET` ends up
    /// encoded as `linux-x86_64` — and the consumer on aarch64 then
    /// trips the cmake template's `target mismatch! built for
    /// linux-x86_64, but current is linux-aarch64` FATAL_ERROR even
    /// though the lib is correct for them. Passing `--target` to
    /// `xtask stage` must rewrite the staged config to match the
    /// archive's actual triple.
    #[test]
    fn stage_rewrites_built_target_when_triple_supplied() {
        let tmp = tempdir();
        let target_dir = fixture_staging_without_lib(tmp.path(), "dora-node-api-c");
        let lib_name = if cfg!(windows) {
            "dora_node_api_c.lib"
        } else {
            "libdora_node_api_c.a"
        };
        fs::write(target_dir.join(lib_name), b"fake\n").unwrap();
        // Bake a wrong DORA_BUILT_TARGET into the staged config — the
        // shape build.rs would have written on a host-x86_64 cross run.
        let config_path = target_dir
            .join("dora-node-api-c/lib/cmake/dora-node-api-c/dora-node-api-cConfig.cmake");
        fs::write(
            &config_path,
            "set(DORA_BUILT_TARGET \"linux-x86_64\")\n# rest of template\n",
        )
        .unwrap();
        let prefix = tmp.path().join("prefix");

        stage_library(
            "dora-node-api-c",
            &target_dir,
            &prefix,
            Some("aarch64-unknown-linux-gnu"),
        )
        .expect("cross-target staging should succeed");

        let staged = fs::read_to_string(
            prefix.join("lib/cmake/dora-node-api-c/dora-node-api-cConfig.cmake"),
        )
        .unwrap();
        assert!(
            staged.contains("set(DORA_BUILT_TARGET \"linux-aarch64\")"),
            "DORA_BUILT_TARGET should be rewritten to match the archive triple: {staged}"
        );
        assert!(
            !staged.contains("linux-x86_64"),
            "stale host-target should be gone: {staged}"
        );
    }

    /// If the cmake template ever drops or renames `DORA_BUILT_TARGET`,
    /// the rewrite must bail loudly rather than silently produce a
    /// config that skips the target-match check. Guards against the
    /// fix-degrading-into-noop failure mode.
    #[test]
    fn stage_bails_when_built_target_line_missing() {
        let tmp = tempdir();
        let target_dir = fixture_staging_without_lib(tmp.path(), "dora-node-api-c");
        let lib_name = if cfg!(windows) {
            "dora_node_api_c.lib"
        } else {
            "libdora_node_api_c.a"
        };
        fs::write(target_dir.join(lib_name), b"fake\n").unwrap();
        let config_path = target_dir
            .join("dora-node-api-c/lib/cmake/dora-node-api-c/dora-node-api-cConfig.cmake");
        // Template-shaped config but with the DORA_BUILT_TARGET line
        // removed — simulates someone editing the .cmake.in template
        // without updating the xtask rewrite.
        fs::write(&config_path, "# placeholder config, no built target\n").unwrap();
        let prefix = tmp.path().join("prefix");

        let err = stage_library(
            "dora-node-api-c",
            &target_dir,
            &prefix,
            Some("aarch64-unknown-linux-gnu"),
        )
        .expect_err("missing DORA_BUILT_TARGET line must fail loudly, not produce silent-no-op");
        assert!(
            err.to_string().contains("DORA_BUILT_TARGET")
                || format!("{err:#}").contains("DORA_BUILT_TARGET"),
            "error must name the missing field: {err:#}"
        );
    }

    /// Pins the triple→cmake-target translation for the four targets
    /// the publish workflow currently builds. Catches a regression
    /// where someone "simplifies" cargo_triple_to_cmake_target and
    /// breaks one of the live publish triples.
    #[test]
    fn triple_translation_covers_published_targets() {
        let cases = [
            ("x86_64-unknown-linux-gnu", "linux-x86_64"),
            ("aarch64-unknown-linux-gnu", "linux-aarch64"),
            ("aarch64-apple-darwin", "macos-aarch64"),
            ("x86_64-pc-windows-msvc", "windows-x86_64"),
        ];
        for (triple, expected) in cases {
            let got = cargo_triple_to_cmake_target(triple).expect("translation should succeed");
            assert_eq!(
                got, expected,
                "triple {triple} → {got}, expected {expected}"
            );
        }
        assert!(
            cargo_triple_to_cmake_target("wasm32-unknown-unknown").is_err(),
            "unsupported triple must bail, not silently default"
        );
    }

    /// Android triples contain "linux" as a substring
    /// (`aarch64-linux-android`, `armv7-linux-androideabi`). A naive
    /// `triple.contains("linux")` check classifies them as Linux and
    /// the cmake target-match would pass on a glibc consumer running
    /// on the same arch — only to fail at link time when Bionic
    /// symbols are missing. The Android check must short-circuit
    /// before Linux. Regression guard.
    #[test]
    fn android_triple_rejected_before_linux_classification() {
        for triple in ["aarch64-linux-android", "armv7-linux-androideabi"] {
            let err = cargo_triple_to_cmake_target(triple).expect_err(
                "Android triple must be rejected, not classified as linux despite the substring",
            );
            let msg = err.to_string();
            assert!(
                msg.to_lowercase().contains("android"),
                "error must name the Android mismatch, not a generic linux-target message: {msg}"
            );
        }
    }

    /// Tiny owned-tempdir helper. Avoids pulling in `tempfile` as a
    /// dependency for a one-off test crate.
    fn tempdir() -> TempDir {
        let base = std::env::temp_dir().join(format!(
            "dora-xtask-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&base).unwrap();
        TempDir { path: base }
    }

    struct TempDir {
        path: PathBuf,
    }

    impl TempDir {
        fn path(&self) -> &Path {
            &self.path
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.path);
        }
    }
}
