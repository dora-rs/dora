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

        /// Staging output directory (e.g., dora-libraries-c-x86_64-unknown-linux-gnu)
        prefix: PathBuf,

        /// Cargo target triple this prefix is being staged for
        /// (e.g., `aarch64-unknown-linux-gnu`); defaults to the host.
        /// It becomes `DORA_BUILT_TARGET` in the cmake config, which a
        /// consumer's cmake checks against its own platform. For cxx
        /// crates, the cxxbridge `build.rs` may run on the host during a
        /// cross build and encode `linux-x86_64`, so xtask rewrites the
        /// value at stage time.
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
        if let Some(triple) = target_triple {
            rewrite_built_target(prefix, crate_name, triple).with_context(|| {
                format!("Failed to rewrite DORA_BUILT_TARGET for {crate_name} ({triple})")
            })?;
        }
    } else {
        stage_c_crate(crate_name, target_dir, prefix, target_triple)?;
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

/// The C API crates: their directory relative to the workspace root and the
/// headers they ship.
const C_CRATES: &[(&str, &str, &[&str])] = &[
    ("dora-node-api-c", "apis/c/node", &["node_api.h"]),
    (
        "dora-operator-api-c",
        "apis/c/operator",
        &["operator_api.h", "operator_types.h"],
    ),
];

/// Stages a C crate: its cmake package config, generated from the crate's
/// `cmake/*.cmake.in` templates, its headers, and its static library.
///
/// The config only needs the crate's name, version and target, so it's
/// generated here rather than in a build script: cargo's build directory
/// layout isn't a stable interface, and a build script that writes outside
/// its `OUT_DIR` breaks when it changes.
fn stage_c_crate(
    crate_name: &str,
    target_dir: &Path,
    prefix: &Path,
    target_triple: Option<&str>,
) -> anyhow::Result<()> {
    let Some(&(_, dir, headers)) = C_CRATES.iter().find(|(name, ..)| *name == crate_name) else {
        bail!("unknown C crate {crate_name}");
    };
    let workspace_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .context("xtask has no parent directory")?;
    let crate_dir = workspace_root.join(dir);
    let target = match target_triple {
        Some(triple) => cargo_triple_to_cmake_target(triple)?,
        None => format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH),
    };
    let version = crate_version(crate_name)?;
    let lib_base = crate_name.replace('-', "_");
    let template = |name: &str| {
        let path = crate_dir.join("cmake").join(name);
        fs::read_to_string(&path).with_context(|| format!("Failed to read {}", path.display()))
    };
    let config = template("dora-api-config.cmake.in")?
        .replace("@PACKAGE@", crate_name)
        .replace("@TARGET@", &target)
        .replace("@LIB_UNIX@", &format!("lib{lib_base}.a"))
        .replace("@LIB_WIN@", &format!("{lib_base}.lib"))
        .replace("@CXX_BRIDGE_FILES@", "");
    let config_version = template("dora-api-version.cmake.in")?.replace("@VERSION@", &version);

    println!("Staging {crate_name} (C crate)...");
    println!("  SOURCE:     {}", crate_dir.display());
    println!("  TARGET_DIR: {}", target_dir.display());
    println!("  PREFIX:     {}", prefix.display());

    let cmake_dir = prefix.join("lib/cmake").join(crate_name);
    let include_dir = prefix.join("include");
    fs::create_dir_all(&cmake_dir)?;
    fs::create_dir_all(&include_dir)?;

    fs::write(cmake_dir.join(format!("{crate_name}Config.cmake")), config)?;
    fs::write(
        cmake_dir.join(format!("{crate_name}ConfigVersion.cmake")),
        config_version,
    )?;
    for header in headers {
        fs::copy(crate_dir.join(header), include_dir.join(header))
            .with_context(|| format!("Failed to copy {header} for {crate_name}"))?;
    }

    copy_library(crate_name, target_dir, prefix)?;

    print_staged_files(prefix, crate_name);

    Ok(())
}

/// The version of a workspace crate, from `cargo pkgid`, which prints
/// `<source>#<name>@<version>` or `<source>#<version>`.
fn crate_version(crate_name: &str) -> anyhow::Result<String> {
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let output = std::process::Command::new(cargo)
        .args(["pkgid", "-p", crate_name])
        .output()
        .context("Failed to run cargo pkgid")?;
    if !output.status.success() {
        bail!(
            "cargo pkgid -p {crate_name} failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }
    let pkgid = String::from_utf8(output.stdout)?;
    let spec = pkgid
        .trim()
        .rsplit_once('#')
        .map_or(pkgid.trim(), |(_, s)| s);
    let version = spec.rsplit_once('@').map_or(spec, |(_, v)| v);
    Ok(version.to_owned())
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

    // Only the bridge source the cmake config references
    // (`@CXX_BRIDGE_FILES@`). The directory also holds cxx-build's own
    // `lib.rs.*` symlinks into OUT_DIR, which don't resolve on Windows with
    // cargo 1.100.
    let bridge_source = format!("{}.cc", crate_name.trim_end_matches("-cxx"));
    fs::copy(src_src.join(&bridge_source), src_dir.join(&bridge_source))
        .with_context(|| format!("Failed to copy {bridge_source} for {crate_name}"))?;

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

    fn c_lib_name(crate_name: &str) -> String {
        let base = crate_name.replace('-', "_");
        if cfg!(windows) {
            format!("{base}.lib")
        } else {
            format!("lib{base}.a")
        }
    }

    /// The publish workflow archives whatever `xtask stage` leaves in
    /// the prefix dir, so a missing static lib MUST be a hard error.
    /// Otherwise we ship a find_package archive whose imported cmake
    /// target points at a non-existent `.a`/`.lib` and the failure
    /// only surfaces at the consumer's link step.
    #[test]
    fn stage_c_crate_errors_when_static_lib_missing() {
        let tmp = tempdir();
        let target_dir = tmp.path().join("target");
        fs::create_dir_all(&target_dir).unwrap();
        let prefix = tmp.path().join("prefix");

        let err = stage_library("dora-node-api-c", &target_dir, &prefix, None)
            .expect_err("missing static lib must fail staging, not warn");
        let msg = err.to_string();
        assert!(
            msg.contains(&c_lib_name("dora-node-api-c")),
            "error must name the missing library: {msg}"
        );
    }

    /// Staging a C crate needs only its static library from the build:
    /// the cmake config is generated from the crate's templates with every
    /// placeholder filled, and the headers come from the crate directory.
    #[test]
    fn stage_c_crate_generates_config_and_copies_headers() {
        for (crate_name, _, headers) in C_CRATES {
            let tmp = tempdir();
            let target_dir = tmp.path().join("target");
            fs::create_dir_all(&target_dir).unwrap();
            fs::write(
                target_dir.join(c_lib_name(crate_name)),
                b"fake static lib\n",
            )
            .unwrap();
            let prefix = tmp.path().join("prefix");

            stage_library(crate_name, &target_dir, &prefix, None)
                .expect("staging should succeed when lib is present");

            assert!(prefix.join("lib").join(c_lib_name(crate_name)).exists());
            for header in *headers {
                assert!(prefix.join("include").join(header).exists(), "{header}");
            }
            let cmake_dir = prefix.join("lib/cmake").join(crate_name);
            let config =
                fs::read_to_string(cmake_dir.join(format!("{crate_name}Config.cmake"))).unwrap();
            let config_version =
                fs::read_to_string(cmake_dir.join(format!("{crate_name}ConfigVersion.cmake")))
                    .unwrap();
            for contents in [&config, &config_version] {
                assert!(!contents.contains('@'), "unfilled placeholder: {contents}");
            }
            let host = format!("{}-{}", std::env::consts::OS, std::env::consts::ARCH);
            assert!(
                config.contains(&format!("set(DORA_BUILT_TARGET \"{host}\")")),
                "{config}"
            );
            assert!(config.contains(&format!("{crate_name}::{crate_name}")));
            let version = crate_version(crate_name).unwrap();
            assert!(
                config_version.contains(&format!("\"{version}\"")),
                "{config_version}"
            );
        }
    }

    /// The cmake config of a cross-compiled archive must name the
    /// archive's triple, not the host's: a consumer on aarch64 otherwise
    /// trips the template's `target mismatch!` FATAL_ERROR even though
    /// the lib is correct for them (#1875).
    #[test]
    fn stage_c_crate_uses_the_supplied_triple() {
        let tmp = tempdir();
        let target_dir = tmp.path().join("target");
        fs::create_dir_all(&target_dir).unwrap();
        fs::write(target_dir.join(c_lib_name("dora-node-api-c")), b"fake\n").unwrap();
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
            "DORA_BUILT_TARGET should match the archive triple: {staged}"
        );
    }

    /// The cxxbridge `build.rs` runs on the host during a cross build, so
    /// its config says `linux-x86_64` for an aarch64 archive; staging with
    /// `--target` rewrites it (#1875).
    #[test]
    fn rewrite_built_target_replaces_the_host_target() {
        let tmp = tempdir();
        let prefix = tmp.path();
        let cmake_dir = prefix.join("lib/cmake/dora-node-api-cxx");
        fs::create_dir_all(&cmake_dir).unwrap();
        let config_path = cmake_dir.join("dora-node-api-cxxConfig.cmake");
        fs::write(
            &config_path,
            "set(DORA_BUILT_TARGET \"linux-x86_64\")\n# rest of template\n",
        )
        .unwrap();

        rewrite_built_target(prefix, "dora-node-api-cxx", "aarch64-unknown-linux-gnu").unwrap();

        let staged = fs::read_to_string(&config_path).unwrap();
        assert!(
            staged.contains("set(DORA_BUILT_TARGET \"linux-aarch64\")"),
            "{staged}"
        );
        assert!(!staged.contains("linux-x86_64"), "{staged}");
    }

    /// If the cmake template ever drops or renames `DORA_BUILT_TARGET`,
    /// the rewrite must bail loudly rather than silently produce a
    /// config that skips the target-match check. Guards against the
    /// fix-degrading-into-noop failure mode.
    #[test]
    fn rewrite_built_target_bails_when_line_missing() {
        let tmp = tempdir();
        let prefix = tmp.path();
        let cmake_dir = prefix.join("lib/cmake/dora-node-api-cxx");
        fs::create_dir_all(&cmake_dir).unwrap();
        fs::write(
            cmake_dir.join("dora-node-api-cxxConfig.cmake"),
            "# placeholder config, no built target\n",
        )
        .unwrap();

        let err = rewrite_built_target(prefix, "dora-node-api-cxx", "aarch64-unknown-linux-gnu")
            .expect_err(
                "missing DORA_BUILT_TARGET line must fail loudly, not produce silent-no-op",
            );
        assert!(
            format!("{err:#}").contains("DORA_BUILT_TARGET"),
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
    ///
    /// The path mixes process id, thread id, and a monotonic counter.
    /// `SystemTime::now().as_nanos()` was the original disambiguator
    /// but `clock_gettime(CLOCK_REALTIME)` on macOS only ticks at
    /// ~µs resolution — `as_nanos()` looks unique but ~83% of paired
    /// calls collide. Two parallel cargo-test threads landing on the
    /// same path silently share a `target/` directory; the sibling
    /// `stage_c_crate_succeeds_when_lib_present` test writes a
    /// `libdora_node_api_c.a` that `stage_c_crate_errors_when_static_lib_missing`
    /// then mistakenly sees, flipping its `expect_err` into a panic
    /// (issue #1940). Process id + thread id + atomic counter is
    /// race-free without depending on clock resolution.
    fn tempdir() -> TempDir {
        static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let base = std::env::temp_dir().join(format!(
            "dora-xtask-test-{}-{:?}-{}",
            std::process::id(),
            std::thread::current().id(),
            seq,
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
