//! Hermetic end-to-end test of the `hub:` flow (P2.11).
//!
//! Builds a self-contained fixture in a tempdir — a git source repo holding a
//! trivial node, a local `node-index` catalog pinning it by commit, and a
//! `hub.toml` binding the `test` namespace to that index — then drives
//! `dora hub search/info`, `dora build`, `dora build --locked`, and
//! `dora hub fetch` against it. No network, no live catalog, and no Python
//! bindings (the node is a plain Rust `main` that exits 0).

use std::{
    path::{Path, PathBuf},
    process::Command,
    sync::Once,
};

static BUILD_CLI: Once = Once::new();

fn dora_bin() -> PathBuf {
    BUILD_CLI.call_once(|| {
        let status = Command::new("cargo")
            .args(["build", "-p", "dora-cli"])
            .status()
            .expect("cargo build dora-cli");
        assert!(status.success(), "failed to build dora CLI");
    });
    let target = std::env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| Path::new(env!("CARGO_MANIFEST_DIR")).join("target"));
    let bin = target
        .join("debug")
        .join(format!("dora{}", std::env::consts::EXE_SUFFIX));
    assert!(bin.exists(), "dora binary missing at {}", bin.display());
    bin
}

fn git(dir: &Path, args: &[&str]) {
    let status = Command::new("git")
        .current_dir(dir)
        .args([
            "-c",
            "user.email=t@dora.rs",
            "-c",
            "user.name=t",
            "-c",
            "commit.gpgsign=false",
        ])
        .args(args)
        .status()
        .expect("run git");
    assert!(status.success(), "git {args:?} failed");
}

fn write(path: &Path, content: &str) {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).unwrap();
    }
    std::fs::write(path, content).unwrap();
}

struct Fixture {
    _tmp: tempfile::TempDir,
    root: PathBuf,
    hub_config: PathBuf,
}

/// Build the source repo + index + hub.toml; returns the fixture paths.
fn build_fixture() -> Fixture {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path().to_path_buf();

    // 1. a git source repo with a trivial node + its manifest
    let src = root.join("source");
    write(
        &src.join("node-hub/hello/Cargo.toml"),
        "[package]\nname = \"hub-smoke-hello\"\nversion = \"0.1.0\"\nedition = \"2021\"\n\
         \n[[bin]]\nname = \"hub-smoke-hello\"\npath = \"src/main.rs\"\n\n[workspace]\n",
    );
    write(
        &src.join("node-hub/hello/src/main.rs"),
        "fn main() { println!(\"hello from the hub smoke node\"); }\n",
    );
    write(
        &src.join("node-hub/hello/dora-node.yml"),
        "apiVersion: 1\nname: hub-smoke-hello\nnamespace: test\n\
         description: \"hermetic smoke node\"\ncategories: [debug]\n\
         keywords: [smoke]\nruntime: rust\n\
         entrypoint: target/release/hub-smoke-hello\nbuild: cargo build --release\n",
    );
    git(&src, &["init", "--quiet", "-b", "main"]);
    git(&src, &["add", "."]);
    git(&src, &["commit", "--quiet", "-m", "init"]);
    let commit = String::from_utf8(
        Command::new("git")
            .current_dir(&src)
            .args(["rev-parse", "HEAD"])
            .output()
            .unwrap()
            .stdout,
    )
    .unwrap()
    .trim()
    .to_string();
    // allow blobless clone over the local transport
    git(&src, &["config", "uploadpack.allowFilter", "true"]);

    // 2. the index catalog
    let src_url = format!("file://{}", src.display());
    write(
        &root.join("index/test/hub-smoke-hello/0.1.0.yml"),
        &format!(
            "manifest:\n  apiVersion: 1\n  name: hub-smoke-hello\n  namespace: test\n  \
             description: \"hermetic smoke node\"\n  categories: [debug]\n  keywords: [smoke]\n  \
             runtime: rust\n  entrypoint: target/release/hub-smoke-hello\n  \
             build: cargo build --release\nsource:\n  git: {src_url}\n  rev: {commit}\n  \
             subdir: node-hub/hello\n"
        ),
    );

    // 3. hub.toml binding the `test` namespace to the local index
    let hub_config = root.join("hub.toml");
    write(
        &hub_config,
        &format!(
            "[[index]]\nalias = \"smoke\"\npath = \"{}\"\nnamespaces = [\"test\"]\n",
            root.join("index").display()
        ),
    );

    Fixture {
        _tmp: tmp,
        root,
        hub_config,
    }
}

fn dora(fixture: &Fixture) -> Command {
    let mut cmd = Command::new(dora_bin());
    cmd.env("DORA_HUB_CONFIG", &fixture.hub_config);
    // the fixture index entries point at a `file://` source repo; opt in to
    // local sources, which a public index would otherwise reject
    cmd.env("DORA_HUB_ALLOW_LOCAL_SOURCES", "1");
    cmd.current_dir(&fixture.root);
    cmd
}

#[test]
fn hub_end_to_end() {
    // Skip cleanly where git isn't available rather than hard-failing.
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub smoke test");
        return;
    }
    let fixture = build_fixture();

    // search finds the node by keyword
    let out = dora(&fixture)
        .args(["hub", "search", "smoke"])
        .output()
        .unwrap();
    assert!(out.status.success(), "search failed: {}", stderr(&out));
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.contains("test/hub-smoke-hello"),
        "search output: {stdout}"
    );

    // info prints the contracts
    let out = dora(&fixture)
        .args(["hub", "info", "test/hub-smoke-hello"])
        .output()
        .unwrap();
    assert!(out.status.success(), "info failed: {}", stderr(&out));
    assert!(
        String::from_utf8_lossy(&out.stdout).contains("hermetic smoke node"),
        "info output missing description"
    );

    // a dataflow referencing the hub package
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");

    // build resolves, clones at the pin, builds in the subdir, writes lockfile
    let out = dora(&fixture)
        .args(["build", flow.to_str().unwrap(), "--write-lockfile"])
        .output()
        .unwrap();
    assert!(out.status.success(), "build failed: {}", stderr(&out));
    let lockfile = fixture.root.join("flow/dataflow.dora-lock.yaml");
    let lock = std::fs::read_to_string(&lockfile).unwrap();
    assert!(lock.contains("subdir: node-hub/hello"), "lockfile: {lock}");
    assert!(
        lock.contains("name: test/hub-smoke-hello"),
        "lockfile: {lock}"
    );

    // hub list shows the pinned package
    let out = dora(&fixture)
        .args(["hub", "list", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(out.status.success(), "list failed: {}", stderr(&out));
    assert!(
        String::from_utf8_lossy(&out.stdout).contains("test/hub-smoke-hello"),
        "list output"
    );

    // run executes the node (it prints and exits 0)
    let out = dora(&fixture)
        .args(["run", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(out.status.success(), "run failed: {}", stderr(&out));
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        combined.contains("hello from the hub smoke node"),
        "node output missing:\n{combined}"
    );

    // --locked + --offline rebuild from the pin without touching the network
    let out = dora(&fixture)
        .args(["build", flow.to_str().unwrap(), "--locked", "--offline"])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "locked build failed: {}",
        stderr(&out)
    );

    // --locked must reject a rewritten index entry: the commit pins the source
    // tree, but the manifest (entrypoint, build, contract) lives in the
    // (mutable) index entry, so a tampered `build:` must hard-error via the
    // pinned manifest digest rather than run silently.
    let entry_path = fixture.root.join("index/test/hub-smoke-hello/0.1.0.yml");
    let original_entry = std::fs::read_to_string(&entry_path).unwrap();
    let tampered = original_entry.replace("build: cargo build --release", "build: echo pwned");
    assert_ne!(
        original_entry, tampered,
        "tamper replace should change the entry"
    );
    std::fs::write(&entry_path, &tampered).unwrap();
    let out = dora(&fixture)
        .args(["build", flow.to_str().unwrap(), "--locked", "--offline"])
        .output()
        .unwrap();
    assert!(
        !out.status.success(),
        "locked build must reject a tampered manifest"
    );
    assert!(
        stderr(&out).contains("changed its manifest"),
        "expected a manifest-tamper error, got: {}",
        stderr(&out)
    );
    std::fs::write(&entry_path, &original_entry).unwrap(); // restore for the steps below

    // hub fetch pre-clones the pinned source
    let out = dora(&fixture)
        .args([
            "hub",
            "fetch",
            flow.to_str().unwrap(),
            "--target-dir",
            "cache",
        ])
        .output()
        .unwrap();
    assert!(out.status.success(), "fetch failed: {}", stderr(&out));
    assert!(
        fixture
            .root
            .join("cache")
            .read_dir()
            .unwrap()
            .next()
            .is_some(),
        "fetch populated nothing"
    );

    // a stale clone (dir deleted, completion marker left behind) must re-clone
    // rather than report success with no source
    let cache = fixture.root.join("cache");
    let clone_dir = std::fs::read_dir(&cache)
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .find(|p| p.is_dir())
        .expect("a clone dir");
    std::fs::remove_dir_all(&clone_dir).unwrap(); // marker `.<commit>.complete` stays
    let out = dora(&fixture)
        .args([
            "hub",
            "fetch",
            flow.to_str().unwrap(),
            "--target-dir",
            "cache",
        ])
        .output()
        .unwrap();
    assert!(out.status.success(), "re-fetch failed: {}", stderr(&out));
    assert!(
        clone_dir.is_dir(),
        "stale marker was trusted; clone not restored"
    );

    // fetch must validate the dataflow against the lockfile: a flow that no
    // longer references the locked hub node is a stale mirror and must error
    std::fs::write(&flow, "nodes:\n  - id: plain\n    path: dynamic\n").unwrap();
    let out = dora(&fixture)
        .args([
            "hub",
            "fetch",
            flow.to_str().unwrap(),
            "--target-dir",
            "cache",
        ])
        .output()
        .unwrap();
    assert!(!out.status.success(), "fetch of a stale lockfile must fail");
    assert!(
        stderr(&out).contains("no longer has") || stderr(&out).contains("regenerate"),
        "expected a stale-lockfile error, got: {}",
        stderr(&out)
    );
}

/// The hub resolver loads a package's shipped `types:` into the dataflow's
/// type registry. The index entry is untrusted, so a rewritten entry that
/// ships a `std/` override (or a cross-namespace type) must be rejected at
/// resolve time — before the type can back a consumer's port check (P2.12).
#[test]
fn hub_rejects_invalid_shipped_type() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub shipped-type test");
        return;
    }
    let fixture = build_fixture();

    // Rewrite the entry's manifest to ship a `std/` type — the namespace rule
    // forbids it, and the resolver must enforce that on the untrusted entry.
    let entry_path = fixture.root.join("index/test/hub-smoke-hello/0.1.0.yml");
    let entry = std::fs::read_to_string(&entry_path).unwrap();
    let tampered = entry.replace(
        "  build: cargo build --release\nsource:",
        "  build: cargo build --release\n  types:\n    std/core/v1/Hijack:\n      arrow: Struct\nsource:",
    );
    assert_ne!(entry, tampered, "expected the types injection to apply");
    std::fs::write(&entry_path, &tampered).unwrap();

    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    let out = dora(&fixture)
        .args(["build", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(
        !out.status.success(),
        "build must reject a package shipping a `std/` type"
    );
    assert!(
        stderr(&out).contains("invalid custom types") || stderr(&out).contains("std/"),
        "expected a shipped-type rejection, got: {}",
        stderr(&out)
    );
}

/// `dora build --hub-override <pkg>=<path>` substitutes a local checkout for a
/// hub package (UC11): the manifest is read from the checkout, contracts are
/// validated, and the node builds from local source — no index resolution, no
/// clone. We assert the build roots at the checkout (its `target/` appears) and
/// that the override is reported.
#[test]
fn hub_override_builds_from_local_checkout() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub override test");
        return;
    }
    let fixture = build_fixture();

    // the local checkout is the node's source dir inside the fixture repo
    let checkout = fixture.root.join("source/node-hub/hello");
    assert!(checkout.join("dora-node.yml").is_file(), "fixture checkout");

    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");

    let out = dora(&fixture)
        .args([
            "build",
            flow.to_str().unwrap(),
            "--hub-override",
            &format!("test/hub-smoke-hello={}", checkout.display()),
        ])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "override build failed: {}",
        stderr(&out)
    );
    let combined = format!("{}{}", String::from_utf8_lossy(&out.stdout), stderr(&out));
    assert!(
        combined.contains("overriding hub package test/hub-smoke-hello"),
        "expected an override note, got:\n{combined}"
    );
    // the node built from local source: the binary lands in the checkout's
    // own target dir, not a clone under the hub cache
    let built = checkout.join("target/release/hub-smoke-hello");
    assert!(
        built.exists(),
        "override did not build in the local checkout (missing {})",
        built.display()
    );
}

/// `dora run --hub-override` must build AND run the LOCAL checkout, not the
/// published/committed source. We modify the checkout's source (uncommitted)
/// to print a unique marker; if the override truly uses local source, that
/// marker — not the index-pinned version — appears at run time (UC11).
#[test]
fn hub_run_override_executes_local_source() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub run-override test");
        return;
    }
    let fixture = build_fixture();
    let checkout = fixture.root.join("source/node-hub/hello");

    // edit the checkout's source WITHOUT committing — the index pin still
    // points at the original commit, so only a local override sees this.
    let marker = "LOCAL-OVERRIDE-MARKER-9173";
    write(
        &checkout.join("src/main.rs"),
        &format!("fn main() {{ println!(\"{marker}\"); }}\n"),
    );

    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    let out = dora(&fixture)
        .args([
            "run",
            flow.to_str().unwrap(),
            "--hub-override",
            &format!("test/hub-smoke-hello={}", checkout.display()),
        ])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "run --hub-override failed: {}",
        stderr(&out)
    );
    let combined = format!("{}{}", String::from_utf8_lossy(&out.stdout), stderr(&out));
    assert!(
        combined.contains(marker),
        "run did not execute the local checkout (marker absent):\n{combined}"
    );
}

/// An override that names no node in the dataflow is reported, not silently
/// ignored.
#[test]
fn hub_override_unused_is_warned() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub override-warn test");
        return;
    }
    let fixture = build_fixture();
    let checkout = fixture.root.join("source/node-hub/hello");
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    let out = dora(&fixture)
        .args([
            "build",
            flow.to_str().unwrap(),
            "--hub-override",
            &format!("test/not-a-node={}", checkout.display()),
        ])
        .output()
        .unwrap();
    // resolution still proceeds against the index for the real node; the unused
    // override is surfaced as a warning
    assert!(
        stderr(&out).contains("did not match any hub node")
            || String::from_utf8_lossy(&out.stdout).contains("did not match any hub node"),
        "expected an unused-override warning, got:\n{}",
        stderr(&out)
    );
}

/// An override is also surfaced when the dataflow has *no* hub nodes at all —
/// the resolver early-returns in that case, so the warning must fire there too.
#[test]
fn hub_override_warns_when_no_hub_nodes() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub override no-hub-node test");
        return;
    }
    let fixture = build_fixture();
    let checkout = fixture.root.join("source/node-hub/hello"); // any existing dir
    // a dataflow with only a non-hub (dynamic) node — nothing to build
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: plain\n    path: dynamic\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    let out = dora(&fixture)
        .args([
            "build",
            flow.to_str().unwrap(),
            "--hub-override",
            &format!("test/ghost={}", checkout.display()),
        ])
        .output()
        .unwrap();
    let combined = format!("{}{}", String::from_utf8_lossy(&out.stdout), stderr(&out));
    assert!(
        combined.contains("did not match any hub node"),
        "expected an unused-override warning even with no hub nodes, got:\n{combined}"
    );
}

/// `--hub-override` + `--write-lockfile` is rejected: the override substitutes
/// local source, so a regenerated lockfile would silently drop the hub pin.
#[test]
fn hub_override_rejects_write_lockfile() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub override write-lockfile test");
        return;
    }
    let fixture = build_fixture();
    let checkout = fixture.root.join("source/node-hub/hello");
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    let out = dora(&fixture)
        .args([
            "build",
            flow.to_str().unwrap(),
            "--write-lockfile",
            "--hub-override",
            &format!("test/hub-smoke-hello={}", checkout.display()),
        ])
        .output()
        .unwrap();
    assert!(
        !out.status.success(),
        "must reject --hub-override + --write-lockfile"
    );
    assert!(
        stderr(&out).contains("write-lockfile"),
        "expected a write-lockfile rejection, got: {}",
        stderr(&out)
    );
}

/// `--hub-override` is rejected with a remote coordinator even when the
/// override package matches NO node (a typo must not silently fall through to
/// a distributed build).
#[test]
fn hub_override_rejects_remote_coordinator_even_if_unmatched() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub override coordinator test");
        return;
    }
    let fixture = build_fixture();
    let checkout = fixture.root.join("source/node-hub/hello");
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    let out = dora(&fixture)
        .args([
            "build",
            flow.to_str().unwrap(),
            "--coordinator-addr",
            "127.0.0.1",
            // a package that matches no node in the dataflow
            "--hub-override",
            &format!("test/not-a-node={}", checkout.display()),
        ])
        .output()
        .unwrap();
    assert!(
        !out.status.success(),
        "must reject --hub-override + remote coordinator regardless of match"
    );
    assert!(
        stderr(&out).contains("remote") && stderr(&out).contains("coordinator"),
        "expected a remote-coordinator rejection, got: {}",
        stderr(&out)
    );
}

/// `dora hub publish --dry-run` validates the manifest and previews the index
/// entry (the version from Cargo.toml, the resolved git pin + subdir) without
/// writing anything (P3.1).
#[test]
fn hub_publish_dry_run_previews_entry() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub publish dry-run test");
        return;
    }
    let fixture = build_fixture();
    let src = fixture.root.join("source");
    let checkout = src.join("node-hub/hello");
    let out = dora(&fixture)
        .args([
            "hub",
            "publish",
            checkout.to_str().unwrap(),
            "--dry-run",
            "--repo",
            &format!("file://{}", src.display()),
        ])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "publish --dry-run failed: {}",
        stderr(&out)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    // version 0.1.0 from Cargo.toml; subdir from the repo layout; name/namespace
    assert!(
        stdout.contains("test/hub-smoke-hello/0.1.0.yml"),
        "expected the entry path in the preview:\n{stdout}"
    );
    assert!(
        stdout.contains("subdir: node-hub/hello"),
        "expected the resolved subdir in the preview:\n{stdout}"
    );
    // dry run must not have written anything into the index
    assert!(
        !fixture
            .root
            .join("index/test/hub-smoke-hello/0.2.0.yml")
            .exists(),
        "dry-run must not write entries"
    );
}

/// End-to-end: `dora hub publish` writes a pinned index entry that `dora build`
/// then resolves and builds — the publish→consume loop, fully local (P3.1).
#[test]
fn hub_publish_then_build_resolves() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub publish→build test");
        return;
    }
    let fixture = build_fixture();
    let src = fixture.root.join("source");
    let checkout = src.join("node-hub/hello");

    // remove the fixture's pre-written entry so `publish` creates it fresh
    let entry = fixture.root.join("index/test/hub-smoke-hello/0.1.0.yml");
    std::fs::remove_file(&entry).unwrap();

    let out = dora(&fixture)
        .args([
            "hub",
            "publish",
            checkout.to_str().unwrap(),
            "--repo",
            &format!("file://{}", src.display()),
        ])
        .output()
        .unwrap();
    assert!(out.status.success(), "publish failed: {}", stderr(&out));
    assert!(entry.exists(), "publish did not write the index entry");
    let written = std::fs::read_to_string(&entry).unwrap();
    assert!(
        written.contains("subdir: node-hub/hello"),
        "entry: {written}"
    );
    assert!(
        written.contains("name: hub-smoke-hello"),
        "entry must hold the manifest: {written}"
    );

    // append-only: re-publishing the same version is refused
    let out = dora(&fixture)
        .args([
            "hub",
            "publish",
            checkout.to_str().unwrap(),
            "--repo",
            &format!("file://{}", src.display()),
        ])
        .output()
        .unwrap();
    assert!(
        !out.status.success(),
        "re-publishing same version must fail"
    );
    assert!(
        stderr(&out).contains("already exists"),
        "expected append-only rejection, got: {}",
        stderr(&out)
    );

    // the published entry resolves end-to-end through `dora build`
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    let out = dora(&fixture)
        .args(["build", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "build of a freshly-published package failed: {}",
        stderr(&out)
    );
}

/// `dora hub publish` reads the manifest and version from the *committed*
/// source at `source.rev`, not the working tree. A dirty version bump must not
/// produce an immutable entry whose version doesn't exist at the pinned commit.
#[test]
fn hub_publish_reads_committed_source_not_worktree() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub publish dirty-tree test");
        return;
    }
    let fixture = build_fixture();
    let src = fixture.root.join("source");
    let checkout = src.join("node-hub/hello");

    // free the committed version's slot so publish can write it fresh
    std::fs::remove_file(fixture.root.join("index/test/hub-smoke-hello/0.1.0.yml")).unwrap();

    // bump the version in the working tree WITHOUT committing it
    write(
        &checkout.join("Cargo.toml"),
        "[package]\nname = \"hub-smoke-hello\"\nversion = \"0.9.9\"\nedition = \"2021\"\n\
         \n[[bin]]\nname = \"hub-smoke-hello\"\npath = \"src/main.rs\"\n\n[workspace]\n",
    );

    let out = dora(&fixture)
        .args([
            "hub",
            "publish",
            checkout.to_str().unwrap(),
            "--repo",
            &format!("file://{}", src.display()),
        ])
        .output()
        .unwrap();
    assert!(out.status.success(), "publish failed: {}", stderr(&out));
    // the committed 0.1.0 is published; the uncommitted 0.9.9 is ignored
    assert!(
        fixture
            .root
            .join("index/test/hub-smoke-hello/0.1.0.yml")
            .exists(),
        "publish must use the committed version (0.1.0)"
    );
    assert!(
        !fixture
            .root
            .join("index/test/hub-smoke-hello/0.9.9.yml")
            .exists(),
        "publish must not embed the uncommitted working-tree version (0.9.9)"
    );
}

/// `--index` must not let a namespace be seeded into an index it isn't bound to
/// (spec §7.3). The `test` namespace is bound to the local `smoke` index, so
/// publishing it into the `official` index is rejected.
#[test]
fn hub_publish_rejects_index_not_bound_to_namespace() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub publish wrong-index test");
        return;
    }
    let fixture = build_fixture();
    let src = fixture.root.join("source");
    let checkout = src.join("node-hub/hello");
    let out = dora(&fixture)
        .args([
            "hub",
            "publish",
            checkout.to_str().unwrap(),
            "--repo",
            &format!("file://{}", src.display()),
            "--index",
            "official",
        ])
        .output()
        .unwrap();
    assert!(
        !out.status.success(),
        "publishing into an unbound index must fail"
    );
    assert!(
        stderr(&out).contains("is bound to index"),
        "expected a namespace-binding rejection, got: {}",
        stderr(&out)
    );
}

/// `dora hub yank` flips the `yanked` flag on a local index entry: a fresh
/// resolve then skips the version (build fails when nothing else satisfies the
/// range), and `--undo` restores it (UC10).
#[test]
fn hub_yank_skips_version_then_undo_restores() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub yank test");
        return;
    }
    let fixture = build_fixture();
    let entry = fixture.root.join("index/test/hub-smoke-hello/0.1.0.yml");
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");

    // baseline: builds before the yank
    assert!(
        dora(&fixture)
            .args(["build", flow.to_str().unwrap()])
            .output()
            .unwrap()
            .status
            .success(),
        "baseline build should succeed"
    );

    // yank it
    let out = dora(&fixture)
        .args([
            "hub",
            "yank",
            "test/hub-smoke-hello@0.1.0",
            "--reason",
            "broken",
        ])
        .output()
        .unwrap();
    assert!(out.status.success(), "yank failed: {}", stderr(&out));
    let written = std::fs::read_to_string(&entry).unwrap();
    assert!(written.contains("yanked: true"), "entry: {written}");
    assert!(written.contains("broken"), "yank reason missing: {written}");

    // a fresh resolve now finds no non-yanked version satisfying `^0.1`
    let out = dora(&fixture)
        .args(["build", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(
        !out.status.success(),
        "build must fail when the only matching version is yanked"
    );

    // undo restores it
    let out = dora(&fixture)
        .args(["hub", "yank", "test/hub-smoke-hello@0.1.0", "--undo"])
        .output()
        .unwrap();
    assert!(out.status.success(), "undo failed: {}", stderr(&out));
    assert!(
        std::fs::read_to_string(&entry)
            .unwrap()
            .contains("yanked: false"),
        "undo did not clear the flag"
    );
    assert!(
        dora(&fixture)
            .args(["build", flow.to_str().unwrap()])
            .output()
            .unwrap()
            .status
            .success(),
        "build should succeed again after undo"
    );
}

/// Re-yanking an already-yanked version with a *different* `--reason` updates
/// the recorded reason instead of silently discarding it.
#[test]
fn hub_yank_updates_reason_when_already_yanked() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub yank reason-update test");
        return;
    }
    let fixture = build_fixture();
    let entry = fixture.root.join("index/test/hub-smoke-hello/0.1.0.yml");

    let yank = |reason: &str| {
        dora(&fixture)
            .args([
                "hub",
                "yank",
                "test/hub-smoke-hello@0.1.0",
                "--reason",
                reason,
            ])
            .output()
            .unwrap()
    };

    assert!(yank("first").status.success(), "initial yank failed");
    // re-yank with a corrected reason — must take effect, not no-op
    let out = yank("corrected");
    assert!(out.status.success(), "re-yank failed: {}", stderr(&out));
    let written = std::fs::read_to_string(&entry).unwrap();
    assert!(
        written.contains("corrected") && !written.contains("first"),
        "re-yank must update the reason, got: {written}"
    );
}

/// A yank reference with `..` path segments must be rejected before any file
/// is touched — the write path can't be allowed to escape the catalog root.
#[test]
fn hub_yank_rejects_path_traversal() {
    let fixture = build_fixture();
    for bad in ["../..@0.1.0", "../x@0.1.0", "..@0.1.0"] {
        let out = dora(&fixture).args(["hub", "yank", bad]).output().unwrap();
        assert!(!out.status.success(), "`{bad}` must be rejected");
        // it must be rejected by the key-part guard (not merely a missing file),
        // proving the traversal never reached a filesystem path
        assert!(
            stderr(&out).contains("invalid package"),
            "`{bad}` should be rejected by the key-part guard, got: {}",
            stderr(&out)
        );
    }
}

/// A symlinked package directory resolving outside the catalog root must be
/// rejected — the `..` guard blocks lexical traversal; this blocks symlink
/// escape, matching the read-path confinement.
#[cfg(unix)]
#[test]
fn hub_yank_rejects_symlink_escape() {
    let fixture = build_fixture();
    // a target entry that lives OUTSIDE the catalog root
    let outside = fixture.root.join("outside/test/hub-smoke-hello");
    write(&outside.join("0.1.0.yml"), "manifest: {}\nsource: {}\n");
    // replace the in-catalog package dir with a symlink to that outside dir
    let pkg_dir = fixture.root.join("index/test/hub-smoke-hello");
    std::fs::remove_dir_all(&pkg_dir).unwrap();
    std::os::unix::fs::symlink(&outside, &pkg_dir).unwrap();

    let out = dora(&fixture)
        .args(["hub", "yank", "test/hub-smoke-hello@0.1.0"])
        .output()
        .unwrap();
    assert!(!out.status.success(), "symlink escape must be rejected");
    assert!(
        stderr(&out).contains("escapes the catalog root"),
        "expected catalog-confinement rejection, got: {}",
        stderr(&out)
    );
}

/// Yanking against a git-backed index (the official one) can't flip a file in
/// place — it prints the flag-flip PR instructions instead.
#[test]
fn hub_yank_git_index_prints_pr_instructions() {
    let fixture = build_fixture();
    let out = dora(&fixture)
        .args(["hub", "yank", "dora-rs/some-node@1.0.0"])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "yank git-index failed: {}",
        stderr(&out)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.contains("git-backed") && stdout.contains("yanked: true"),
        "expected flag-flip PR instructions, got:\n{stdout}"
    );
}

/// `dora hub outdated` reports a hub pin that is behind the index's latest
/// non-yanked version, and says nothing when up to date (P3.2).
#[test]
fn hub_outdated_reports_newer_version() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub outdated test");
        return;
    }
    let fixture = build_fixture();
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    // pin 0.1.0 in the lockfile
    assert!(
        dora(&fixture)
            .args(["build", flow.to_str().unwrap(), "--write-lockfile"])
            .output()
            .unwrap()
            .status
            .success(),
        "build --write-lockfile should succeed"
    );

    // only 0.1.0 exists -> up to date
    let out = dora(&fixture)
        .args(["hub", "outdated", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(out.status.success(), "outdated failed: {}", stderr(&out));
    assert!(
        String::from_utf8_lossy(&out.stdout).contains("up to date"),
        "expected up-to-date, got:\n{}",
        String::from_utf8_lossy(&out.stdout)
    );

    // add a newer version into the index (copy the 0.1.0 entry to 0.2.0)
    let entry_010 = fixture.root.join("index/test/hub-smoke-hello/0.1.0.yml");
    let entry_020 = fixture.root.join("index/test/hub-smoke-hello/0.2.0.yml");
    std::fs::copy(&entry_010, &entry_020).unwrap();

    let out = dora(&fixture)
        .args(["hub", "outdated", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(out.status.success(), "outdated failed: {}", stderr(&out));
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.contains("0.1.0 -> 0.2.0") && stdout.contains("newer available"),
        "expected an outdated report, got:\n{stdout}"
    );
}

/// A check that can't complete (the pinned package is gone from the index) must
/// exit non-zero and must NOT be summarized as "up to date" (P3.2).
#[test]
fn hub_outdated_errors_when_pin_unresolvable() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub outdated error test");
        return;
    }
    let fixture = build_fixture();
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    assert!(
        dora(&fixture)
            .args(["build", flow.to_str().unwrap(), "--write-lockfile"])
            .output()
            .unwrap()
            .status
            .success(),
        "build --write-lockfile should succeed"
    );

    // remove the index entry so the pin can no longer be resolved
    std::fs::remove_file(fixture.root.join("index/test/hub-smoke-hello/0.1.0.yml")).unwrap();

    let out = dora(&fixture)
        .args(["hub", "outdated", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(!out.status.success(), "a failed check must exit non-zero");
    assert!(
        !String::from_utf8_lossy(&out.stdout).contains("up to date"),
        "a failed check must not be summarized as up to date:\n{}",
        String::from_utf8_lossy(&out.stdout)
    );
}

/// `dora hub update` re-resolves hub pins to the latest in-range version and
/// rewrites the lockfile without building; a later `dora build --locked` then
/// accepts the refreshed lockfile, and re-running update is a no-op (P3.2).
#[test]
fn hub_update_bumps_pin_and_stays_locked_compatible() {
    if Command::new("git").arg("--version").output().is_err() {
        eprintln!("git not available — skipping hub update test");
        return;
    }
    let fixture = build_fixture();
    write(
        &fixture.root.join("flow/dataflow.yml"),
        "nodes:\n  - id: hello\n    hub: test/hub-smoke-hello@^0.1\n",
    );
    let flow = fixture.root.join("flow/dataflow.yml");
    let lockfile = fixture.root.join("flow/dataflow.dora-lock.yaml");

    // pin 0.1.0
    assert!(
        dora(&fixture)
            .args(["build", flow.to_str().unwrap(), "--write-lockfile"])
            .output()
            .unwrap()
            .status
            .success(),
        "initial build --write-lockfile should succeed"
    );
    assert!(
        std::fs::read_to_string(&lockfile)
            .unwrap()
            .contains("version: 0.1.0"),
        "lockfile should pin 0.1.0"
    );

    // a newer in-range version appears in the index (^0.1 admits 0.1.1)
    std::fs::copy(
        fixture.root.join("index/test/hub-smoke-hello/0.1.0.yml"),
        fixture.root.join("index/test/hub-smoke-hello/0.1.1.yml"),
    )
    .unwrap();

    // update bumps the pin (no build)
    let out = dora(&fixture)
        .args(["hub", "update", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(out.status.success(), "update failed: {}", stderr(&out));
    let after_update = std::fs::read_to_string(&lockfile).unwrap();
    assert!(
        after_update.contains("version: 0.1.1"),
        "update should bump the pin to 0.1.1"
    );

    // the real invariant: `update`'s lockfile is byte-identical to what
    // `dora build --write-lockfile` writes for the same descriptor + index
    // state (build re-resolves hub nodes fresh when not `--locked`). A wrong
    // commit/subdir/fingerprint would diverge here even though `--locked`
    // treats those as warn-only.
    let out = dora(&fixture)
        .args(["build", flow.to_str().unwrap(), "--write-lockfile"])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "reference build failed: {}",
        stderr(&out)
    );
    let after_build = std::fs::read_to_string(&lockfile).unwrap();
    assert_eq!(
        after_update, after_build,
        "update must write the same lockfile bytes as `dora build --write-lockfile`"
    );

    // and the refreshed lockfile is consumable under `--locked`
    let out = dora(&fixture)
        .args(["build", flow.to_str().unwrap(), "--locked"])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "build --locked after update failed: {}",
        stderr(&out)
    );

    // re-running update is a no-op
    let out = dora(&fixture)
        .args(["hub", "update", flow.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "second update failed: {}",
        stderr(&out)
    );
    assert!(
        String::from_utf8_lossy(&out.stdout).contains("up to date"),
        "second update should be a no-op:\n{}",
        String::from_utf8_lossy(&out.stdout)
    );
}

fn stderr(out: &std::process::Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}
