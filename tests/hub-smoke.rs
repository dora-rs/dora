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

fn stderr(out: &std::process::Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}
