# TUI Repository Split – Implementation Checklist

Now that `tui-interface` and `dora-protocol-client` are decoupled from Dora-core, we can stand up a
dedicated “Dora TUI” repository with minimal friction. This document captures the concrete steps to
create and maintain that repo while keeping the current workspace healthy.

> **Update (2025-03-28):** The checklist below has been executed. The new repository lives at
> [heyong4725/dora-tui](https://github.com/heyong4725/dora-tui) and the main workspace now depends on
> its crates via git.

## 1. Repository Skeleton

```
dora-tui/
├── Cargo.toml
├── README.md
├── crates/
│   ├── tui-interface/         # moved from monorepo (same crate name/version)
│   └── dora-protocol-client/  # moved from monorepo
├── binaries/
│   └── tui/                   # extracted from binaries/cli (feature `tui-protocol-services`)
├── examples/
├── docs/
└── .github/workflows/
```

- Workspace `Cargo.toml` declares `tui-interface`, `dora-protocol-client`, and `tui` binary.
- Keep crate names/versions identical so existing downstream `Cargo.lock`s can reuse them.
- Include README with quick-start (`cargo run -p tui --features tui-protocol-services -- tui`).

## 2. Code Migration

1. Copy `crates/tui-interface` and `crates/protocol-client` verbatim into the new repo.  
   - Update path dependencies inside the monorepo to use `git = ".."` until we switch entirely.
2. Extract TUI code:
   - Start from `binaries/cli` and copy the `tui` module (app, views, components, tests).
   - Create `binaries/tui/Cargo.toml` with `tui-interface`, `dora-protocol-client`, and public dependencies (`ratatui`, `tokio`, etc.).  
   - Remove legacy CLI-only modules (commands, automation, etc.).
3. Update `binaries/tui/src/main.rs` to parse only the `dora tui` invocation paths.
4. Ensure `tui-interface` and `dora-protocol-client` use relative workspace paths inside the new repo (no `../../`).

## 3. CI / Tooling

- Add GitHub Actions workflow:
  - `cargo fmt --all -- --check`
  - `cargo clippy --all-targets --all-features`
  - `cargo test --all --workspace`
- Optionally pin MSRV (currently `1.85.0`).
- Provide Nix or devcontainer definition mirroring Dora’s environment for consistency.

## 4. Backwards Compatibility

In the monorepo, transition to `git` dependencies for the two crates once the new repo is public:

```toml
tui-interface = { git = "https://github.com/heyong4725/dora-tui", branch = "main", package = "tui-interface", path = "crates/tui-interface" }
dora-protocol-client = { git = "https://github.com/heyong4725/dora-tui", branch = "main", package = "dora-protocol-client", path = "crates/dora-protocol-client" }
dora-protocol = { git = "https://github.com/heyong4725/dora-tui", branch = "main", package = "dora-protocol", path = "libraries/protocol" }
```

During the overlap, retain the workspace copies but mark them with `publish = false` and add a
`README.md` indicating they will relocate. After Dora’s Cargo.lock is updated to the git source,
delete the local copies to avoid duplication. *(Completed — the copies have been removed.)*

## 5. Release Process

1. Tag the new repo (`v0.1.0`) once tests pass.
2. Update Dora-core to depend on the tagged commit.
3. When stable, publish crates (`tui-interface`, `dora-protocol-client`) to crates.io if desired.

## 6. Outstanding TODOs Before Split

- Remove remaining references to `tui-cli-services` in the default build or relocate it to a
  compatibility crate if we want to keep legacy mode alive in Dora-core.
- Finalize documentation: port `docs/tui-boundary/*` relevant to TUI into the new repo (usage,
  roadmap, ADR snapshots).
- Decide how to share theme assets/help content—either copy or host them in the new repo and have
  Dora-core pull when building the bundled CLI.

Once the above are complete, we can spin up the new GitHub repository, push the copied workspace,
and wire Dora-core to the external crates. Running `cargo run -p tui --features tui-protocol-services -- tui`
in the new repo will provide the same experience we have today.***
