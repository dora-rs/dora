# Hub Registry Hardening

This document records the decisions and limitations from the Hub registry review
(PR #4137). It is the reference for the follow-up implementation so the code,
guide, and PR description stay aligned.

## Decisions

- **Registry resolution is opt-in.** Without `DORA_HUB_REGISTRY_URL`, unbound
  namespaces keep using the configured/default git index. Setting the variable
  (for example to `https://dora.pub`) opts into registry resolution. The build
  resolver and the discovery commands use the same predicate. Changing the
  project-wide default is a separate maintainer decision.
- **Registry publishing is opt-in.** `dora hub publish` keeps the legacy
  index-entry path unless `--registry` or `DORA_HUB_REGISTRY_URL` is used.
  A namespace explicitly bound in `hub.toml`, or an explicit `official` index,
  keeps the git/path path even when the env var is set; `--registry` is the
  deliberate per-invocation override.
- **Hub checkout caches are project-keyed.** Hub sources live under
  `<cache>/sources/<project-key>/...`, outside the dataflow directory to avoid
  Cargo workspace inference. Plain `git:` nodes keep their historical
  `<dataflow>/git` location.
- **The frozen wire `HubProvenance` is unchanged.** The registry archive
  checksum is stored in a separate top-level `hub_archive_sha256` lockfile map,
  not in the dora-message wire type.
- **Only the official registry accepts `DORA_HUB_TOKEN`.** A custom/mirror
  registry must receive `--token` or a saved per-registry token. The variable
  is not attached to clap help, and it is removed from spawned nodes/build
  scripts.
- **Publishing to the public registry asks for confirmation.** Non-interactive
  runs must pass `--yes`; `--dry-run` remains the way to inspect the payload.

## Security and reliability behavior

- Registry archive fallback is accepted only when its SHA-256 matches the
  checksum pinned in the lockfile; a missing checksum disables the fallback.
- Lockfile source URLs are revalidated before git; strict `--locked` fails when
  the index source no longer matches the pin.
- `--offline` skips the hosting-source probe and archive fallback.
- Registry responses have size limits and HTTPS registries reject downgrade
  redirects.
- Token files are created with private permissions, refuse symlink writes, and
  are keyed per registry.
- Archive extraction accepts only normal/current-dir paths and regular
  files/directories, rejects unsafe entry types, limits entry count, and caps
  expanded size.
- Registry cache refreshes are serialized per package and installed through a
  staging directory, so readers see a complete old or new view.
- `source_available` verifies the pinned commit with a temporary shallow fetch,
  not `ls-remote` ref tips; quiet/null stdio avoids the pipe-buffer deadlock,
  and warm project-keyed checkouts skip the network probe entirely.
- Artifact entries without a `platforms` list are rejected instead of matching
  every host.

## Known limitations

- The archive fallback is built from the published package subtree. A monorepo
  package that depends on its parent Cargo workspace may build from the hosting
  repository but not from the fallback archive; the failure is explicit.
- A warm cached checkout is trusted without a fresh hosting probe; this is
  intentional for availability and offline builds.
- Cross-process clone reuse of the same project remains protected by the
  project-keyed path, not a repository-wide interprocess lock.
- Windows-specific path behavior is covered by code paths and unit tests where
  possible; full Windows CI remains a nightly responsibility.

## Verification

The follow-up is validated locally with:

- `cargo fmt --all -- --check`
- clippy for `dora-cli`, `dora-hub-client`, `dora-core`, and `dora-daemon`
- `cargo test -p dora-hub-client`
- `cargo test -p dora-cli --lib`
- `dora-cli` hub-login and CLI-surface tests
- `dora-examples` hub-smoke
- an opt-in registry build, checksum-verified archive fallback, and offline
  reuse against a live package

CI PR runs intentionally skip the workspace-wide Test/E2E/contract jobs; those
remain merge-queue/nightly responsibilities. The PR description must state this
and include the local evidence.
