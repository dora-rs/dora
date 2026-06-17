//! `hub:` reference resolution — the desugar step (spec §10.1, P2.5/P2.7).
//!
//! The CLI rewrites every `hub:` node into a concrete git-sourced node
//! *before dispatch*: the index entry supplies the pinned commit (and
//! optional `subdir`), the package manifest supplies the entrypoint, build
//! command, and typed contracts. Downstream — coordinator, daemons, the
//! lockfile — sees an ordinary git node carrying the optional `subdir` and
//! hub provenance marker on its [`GitSource`].

use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};

use dora_core::{
    build::validate_subdir,
    manifest::{
        MANIFEST_FILENAME, NodeManifest,
        inject::{InjectionResult, apply_manifest_contracts},
    },
    types::TypeRegistry,
};
use dora_hub_client::{
    config::ResolvedConfig,
    index::{IndexCatalog, current_platform},
    reference::PackageRef,
    transport::IndexFetcher,
};
use dora_message::{
    common::{BinaryPin, GitSource, HubProvenance},
    descriptor::Descriptor,
    id::NodeId,
};
use eyre::{Context, ContextCompat};

/// Outcome of resolving the `hub:` nodes of a dataflow.
#[derive(Debug, Default)]
pub struct HubResolution {
    /// Progress/info lines for the user.
    pub notes: Vec<String>,
    /// Non-fatal problems (yanked pins, index rollback warnings).
    pub warnings: Vec<String>,
    /// Per-node resolved git source (with subdir + hub provenance) — merged
    /// into the build's `git_sources` map and the lockfile.
    pub sources: BTreeMap<NodeId, GitSource>,
    /// Per-node resolved prebuilt-binary pin (url + sha256 + provenance) for
    /// nodes that resolved to a `binary` artifact — written to the lockfile so
    /// `--locked` reproduces the exact artifact (spec §8.2).
    pub binary_sources: BTreeMap<NodeId, BinaryPin>,
    /// Per-node local working dir for `--hub-override` substitutions (UC11):
    /// the node builds and runs from this checkout instead of a cloned commit.
    /// Threaded into the local builder so build + spawn root there.
    pub override_dirs: BTreeMap<NodeId, PathBuf>,
}

impl HubResolution {
    pub fn is_empty(&self) -> bool {
        self.sources.is_empty() && self.binary_sources.is_empty() && self.override_dirs.is_empty()
    }
}

/// Normalize an index `source.git` into a form the build's `GitManager` can
/// consume. `validate_git_url` accepts two scheme-less forms — scp-style
/// `git@host:path` and absolute local paths — but `GitManager::choose_clone_dir`
/// parses the URL with `url::Url`, which rejects both. Rewrite them into the
/// equivalent `ssh://` / `file://` URLs git treats identically, so a source
/// that resolves also clones instead of failing late during the build.
fn normalize_git_source_url(url: &str) -> String {
    if url.contains("://") {
        return url.to_string();
    }
    if let Some(rest) = url.strip_prefix("git@")
        && let Some((host, path)) = rest.split_once(':')
    {
        return format!("ssh://git@{host}/{}", path.trim_start_matches('/'));
    }
    if url.starts_with('/') {
        return format!("file://{url}");
    }
    url.to_string()
}

/// A stable SHA-256 digest of an index entry's manifest, recorded in the
/// lockfile so `--locked` can detect a rewritten index entry (entrypoint,
/// build command, or typed contract). `serde_json` sorts the manifest's
/// `BTreeMap` fields, so the serialization — and the digest — is canonical.
fn manifest_digest(manifest: &NodeManifest) -> eyre::Result<String> {
    use sha2::{Digest, Sha256};
    let bytes = serde_json::to_vec(manifest).context("failed to serialize node manifest")?;
    Ok(Sha256::digest(&bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect())
}

/// Resolve and desugar every `hub:` node in `dataflow`.
///
/// With `pins` (from a lockfile, `--locked`), no index resolution happens:
/// the pinned commit is used verbatim and the index entry of the pinned
/// version supplies the manifest. Without pins, each reference resolves to
/// the highest non-yanked matching version.
/// Where a resolved hub node's bytes come from: a git source (built from
/// source, the default) or a prebuilt per-platform binary artifact (spec §8.2).
enum NodeBytes {
    Git(GitSource),
    Binary {
        url: String,
        sha256: String,
        platform: String,
    },
}

/// Decide where a freshly-resolved entry's bytes come from (spec §8.1/§8.2):
/// a prebuilt artifact for this platform if the entry ships one, else the
/// `fallback-git` source build, else the plain git source.
fn resolve_node_bytes(
    source: &dora_hub_client::index::SourceSpec,
    manifest: &dora_core::manifest::NodeManifest,
    reference: &PackageRef,
    version: &dora_hub_client::semver::Version,
) -> eyre::Result<NodeBytes> {
    let provenance = || -> eyre::Result<HubProvenance> {
        Ok(HubProvenance {
            name: reference.key(),
            version: version.to_string(),
            manifest_digest: Some(manifest_digest(manifest)?),
        })
    };
    let git_bytes = |git: &str, rev: &str, subdir: Option<String>| -> eyre::Result<NodeBytes> {
        Ok(NodeBytes::Git(GitSource {
            repo: normalize_git_source_url(git),
            commit_hash: rev.to_string(),
            subdir,
            hub: Some(provenance()?),
        }))
    };
    if !source.binary.is_empty() {
        let platform = current_platform();
        if let Some(art) = source.select_binary(&platform) {
            art.validate()?;
            return Ok(NodeBytes::Binary {
                url: art.url.clone(),
                sha256: art.sha256.clone(),
                platform,
            });
        }
        // no artifact for this platform: degrade to a source build if the
        // entry offers one (cargo-binstall model), else hard-fail.
        if let Some(fallback) = &source.fallback_git {
            let (git, rev) = fallback.git_pin()?;
            return git_bytes(git, rev, fallback.subdir.clone());
        }
        eyre::bail!(
            "no prebuilt binary for platform `{platform}` and no `fallback-git` to build from"
        );
    }
    let (git, rev) = source.git_pin()?;
    git_bytes(git, rev, source.subdir.clone())
}

/// Resolve a node whose lockfile entry is a binary pin (`--locked`): the
/// pinned `url`+`sha256` are used verbatim, but the manifest (entrypoint,
/// build, contract) still comes from the mutable index entry — so the entry's
/// artifact is cross-checked (warn on rewrite) and its manifest digest is
/// hard-checked, mirroring the git locked path.
fn resolve_locked_binary(
    bpin: &BinaryPin,
    catalog: &IndexCatalog,
    reference: &PackageRef,
    node_id: &NodeId,
    resolution: &mut HubResolution,
) -> eyre::Result<(
    dora_hub_client::semver::Version,
    dora_hub_client::index::IndexEntry,
    NodeBytes,
)> {
    if bpin.hub.name != reference.key() {
        eyre::bail!(
            "node `{node_id}`: lockfile pins `{}` but the dataflow now references `{}` — \
             regenerate with `dora build --write-lockfile`",
            bpin.hub.name,
            reference.key()
        );
    }
    // the lockfile holds one binary pin per node, for the platform it was
    // generated on — a Linux pin must not download its artifact on macOS.
    let host = current_platform();
    if bpin.platform != host {
        eyre::bail!(
            "node `{node_id}`: the lockfile pins the binary artifact for `{}`, but this host is \
             `{host}` — regenerate the lockfile on this platform (`dora build --write-lockfile`)",
            bpin.platform
        );
    }
    let version: dora_hub_client::semver::Version = bpin
        .hub
        .version
        .parse()
        .with_context(|| format!("node `{node_id}`: invalid version in lockfile"))?;
    if !reference.requirement.matches(&version) {
        eyre::bail!(
            "node `{node_id}`: locked version {version} no longer satisfies `{}` — \
             regenerate with `dora build --write-lockfile`",
            reference.requirement
        );
    }
    let entry = catalog
        .entry(&reference.namespace, &reference.name, &version)
        .with_context(|| {
            format!(
                "node `{node_id}`: pinned version {version} of `{}` is missing from the index",
                reference.key()
            )
        })?;
    if entry.yanked {
        resolution.warnings.push(format!(
            "node `{node_id}`: pinned version {version} of `{}` has been yanked — \
             the pin keeps working, but consider updating",
            reference.key()
        ));
    }
    // the locked artifact should still match the (append-only) index entry
    let agrees = matches!(
        entry.source.select_binary(&bpin.platform),
        Some(art) if art.url == bpin.url && art.sha256 == bpin.sha256
    );
    if !agrees {
        resolution.warnings.push(format!(
            "node `{node_id}`: the index entry for `{}@{version}` no longer matches the \
             locked binary artifact for `{}` — the index may have been rewritten; \
             the lockfile pin is used",
            reference.key(),
            bpin.platform
        ));
    }
    if let Some(locked_digest) = &bpin.hub.manifest_digest
        && *locked_digest != manifest_digest(&entry.manifest)?
    {
        eyre::bail!(
            "node `{node_id}`: the locked index entry for `{}@{version}` changed its manifest \
             (entrypoint, build, or contract) since the lockfile was written — \
             regenerate with `dora build --write-lockfile`",
            reference.key()
        );
    }
    // re-validate the pin's artifact shape, mirroring the git path's commit-hash
    // re-check: a tampered lockfile must not smuggle a non-https url (which would
    // slip past the daemon's `source_is_url` gate and skip sha256 verification)
    // or a malformed digest.
    if !bpin.url.starts_with("https://") {
        eyre::bail!(
            "node `{node_id}`: lockfile binary url for `{}` is not https",
            bpin.platform
        );
    }
    if bpin.sha256.len() != 64 || !bpin.sha256.chars().all(|c| c.is_ascii_hexdigit()) {
        eyre::bail!(
            "node `{node_id}`: lockfile binary sha256 for `{}` is malformed (need 64 hex chars)",
            bpin.platform
        );
    }
    let bytes = NodeBytes::Binary {
        url: bpin.url.clone(),
        sha256: bpin.sha256.clone(),
        platform: bpin.platform.clone(),
    };
    Ok((version, entry, bytes))
}

pub fn resolve_hub_nodes(
    dataflow: &mut Descriptor,
    registry: &mut TypeRegistry,
    offline: bool,
    pins: Option<&BTreeMap<NodeId, GitSource>>,
    binary_pins: Option<&BTreeMap<NodeId, BinaryPin>>,
    overrides: &BTreeMap<String, PathBuf>,
) -> eyre::Result<HubResolution> {
    let mut resolution = HubResolution::default();
    if !dataflow.nodes.iter().any(|n| n.hub.is_some()) {
        // No hub nodes — but if `--hub-override`s were given they match nothing,
        // so surface that rather than silently ignoring the flags.
        for key in overrides.keys() {
            resolution.warnings.push(format!(
                "--hub-override `{key}` did not match any hub node in the dataflow"
            ));
        }
        return Ok(resolution);
    }
    let mut used_overrides = BTreeSet::new();
    resolution
        .notes
        .push("`hub:` is an unstable feature — its behavior may change in future releases".into());

    let config = ResolvedConfig::load_default().context("failed to load hub configuration")?;
    let mut fetcher = IndexFetcher::new(offline)?;

    for node in &mut dataflow.nodes {
        let Some(raw_reference) = node.hub.clone() else {
            continue;
        };
        // the index entry supplies source and build (spec §10.1)
        if node.path.is_some()
            || node.git.is_some()
            || node.build.is_some()
            || node.branch.is_some()
            || node.tag.is_some()
            || node.rev.is_some()
            || node.operators.is_some()
            || node.operator.is_some()
            || node.ros2.is_some()
        {
            eyre::bail!(
                "node `{}`: `hub:` is mutually exclusive with `path`, `git`, \
                 `build`, and operator fields — the hub package supplies them",
                node.id
            );
        }

        let reference = PackageRef::parse(&raw_reference)
            .with_context(|| format!("node `{}`: invalid `hub:` reference", node.id))?;

        // `--hub-override`: substitute a local checkout for this package (UC11
        // inner loop). The manifest is read from the checkout, contracts are
        // still validated, and the node builds + runs from local source — no
        // index resolution, no lockfile pin (it's an ephemeral dev override).
        if let Some(local_dir) = overrides.get(&reference.key()) {
            used_overrides.insert(reference.key());
            let manifest_path = local_dir.join(MANIFEST_FILENAME);
            let manifest = NodeManifest::read(&manifest_path).with_context(|| {
                format!(
                    "node `{}`: --hub-override for `{}` has no readable {MANIFEST_FILENAME} at `{}`",
                    node.id,
                    reference.key(),
                    manifest_path.display()
                )
            })?;
            // the local manifest is the developer's own input — validate it in
            // full (this includes the shipped-type namespace rule, §6.3)
            let issues = manifest.validate(registry);
            if !issues.is_empty() {
                eyre::bail!(
                    "node `{}`: overridden manifest `{}` is invalid:\n  - {}",
                    node.id,
                    manifest_path.display(),
                    issues
                        .iter()
                        .map(|i| format!("{}: {}", i.field, i.message))
                        .collect::<Vec<_>>()
                        .join("\n  - ")
                );
            }
            for (urn, def) in &manifest.types {
                if registry.resolve(urn).is_none() {
                    let _ = registry.add_user_type(urn, def.clone());
                }
            }
            let label = format!("{} (local override)", reference.key());
            let mut contracts = InjectionResult::default();
            apply_manifest_contracts(node, &manifest, &label, registry, &mut contracts);
            if !contracts.warnings.is_empty() {
                eyre::bail!(
                    "node `{}`: does not match the `{label}` package contract:\n  - {}",
                    node.id,
                    contracts.warnings.join("\n  - ")
                );
            }
            resolution.notes.extend(contracts.notes);
            // desugar to a local path node; the checkout is the build/run dir
            node.path = Some(manifest.entrypoint.replace('\\', "/"));
            node.build = manifest.build.clone();
            node.hub = None;
            resolution.notes.push(format!(
                "node `{}`: overriding hub package {} with local checkout `{}` \
                 (local source is trusted: spawned without hub `$PATH` confinement)",
                node.id,
                reference.key(),
                local_dir.display()
            ));
            resolution
                .override_dirs
                .insert(node.id.clone(), local_dir.clone());
            continue;
        }

        // under `--locked` (pins present), a node with no lockfile entry must
        // fail fast — *before* any index fetch — rather than hitting the
        // network (or an offline cache miss) only to bail afterwards.
        let pin = pins.and_then(|p| p.get(&node.id));
        let binary_pin = binary_pins.and_then(|p| p.get(&node.id));
        if pins.is_some() && pin.is_none() && binary_pin.is_none() {
            eyre::bail!(
                "node `{}`: `hub:` reference is not in the lockfile — \
                 regenerate with `dora build --write-lockfile`",
                node.id
            );
        }

        let index_config = config.index_for_namespace(&reference.namespace);
        let catalog_dir = fetcher
            .catalog_dir(index_config, &config.config_dir)
            .with_context(|| format!("node `{}`: failed to fetch the hub index", node.id))?;
        let catalog = IndexCatalog::open(&catalog_dir)?;

        let (version, entry, bytes) = if let Some(bpin) = binary_pin {
            resolve_locked_binary(bpin, &catalog, &reference, &node.id, &mut resolution)?
        } else {
            match pin {
                Some(pin) => {
                    // --locked: the pinned commit is used verbatim, no resolution
                    let valid_hash = matches!(pin.commit_hash.len(), 40 | 64)
                        && pin.commit_hash.chars().all(|c| c.is_ascii_hexdigit());
                    if !valid_hash {
                        eyre::bail!(
                            "node `{}`: lockfile commit hash is not a valid hex hash",
                            node.id
                        );
                    }
                    let provenance = pin.hub.as_ref().with_context(|| {
                        format!(
                            "node `{}`: lockfile entry has no hub provenance — \
                         regenerate with `dora build --write-lockfile`",
                            node.id
                        )
                    })?;
                    if provenance.name != reference.key() {
                        eyre::bail!(
                            "node `{}`: lockfile pins `{}` but the dataflow now references \
                         `{}` — regenerate with `dora build --write-lockfile`",
                            node.id,
                            provenance.name,
                            reference.key()
                        );
                    }
                    let version: dora_hub_client::semver::Version =
                        provenance.version.parse().with_context(|| {
                            format!("node `{}`: invalid version in lockfile", node.id)
                        })?;
                    if !reference.requirement.matches(&version) {
                        eyre::bail!(
                            "node `{}`: locked version {version} no longer satisfies `{}` — \
                         regenerate with `dora build --write-lockfile`",
                            node.id,
                            reference.requirement
                        );
                    }
                    let entry = catalog
                        .entry(&reference.namespace, &reference.name, &version)
                        .with_context(|| {
                            format!(
                                "node `{}`: pinned version {version} of `{}` is missing \
                             from the index",
                                node.id,
                                reference.key()
                            )
                        })?;
                    if entry.yanked {
                        resolution.warnings.push(format!(
                            "node `{}`: pinned version {version} of `{}` has been yanked{} — \
                         the pin keeps working, but consider updating",
                            node.id,
                            reference.key(),
                            entry
                                .yank_reason
                                .as_deref()
                                .map(|r| format!(
                                    " ({})",
                                    r.chars()
                                        .filter(|c| !c.is_control())
                                        .take(200)
                                        .collect::<String>()
                                ))
                                .unwrap_or_default()
                        ));
                    }
                    // the lockfile is authoritative for the pin, but the index
                    // entry of the pinned version should still agree — a mismatch
                    // means the append-only index was tampered with or rewritten
                    match entry.source.git_pin() {
                        Ok((git, rev))
                            if normalize_git_source_url(git) != pin.repo
                                || rev != pin.commit_hash
                                || entry.source.subdir != pin.subdir =>
                        {
                            resolution.warnings.push(format!(
                                "node `{}`: the index entry for `{}@{version}` no longer matches \
                             the lockfile pin — the index may have been rewritten; \
                             the lockfile pin is used",
                                node.id,
                                reference.key()
                            ));
                        }
                        Err(_) => {
                            resolution.warnings.push(format!(
                                "node `{}`: the index entry for `{}@{version}` has a malformed \
                             source — the index may have been rewritten; \
                             the lockfile pin is used",
                                node.id,
                                reference.key()
                            ));
                        }
                        _ => {}
                    }
                    // the commit hash pins the source *tree*, but the manifest
                    // (entrypoint, build command, and the typed contract) comes from
                    // the *mutable* index entry — so a rewritten entry could change
                    // any of it at a pinned version. `--locked` hard-errors if the
                    // manifest digest no longer matches what was locked. The digest
                    // doubles as the new-lockfile-format sentinel: when absent
                    // (lockfile predates this field) the check is skipped.
                    if let Some(provenance) = &pin.hub
                        && let Some(locked_digest) = &provenance.manifest_digest
                        && *locked_digest != manifest_digest(&entry.manifest)?
                    {
                        eyre::bail!(
                            "node `{}`: the locked index entry for `{}@{version}` changed its \
                         manifest (entrypoint, build, or contract) since the lockfile was \
                         written — regenerate with `dora build --write-lockfile`",
                            node.id,
                            reference.key()
                        );
                    }
                    // a git pin only — binary pins take the `resolve_locked_binary`
                    // path above, so this arm is always a git source.
                    (version, entry, NodeBytes::Git(pin.clone()))
                }
                None => {
                    let resolved = catalog.resolve(&reference).with_context(|| {
                        format!("node `{}`: failed to resolve `{raw_reference}`", node.id)
                    })?;
                    let bytes = resolve_node_bytes(
                        &resolved.entry.source,
                        &resolved.entry.manifest,
                        &reference,
                        &resolved.version,
                    )
                    .with_context(|| {
                        format!(
                            "node `{}`: invalid source for `{}@{}`",
                            node.id,
                            reference.key(),
                            resolved.version
                        )
                    })?;
                    (resolved.version, resolved.entry, bytes)
                }
            }
        };

        if let NodeBytes::Git(git) = &bytes
            && let Some(subdir) = &git.subdir
        {
            validate_subdir(subdir)
                .with_context(|| format!("node `{}`: invalid index entry", node.id))?;
        }

        let manifest = &entry.manifest;
        let label = format!("{}@{version}", reference.key());

        // The index entry is untrusted. Its self-declared `namespace` must
        // match the namespace the entry was actually fetched under
        // (`reference.namespace`): `shipped_type_issues` gates URNs against
        // `manifest.namespace`, so an entry that sets `namespace: victim` and
        // ships `victim/...` types would bypass the cross-namespace guard if we
        // didn't bind it to the requested namespace first (spec §6.3, P2.12).
        if manifest.namespace != reference.namespace {
            eyre::bail!(
                "node `{}`: the index entry for `{}` declares namespace `{}` \
                 but was fetched under namespace `{}` — the index may have been tampered with",
                node.id,
                reference.key(),
                manifest
                    .namespace
                    .chars()
                    .filter(|c| !c.is_control())
                    .collect::<String>(),
                reference.namespace
            );
        }

        // With the namespace now verified, validate shipped types
        // (materializable bodies, no `std/` override, no cross-namespace URN).
        let type_issues = manifest.shipped_type_issues(registry);
        if !type_issues.is_empty() {
            eyre::bail!(
                "node `{}`: the `{label}` package ships invalid custom types — \
                 the index entry may be malformed or rewritten:\n  - {}",
                node.id,
                type_issues
                    .iter()
                    // route through `ManifestIssue`'s Display, which strips
                    // control chars — `i.message` can embed an unvalidated
                    // field `type:` string from a hostile manifest
                    .map(|i| i.to_string())
                    .collect::<Vec<_>>()
                    .join("\n  - ")
            );
        }
        // register the package's shipped types, then inject contracts and
        // check the dataflow's wiring against the declared ports
        for (urn, def) in &manifest.types {
            if registry.resolve(urn).is_none() {
                let _ = registry.add_user_type(urn, def.clone());
            }
        }
        let mut contracts = InjectionResult::default();
        apply_manifest_contracts(node, manifest, &label, registry, &mut contracts);
        if !contracts.warnings.is_empty() {
            // for hub packages the manifest is the contract — violations are
            // errors, not warnings (spec §10.1)
            eyre::bail!(
                "node `{}`: does not match the `{label}` package contract:\n  - {}",
                node.id,
                contracts.warnings.join("\n  - ")
            );
        }
        resolution.notes.extend(contracts.notes);

        // desugar into a concrete node
        match bytes {
            NodeBytes::Git(source) => {
                node.path = Some(manifest.entrypoint.replace('\\', "/"));
                node.build = manifest.build.clone();
                node.git = Some(source.repo.clone());
                node.rev = Some(source.commit_hash.clone());
                node.path_sha256 = None;
                node.hub = None;
                let short_hash: String = source.commit_hash.chars().take(12).collect();
                resolution.notes.push(format!(
                    "node `{}`: resolved hub package {label} -> {short_hash}",
                    node.id
                ));
                resolution.sources.insert(node.id.clone(), source);
            }
            NodeBytes::Binary {
                url,
                sha256,
                platform,
            } => {
                // a prebuilt artifact: a sha256-verified URL download, no clone
                // or build (spec §8.2). Recorded in the lockfile so `--locked`
                // reproduces this exact artifact.
                let pin = BinaryPin {
                    platform: platform.clone(),
                    url: url.clone(),
                    sha256: sha256.clone(),
                    hub: HubProvenance {
                        name: reference.key(),
                        version: version.to_string(),
                        manifest_digest: Some(manifest_digest(manifest)?),
                    },
                };
                node.path = Some(url);
                node.path_sha256 = Some(sha256);
                node.build = None;
                node.git = None;
                node.rev = None;
                node.hub = None;
                resolution.notes.push(format!(
                    "node `{}`: resolved hub package {label} -> prebuilt binary for {platform}",
                    node.id
                ));
                resolution.binary_sources.insert(node.id.clone(), pin);
            }
        }
    }

    for key in overrides.keys() {
        if !used_overrides.contains(key) {
            resolution.warnings.push(format!(
                "--hub-override `{key}` did not match any hub node in the dataflow"
            ));
        }
    }

    resolution.warnings.append(&mut fetcher.warnings);
    Ok(resolution)
}

#[cfg(test)]
mod tests {
    use super::normalize_git_source_url;

    #[test]
    fn normalizes_scp_and_path_sources_to_parseable_urls() {
        // every `expected` is a scheme-prefixed URL `url::Url::parse` accepts,
        // which is exactly what GitManager::choose_clone_dir needs.
        let cases = [
            // scp-style → ssh://
            (
                "git@github.com:org/repo.git",
                "ssh://git@github.com/org/repo.git",
            ),
            ("/srv/mirrors/repo", "file:///srv/mirrors/repo"),
            // already-parseable forms are left untouched
            (
                "https://github.com/org/repo.git",
                "https://github.com/org/repo.git",
            ),
            ("ssh://git@host/org/repo", "ssh://git@host/org/repo"),
            ("file:///local/repo", "file:///local/repo"),
        ];
        for (input, expected) in cases {
            assert_eq!(normalize_git_source_url(input), expected, "input `{input}`");
        }
    }

    #[test]
    fn binary_source_selects_falls_back_or_errors() {
        use dora_core::manifest::NodeManifest;
        use dora_hub_client::index::{BinaryArtifact, SourceSpec, current_platform};

        use super::{NodeBytes, PackageRef, resolve_node_bytes};

        let manifest = NodeManifest::parse(
            "apiVersion: 1\nname: dora-yolo\nnamespace: dora-rs\nruntime: python\nentrypoint: dora-yolo\n",
        )
        .unwrap();
        let reference = PackageRef::parse("dora-yolo@^0.5").unwrap();
        let version = dora_hub_client::semver::Version::parse("0.5.0").unwrap();
        let art = |p: &str| BinaryArtifact {
            platform: p.to_string(),
            url: format!("https://example.com/{p}"),
            sha256: "a".repeat(64),
        };

        // covered platform -> a prebuilt binary
        let covered = SourceSpec {
            binary: vec![art(&current_platform())],
            ..Default::default()
        };
        assert!(matches!(
            resolve_node_bytes(&covered, &manifest, &reference, &version).unwrap(),
            NodeBytes::Binary { .. }
        ));

        // uncovered platform + fallback-git -> degrade to a source build
        let fallback = SourceSpec {
            binary: vec![art("nonexistent-arch")],
            fallback_git: Some(Box::new(SourceSpec {
                git: Some("https://github.com/acme/lidar".into()),
                rev: Some("a".repeat(40)),
                ..Default::default()
            })),
            ..Default::default()
        };
        assert!(matches!(
            resolve_node_bytes(&fallback, &manifest, &reference, &version).unwrap(),
            NodeBytes::Git(_)
        ));

        // uncovered platform + no fallback -> hard error
        let uncovered = SourceSpec {
            binary: vec![art("nonexistent-arch")],
            ..Default::default()
        };
        assert!(resolve_node_bytes(&uncovered, &manifest, &reference, &version).is_err());

        // plain git source (no binary) -> git
        let git = SourceSpec {
            git: Some("https://github.com/dora-rs/dora-hub".into()),
            rev: Some("b".repeat(40)),
            ..Default::default()
        };
        assert!(matches!(
            resolve_node_bytes(&git, &manifest, &reference, &version).unwrap(),
            NodeBytes::Git(_)
        ));
    }

    #[test]
    fn locked_binary_rejects_platform_mismatch() {
        use dora_hub_client::index::IndexCatalog;
        use dora_message::common::{BinaryPin, HubProvenance};

        use super::{HubResolution, PackageRef, resolve_locked_binary};

        // an empty catalog is fine: the platform check fails before any lookup
        let tmp = tempfile::tempdir().unwrap();
        let catalog = IndexCatalog::open(tmp.path()).unwrap();
        let reference = PackageRef::parse("dora-rs/lidar@^2").unwrap();
        let bpin = BinaryPin {
            platform: "some-other-platform-not-this-host".into(),
            url: "https://example.com/x".into(),
            sha256: "a".repeat(64),
            hub: HubProvenance {
                name: reference.key(),
                version: "2.1.0".into(),
                manifest_digest: None,
            },
        };
        let mut resolution = HubResolution::default();
        let Err(err) = resolve_locked_binary(
            &bpin,
            &catalog,
            &reference,
            &"lidar".parse().unwrap(),
            &mut resolution,
        ) else {
            panic!("expected a platform-mismatch error");
        };
        assert!(err.to_string().contains("this host"), "{err}");
    }
}
