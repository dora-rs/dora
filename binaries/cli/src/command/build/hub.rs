//! `hub:` reference resolution — the desugar step (spec §10.1, P2.5/P2.7).
//!
//! The CLI rewrites every `hub:` node into a concrete git-sourced node
//! *before dispatch*: the index entry supplies the pinned commit (and
//! optional `subdir`), the package manifest supplies the entrypoint, build
//! command, and typed contracts. Downstream — coordinator, daemons, the
//! lockfile — sees an ordinary git node carrying the optional `subdir` and
//! hub provenance marker on its [`GitSource`].

use std::collections::BTreeMap;

use dora_core::{
    build::validate_subdir,
    manifest::inject::{InjectionResult, apply_manifest_contracts},
    types::TypeRegistry,
};
use dora_hub_client::{
    config::ResolvedConfig, index::IndexCatalog, reference::PackageRef, transport::IndexFetcher,
};
use dora_message::{
    common::{GitSource, HubProvenance},
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
}

impl HubResolution {
    pub fn is_empty(&self) -> bool {
        self.sources.is_empty()
    }
}

/// Resolve and desugar every `hub:` node in `dataflow`.
///
/// With `pins` (from a lockfile, `--locked`), no index resolution happens:
/// the pinned commit is used verbatim and the index entry of the pinned
/// version supplies the manifest. Without pins, each reference resolves to
/// the highest non-yanked matching version.
pub fn resolve_hub_nodes(
    dataflow: &mut Descriptor,
    registry: &mut TypeRegistry,
    offline: bool,
    pins: Option<&BTreeMap<NodeId, GitSource>>,
) -> eyre::Result<HubResolution> {
    let mut resolution = HubResolution::default();
    if !dataflow.nodes.iter().any(|n| n.hub.is_some()) {
        return Ok(resolution);
    }
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
        let index_config = config.index_for_namespace(&reference.namespace);
        let catalog_dir = fetcher
            .catalog_dir(index_config, &config.config_dir)
            .with_context(|| format!("node `{}`: failed to fetch the hub index", node.id))?;
        let catalog = IndexCatalog::open(&catalog_dir)?;

        let pin = pins.and_then(|p| p.get(&node.id));
        let (version, entry, source) = match pin {
            Some(pin) => {
                // --locked: the pinned commit is used verbatim, no resolution
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
                let version: dora_hub_client::semver::Version = provenance
                    .version
                    .parse()
                    .with_context(|| format!("node `{}`: invalid version in lockfile", node.id))?;
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
                                r.chars().filter(|c| !c.is_control()).collect::<String>()
                            ))
                            .unwrap_or_default()
                    ));
                }
                (version, entry, pin.clone())
            }
            None => {
                let resolved = catalog.resolve(&reference).with_context(|| {
                    format!("node `{}`: failed to resolve `{raw_reference}`", node.id)
                })?;
                let (git_url, rev) = resolved.entry.source.git_pin().with_context(|| {
                    format!(
                        "node `{}`: invalid source for `{}@{}`",
                        node.id,
                        reference.key(),
                        resolved.version
                    )
                })?;
                let source = GitSource {
                    repo: git_url.to_string(),
                    commit_hash: rev.to_string(),
                    subdir: resolved.entry.source.subdir.clone(),
                    hub: Some(HubProvenance {
                        name: reference.key(),
                        version: resolved.version.to_string(),
                    }),
                };
                (resolved.version, resolved.entry, source)
            }
        };

        if let Some(subdir) = &source.subdir {
            validate_subdir(subdir)
                .with_context(|| format!("node `{}`: invalid index entry", node.id))?;
        }

        let manifest = &entry.manifest;
        let label = format!("{}@{version}", reference.key());

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

        // desugar into a concrete git node
        node.path = Some(manifest.entrypoint.replace('\\', "/"));
        node.build = manifest.build.clone();
        node.git = Some(source.repo.clone());
        node.rev = Some(source.commit_hash.clone());
        node.hub = None;
        resolution.notes.push(format!(
            "node `{}`: resolved hub package {label} -> {}",
            node.id,
            &source.commit_hash[..source.commit_hash.len().min(12)]
        ));
        resolution.sources.insert(node.id.clone(), source);
    }

    resolution.warnings.append(&mut fetcher.warnings);
    Ok(resolution)
}
