//! `dora hub info` — show a package's contracts and example.

use super::common::{HubContext, sanitize};
use crate::command::Executable;
use dora_hub_client::reference::PackageRef;

/// Show details of a hub node
#[derive(Debug, clap::Args)]
pub struct Info {
    /// Package reference, e.g. `dora-yolo` or `acme/lidar@2.1`
    #[clap(value_name = "PACKAGE")]
    package: String,
    /// Do not refresh indexes over the network (cache only)
    #[clap(long, action)]
    offline: bool,
}

impl Executable for Info {
    fn execute(self) -> eyre::Result<()> {
        let reference = PackageRef::parse(&self.package)?;
        let mut ctx = HubContext::load(self.offline)?;
        let catalog = ctx.catalog_for_namespace(&reference.namespace)?;

        let (version, entry) = resolve_or_yanked(&catalog, &reference, &self.package)?;
        ctx.drain_warnings();

        let m = &entry.manifest;
        println!("{} {}", reference.key(), version);
        if entry.yanked {
            println!("  (yanked)");
        }
        if let Some(desc) = &m.description {
            println!("  {}", sanitize(desc));
        }
        if !m.categories.is_empty() {
            let cats = m
                .categories
                .iter()
                .map(|c| c.as_str())
                .collect::<Vec<_>>()
                .join(", ");
            println!("  categories: {cats}");
        }
        if !m.platforms.is_empty() {
            // platforms are free-text manifest content from an untrusted index
            // entry (the discovery path only parses, never validates) — sanitize
            // like every other index-supplied field printed here
            let platforms = m
                .platforms
                .iter()
                .map(|p| sanitize(p))
                .collect::<Vec<_>>()
                .join(", ");
            println!("  platforms: {platforms}");
        }
        println!("  runtime: {:?}", m.runtime);

        print_ports("inputs", &m.inputs);
        print_ports("outputs", &m.outputs);

        if !m.env.is_empty() {
            println!("  env:");
            for (name, def) in &m.env {
                let ty = def
                    .r#type
                    .map(|t| format!(" ({})", t.as_str()))
                    .unwrap_or_default();
                println!("    {}{ty}", sanitize(name));
            }
        }
        if let Some(example) = &m.example {
            println!("  example:");
            for line in example.lines() {
                println!("    {}", sanitize(line));
            }
        }
        Ok(())
    }
}

/// Resolve `reference`, falling back to the highest **yanked** version that
/// matches when `resolve()` fails because every matching version is yanked.
///
/// `resolve()` only returns non-yanked entries, so without this an explicit (or
/// all-yanked) lookup errors instead of surfacing the package with its
/// `(yanked)` marker — the marker branch in `info` would be dead code
/// (dora-rs/dora#2275). When no matching version exists at all, the original
/// resolve error is re-raised unchanged.
fn resolve_or_yanked(
    catalog: &dora_hub_client::index::IndexCatalog,
    reference: &PackageRef,
    package_label: &str,
) -> eyre::Result<(
    dora_hub_client::semver::Version,
    dora_hub_client::index::IndexEntry,
)> {
    match catalog.resolve(reference) {
        Ok(resolved) => Ok((resolved.version, resolved.entry)),
        Err(resolve_err) => {
            // `versions()` is sorted ascending, so `.rev()` yields highest-first.
            // Use `?` rather than `.ok()` so that corrupt entry files surface their
            // parse error instead of being silently skipped — otherwise a malformed
            // non-yanked entry could be masked by a lower valid yanked result.
            for v in catalog
                .versions(&reference.namespace, &reference.name)
                .unwrap_or_default()
                .into_iter()
                .rev()
                .filter(|v| reference.requirement.matches(v))
            {
                let entry = catalog.entry(&reference.namespace, &reference.name, &v)?;
                if entry.yanked {
                    return Ok((v, entry));
                }
            }
            Err(resolve_err.wrap_err(format!("failed to resolve `{package_label}`")))
        }
    }
}

fn print_ports(
    label: &str,
    ports: &std::collections::BTreeMap<String, dora_core::manifest::PortDef>,
) {
    if ports.is_empty() {
        return;
    }
    println!("  {label}:");
    for (name, def) in ports {
        let ty = def
            .r#type
            .as_deref()
            .map(|t| format!(": {}", sanitize(t)))
            .unwrap_or_else(|| ": (untyped)".into());
        println!("    {}{ty}", sanitize(name));
    }
}

#[cfg(test)]
mod tests {
    //! Regression coverage for dora-rs/dora#2275: `info`'s `(yanked)` marker
    //! was unreachable because `resolve()` never returns yanked entries.
    use super::resolve_or_yanked;
    use dora_hub_client::index::IndexCatalog;
    use dora_hub_client::reference::PackageRef;

    fn entry_yaml(yanked: bool) -> String {
        format!(
            "manifest:\n  apiVersion: 1\n  name: dora-yolo\n  namespace: dora-rs\n  \
             runtime: python\n  entrypoint: dora-yolo\nsource:\n  \
             git: https://github.com/dora-rs/dora-hub\n  \
             rev: aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n  \
             subdir: node-hub/dora-yolo\nyanked: {yanked}\n"
        )
    }

    fn write_entry(dir: &std::path::Path, version: &str, yanked: bool) {
        std::fs::write(dir.join(format!("{version}.yml")), entry_yaml(yanked)).unwrap();
    }

    /// Catalog with `dora-rs/dora-yolo` (0.5.1 + 0.5.2 live, 0.6.0 yanked) and
    /// `dora-rs/legacy` (only version 0.1.0, yanked).
    fn fixture() -> (tempfile::TempDir, IndexCatalog) {
        let tmp = tempfile::tempdir().unwrap();
        let yolo = tmp.path().join("dora-rs/dora-yolo");
        std::fs::create_dir_all(&yolo).unwrap();
        write_entry(&yolo, "0.5.1", false);
        write_entry(&yolo, "0.5.2", false);
        write_entry(&yolo, "0.6.0", true);
        std::fs::write(yolo.join("package.yml"), "description: yolo\nowners: [x]\n").unwrap();

        let legacy = tmp.path().join("dora-rs/legacy");
        std::fs::create_dir_all(&legacy).unwrap();
        write_entry(&legacy, "0.1.0", true);
        std::fs::write(
            legacy.join("package.yml"),
            "description: legacy\nowners: [x]\n",
        )
        .unwrap();

        let catalog = IndexCatalog::open(tmp.path()).unwrap();
        (tmp, catalog)
    }

    fn parse(s: &str) -> PackageRef {
        PackageRef::parse(s).unwrap()
    }

    // The core fix: an explicitly requested yanked version is surfaced with its
    // entry (so the `(yanked)` branch is reachable), not an error.
    #[test]
    fn surfaces_explicitly_requested_yanked_version() {
        let (_tmp, catalog) = fixture();
        let (version, entry) = resolve_or_yanked(
            &catalog,
            &parse("dora-rs/dora-yolo@0.6.0"),
            "dora-rs/dora-yolo@0.6.0",
        )
        .unwrap();
        assert_eq!(version.to_string(), "0.6.0");
        assert!(entry.yanked, "the (yanked) marker branch must be reachable");
    }

    // A bare reference still resolves to the highest non-yanked version,
    // skipping the yanked 0.6.0 — the fallback must not change this.
    #[test]
    fn bare_reference_resolves_highest_non_yanked() {
        let (_tmp, catalog) = fixture();
        let (version, entry) =
            resolve_or_yanked(&catalog, &parse("dora-rs/dora-yolo"), "dora-rs/dora-yolo").unwrap();
        assert_eq!(version.to_string(), "0.5.2");
        assert!(!entry.yanked);
    }

    // A bare reference whose every matching version is yanked surfaces the
    // yanked entry instead of erroring.
    #[test]
    fn bare_reference_surfaces_yanked_when_all_yanked() {
        let (_tmp, catalog) = fixture();
        let (version, entry) =
            resolve_or_yanked(&catalog, &parse("dora-rs/legacy"), "dora-rs/legacy").unwrap();
        assert_eq!(version.to_string(), "0.1.0");
        assert!(entry.yanked);
    }

    // No matching version at all → the original resolve error is re-raised.
    #[test]
    fn errors_when_no_version_matches() {
        let (_tmp, catalog) = fixture();
        assert!(
            resolve_or_yanked(
                &catalog,
                &parse("dora-rs/nonexistent"),
                "dora-rs/nonexistent"
            )
            .is_err()
        );
    }
}
