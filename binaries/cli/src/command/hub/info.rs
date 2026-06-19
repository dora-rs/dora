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

        // resolve() only returns non-yanked entries. If it fails (e.g. every
        // matching version is yanked), fall back to the highest yanked match so
        // `info` can surface the (yanked) marker instead of an opaque error.
        let (version, entry) = match catalog.resolve(&reference) {
            Ok(resolved) => (resolved.version, resolved.entry),
            Err(resolve_err) => {
                let yanked_match = catalog
                    .versions(&reference.namespace, &reference.name)
                    .unwrap_or_default()
                    .into_iter()
                    .rev()
                    .filter(|v| reference.requirement.matches(v))
                    .find_map(|v| {
                        catalog
                            .entry(&reference.namespace, &reference.name, &v)
                            .ok()
                            .filter(|e| e.yanked)
                            .map(|e| (v, e))
                    });
                match yanked_match {
                    Some(pair) => pair,
                    None => {
                        return Err(
                            resolve_err.wrap_err(format!("failed to resolve `{}`", self.package))
                        );
                    }
                }
            }
        };
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
