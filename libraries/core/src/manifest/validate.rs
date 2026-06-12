//! Static validation of node manifests (spec §5/§11).
//!
//! This is the core of `dora hub publish --dry-run` and
//! `dora validate --node-manifest <file>`: schema sanity, entrypoint path
//! confinement rules, the env-var deny-list, and type-URN resolution.

use semver::VersionReq;

use super::{EnvDefault, EnvVarType, NodeManifest};
use crate::types::{TypeRegistry, parse_urn};

/// A single validation problem, referencing the manifest field it concerns.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ManifestIssue {
    /// Manifest field the issue concerns (e.g. `entrypoint`, `inputs.image.type`).
    pub field: String,
    /// Human-readable problem description.
    pub message: String,
}

impl std::fmt::Display for ManifestIssue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // field/message embed untrusted manifest content — strip control
        // characters so a hostile manifest cannot inject terminal escapes
        write_sanitized(f, &self.field)?;
        f.write_str(": ")?;
        write_sanitized(f, &self.message)
    }
}

fn write_sanitized(f: &mut std::fmt::Formatter<'_>, s: &str) -> std::fmt::Result {
    // newlines are stripped too: no issue message is legitimately multi-line,
    // and an embedded `\n` would let a manifest spoof additional output lines
    for c in s.chars().filter(|c| !c.is_control()) {
        std::fmt::Write::write_char(f, c)?;
    }
    Ok(())
}

/// Environment variable names that could hijack the loader, the interpreter,
/// or the shell of the spawned node (spec §11). Compared case-insensitively.
const ENV_DENY_EXACT: &[&str] = &[
    "PATH",
    "PYTHONPATH",
    "PYTHONHOME",
    "PYTHONSTARTUP",
    "PYTHONEXECUTABLE",
    "PYTHONBREAKPOINT",
    "VIRTUAL_ENV",
    "BASH_ENV",
    "ENV",
    "IFS",
];
const ENV_DENY_PREFIX: &[&str] = &["LD_", "DYLD_"];

/// Platform identifiers accepted in `platforms:` (spec §5).
const KNOWN_OS: &[&str] = &["linux", "macos", "windows"];
const KNOWN_ARCH: &[&str] = &["x86_64", "aarch64", "armv7"];

impl NodeManifest {
    /// Validate the manifest against the schema rules and the given type
    /// registry. Returns all problems found; an empty vec means valid.
    pub fn validate(&self, registry: &TypeRegistry) -> Vec<ManifestIssue> {
        let mut issues = Vec::new();
        let mut issue = |field: &str, message: String| {
            issues.push(ManifestIssue {
                field: field.to_string(),
                message,
            });
        };

        if self.api_version != super::MANIFEST_API_VERSION {
            issue(
                "apiVersion",
                format!(
                    "unsupported manifest version {} (this dora supports {})",
                    self.api_version,
                    super::MANIFEST_API_VERSION
                ),
            );
        }

        match &self.name {
            None => issue("name", "missing (required)".into()),
            Some(name) => {
                if let Some(problem) = check_package_name(name) {
                    issue("name", problem);
                }
            }
        }
        if let Some(problem) = check_namespace(&self.namespace) {
            issue("namespace", problem);
        }

        if let Some(problem) = check_entrypoint(&self.entrypoint) {
            issue("entrypoint", problem);
        }

        for platform in &self.platforms {
            if let Some(problem) = check_platform(platform) {
                issue("platforms", problem);
            }
        }

        if let Some(req) = &self.dora
            && VersionReq::parse(req).is_err()
        {
            issue("dora", format!("`{req}` is not a valid semver requirement"));
        }

        // Validate shipped type bodies against a registry that includes the
        // manifest's own types, so that a port referencing a shipped type is
        // backed by a type that can actually be materialized (the P1.3 gate
        // must reject unresolvable field types, not just unknown URN keys).
        let mut local_registry = registry.clone();
        for (urn, def) in &self.types {
            local_registry.insert_type(urn.clone(), def.clone());
        }
        for (urn, def) in &self.types {
            if let Some(problem) = check_shipped_type_urn(urn, &self.namespace) {
                issue(&format!("types.{urn}"), problem);
                continue;
            }
            for field in &def.fields {
                if !local_registry.field_type_resolves(&field.r#type) {
                    issue(
                        &format!("types.{urn}.fields.{}", field.name),
                        format!("unknown field type `{}`", field.r#type),
                    );
                }
            }
        }

        for (direction, ports) in [("inputs", &self.inputs), ("outputs", &self.outputs)] {
            for (port, def) in ports {
                if let Some(problem) = check_port_name(port) {
                    issue(&format!("{direction}.{port}"), problem);
                }
                if let Some(urn) = &def.r#type
                    && let Some(problem) = self.check_port_type(urn, registry)
                {
                    issue(&format!("{direction}.{port}.type"), problem);
                }
                if direction == "outputs" && def.required.is_some() {
                    issue(
                        &format!("outputs.{port}.required"),
                        "`required` is only meaningful on inputs".into(),
                    );
                }
            }
        }

        for (name, def) in &self.env {
            if let Some(problem) = check_env_name(name) {
                issue(&format!("env.{name}"), problem);
            }
            if let (Some(declared), Some(default)) = (def.r#type, &def.default)
                && !env_default_matches(declared, default)
            {
                issue(
                    &format!("env.{name}.default"),
                    format!(
                        "default value does not match declared type `{}`",
                        declared.as_str()
                    ),
                );
            }
        }

        if let Some(example) = &self.example
            && serde_yaml::from_str::<serde_yaml::Value>(example).is_err()
        {
            issue("example", "is not valid YAML".into());
        }

        issues
    }

    /// Like [`validate`](Self::validate), but returns an error listing all
    /// problems if any were found.
    pub fn validate_strict(&self, registry: &TypeRegistry) -> eyre::Result<()> {
        let issues = self.validate(registry);
        if issues.is_empty() {
            return Ok(());
        }
        let list = issues
            .iter()
            .map(|i| format!("  - {i}"))
            .collect::<Vec<_>>()
            .join("\n");
        eyre::bail!(
            "node manifest validation failed ({} problem(s)):\n{list}",
            issues.len()
        );
    }

    /// Check that a port type URN resolves: against the registry, or against
    /// the custom types shipped in this manifest.
    fn check_port_type(&self, urn: &str, registry: &TypeRegistry) -> Option<String> {
        if registry.resolve(urn).is_some() {
            return None;
        }
        let base = parse_urn(urn)
            .map(|p| p.base)
            .unwrap_or_else(|| urn.to_string());
        if self.types.contains_key(&base) {
            return None;
        }
        if has_namespace_prefix(&base, &self.namespace) {
            return Some(format!(
                "unknown type `{urn}` — declare it under `types:` in this manifest"
            ));
        }
        match registry.suggest(urn) {
            Some(suggestion) => Some(format!(
                "unknown type `{urn}` — did you mean `{suggestion}`?"
            )),
            None => Some(format!("unknown type `{urn}`")),
        }
    }
}

/// Package names: lowercase alphanumeric plus `-`, `_`, `.`; must start and
/// end alphanumeric (PEP 503-normalized names satisfy this).
fn check_package_name(name: &str) -> Option<String> {
    if name.is_empty() {
        return Some("must not be empty".into());
    }
    if name.len() > 64 {
        return Some("must be at most 64 characters".into());
    }
    let valid_char = |c: char| c.is_ascii_lowercase() || c.is_ascii_digit() || "-_.".contains(c);
    if !name.chars().all(valid_char) {
        return Some(format!(
            "`{name}` may only contain lowercase letters, digits, `-`, `_`, `.`"
        ));
    }
    let first = name.chars().next().unwrap();
    let last = name.chars().last().unwrap();
    if !first.is_ascii_alphanumeric() || !last.is_ascii_alphanumeric() {
        return Some(format!("`{name}` must start and end alphanumeric"));
    }
    None
}

/// Namespaces are GitHub orgs/users: lowercase alphanumeric plus `-`, no
/// leading/trailing/double hyphen, at most 39 characters.
fn check_namespace(ns: &str) -> Option<String> {
    if ns.is_empty() {
        return Some("must not be empty".into());
    }
    if ns.len() > 39 {
        return Some("must be at most 39 characters (GitHub limit)".into());
    }
    let valid_char = |c: char| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-';
    if !ns.chars().all(valid_char) {
        return Some(format!(
            "`{ns}` may only contain lowercase letters, digits, and `-`"
        ));
    }
    if ns.starts_with('-') || ns.ends_with('-') || ns.contains("--") {
        return Some(format!("`{ns}` has invalid hyphen placement"));
    }
    if ns == "std" {
        return Some("`std` is reserved for the built-in type library".into());
    }
    None
}

/// Entrypoints must be relative paths confined to the node's working dir:
/// no absolute paths, no `..` components (spec §11). The character set is
/// deliberately conservative — console scripts and build outputs only need
/// alphanumerics plus `. _ - / \`; rejecting everything else (whitespace,
/// shell metacharacters, control characters, `~`, drive letters) keeps the
/// entrypoint a single unambiguous path token for the spawn side.
fn check_entrypoint(entrypoint: &str) -> Option<String> {
    if entrypoint.is_empty() {
        return Some("must not be empty".into());
    }
    let valid_char =
        |c: char| c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-' | '/' | '\\');
    if !entrypoint.chars().all(valid_char) {
        return Some(format!(
            "`{entrypoint}` may only contain alphanumerics and `. _ - / \\`"
        ));
    }
    if entrypoint.starts_with('/') || entrypoint.starts_with('\\') {
        return Some(format!("`{entrypoint}` must be a relative path"));
    }
    // Split on both separators regardless of host platform: a manifest is
    // validated once but may run anywhere, so `..` must be caught whether
    // the consuming platform treats `\` as a separator or not.
    if entrypoint.split(['/', '\\']).any(|c| c == "..") {
        return Some(format!("`{entrypoint}` must not contain `..` components"));
    }
    None
}

fn check_platform(platform: &str) -> Option<String> {
    let Some((os, arch)) = platform.split_once('-') else {
        return Some(format!(
            "`{platform}` is not of the form `<os>-<arch>` (e.g. `linux-x86_64`)"
        ));
    };
    if !KNOWN_OS.contains(&os) {
        return Some(format!(
            "unknown OS `{os}` in `{platform}` (known: {})",
            KNOWN_OS.join(", ")
        ));
    }
    if !KNOWN_ARCH.contains(&arch) {
        return Some(format!(
            "unknown architecture `{arch}` in `{platform}` (known: {})",
            KNOWN_ARCH.join(", ")
        ));
    }
    None
}

/// Shipped custom types must live under the package's namespace (spec §6.3),
/// be declared without parameters, and use the URN character set.
fn check_shipped_type_urn(urn: &str, namespace: &str) -> Option<String> {
    let valid_char = |c: char| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.' | '/');
    if !urn.chars().all(valid_char) {
        return Some(format!(
            "type URN `{urn}` may only contain alphanumerics and `_ - . /`"
        ));
    }
    if urn.starts_with("std/") || urn == "std" {
        return Some("shipped types cannot use the `std/` prefix".into());
    }
    if !has_namespace_prefix(urn, namespace) {
        return Some(format!(
            "shipped types must live under this package's namespace (`{namespace}/…`)"
        ));
    }
    None
}

/// Whether a declared env default value is of the declared type. An integer
/// default satisfies a `float` declaration (YAML `0` parses as an integer).
fn env_default_matches(declared: EnvVarType, default: &EnvDefault) -> bool {
    match declared {
        // any scalar is a valid string default: a `type: string` var with an
        // unquoted YAML scalar (`default: true`, `default: 8080`) deserializes
        // to Bool/Integer, but the publisher means the literal string — don't
        // force them to quote it
        EnvVarType::String => true,
        EnvVarType::Int => matches!(default, EnvDefault::Integer(_)),
        EnvVarType::Float => matches!(default, EnvDefault::Float(_) | EnvDefault::Integer(_)),
        EnvVarType::Bool => matches!(default, EnvDefault::Bool(_)),
    }
}

/// Whether `urn` starts with `<namespace>/`.
fn has_namespace_prefix(urn: &str, namespace: &str) -> bool {
    urn.strip_prefix(namespace)
        .is_some_and(|rest| rest.starts_with('/'))
}

/// Port names must be valid dataflow `DataId`s (the runtime's own rule),
/// minus `/`, which the descriptor syntax uses as a separator.
fn check_port_name(name: &str) -> Option<String> {
    if let Err(err) = name.parse::<dora_message::id::DataId>() {
        return Some(format!("port name `{name}` is invalid: {err}"));
    }
    if name.contains('/') {
        return Some(format!("port name `{name}` must not contain `/`"));
    }
    None
}

/// Reject environment variables that could hijack the dynamic loader or the
/// Python interpreter of the spawned node (spec §11). Names must be valid
/// environment variable identifiers — this also closes whitespace-padding
/// tricks like `" PATH "` slipping past the deny-list.
fn check_env_name(name: &str) -> Option<String> {
    let mut chars = name.chars();
    let valid_first = chars
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_');
    if !valid_first || !chars.all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return Some(format!(
            "`{name}` is not a valid environment variable name \
             (expected `[A-Za-z_][A-Za-z0-9_]*`)"
        ));
    }
    let upper = name.to_ascii_uppercase();
    if ENV_DENY_EXACT.contains(&upper.as_str())
        || ENV_DENY_PREFIX.iter().any(|p| upper.starts_with(p))
    {
        return Some(format!(
            "`{name}` is security-sensitive and cannot be declared by a node manifest"
        ));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifest::NodeManifest;

    fn minimal(extra: &str) -> NodeManifest {
        NodeManifest::parse(&format!(
            r#"
apiVersion: 1
name: dora-test
namespace: dora-rs
runtime: python
entrypoint: dora-test
{extra}"#,
        ))
        .unwrap()
    }

    fn validate(extra: &str) -> Vec<ManifestIssue> {
        minimal(extra).validate(&TypeRegistry::new())
    }

    #[test]
    fn minimal_manifest_is_valid() {
        assert_eq!(validate(""), vec![]);
    }

    #[test]
    fn spec_example_is_valid() {
        // BBox2D from the spec text doesn't exist in std/vision/v1 — use the
        // real BoundingBox type.
        let issues = validate(
            r#"
dora: ">=0.4"
inputs:
  image:
    type: std/media/v1/Image
outputs:
  bbox:
    type: std/vision/v1/BoundingBox
"#,
        );
        assert_eq!(issues, vec![]);
    }

    #[test]
    fn rejects_wrong_api_version() {
        let mut m = minimal("");
        m.api_version = 99;
        let issues = m.validate(&TypeRegistry::new());
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].field, "apiVersion");
    }

    #[test]
    fn rejects_missing_name() {
        let mut m = minimal("");
        m.name = None;
        let issues = m.validate(&TypeRegistry::new());
        assert!(issues.iter().any(|i| i.field == "name"));
    }

    #[test]
    fn rejects_bad_names() {
        for bad in ["UPPER", "-leading", "trailing-", "has space", "ünicode"] {
            assert!(check_package_name(bad).is_some(), "{bad} should be invalid");
        }
        for good in ["dora-yolo", "lidar_driver", "node.v2", "x"] {
            assert!(check_package_name(good).is_none(), "{good} should be valid");
        }
    }

    #[test]
    fn rejects_bad_namespaces() {
        for bad in ["", "-acme", "acme-", "ac--me", "Acme", "a.b"] {
            assert!(check_namespace(bad).is_some(), "{bad:?} should be invalid");
        }
        for good in ["acme", "dora-rs", "a1"] {
            assert!(check_namespace(good).is_none(), "{good} should be valid");
        }
    }

    #[test]
    fn rejects_escaping_entrypoints() {
        for bad in [
            "/usr/bin/sh",
            "../outside",
            "foo/../../bin",
            "C:\\windows\\evil.exe",
            "\\\\server\\share",
            "~/x",
            "",
        ] {
            assert!(check_entrypoint(bad).is_some(), "{bad:?} should be invalid");
        }
        for good in ["dora-yolo", "target/release/node", "build/lidar", "a.py"] {
            assert!(check_entrypoint(good).is_none(), "{good} should be valid");
        }
        // shell metacharacters, whitespace, control chars are out of charset
        for bad in ["foo; rm -rf x", "foo bar", "a|b", "a$(b)", "a\x1b[2Jb"] {
            assert!(check_entrypoint(bad).is_some(), "{bad:?} should be invalid");
        }
    }

    #[test]
    fn rejects_required_on_outputs() {
        let issues = validate("outputs:\n  bbox:\n    required: true\n");
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].field, "outputs.bbox.required");
    }

    #[test]
    fn rejects_std_namespace() {
        assert!(check_namespace("std").is_some());
    }

    #[test]
    fn display_strips_control_characters() {
        let issue = ManifestIssue {
            field: "name".into(),
            message: "`\x1b[2Jevil\x07` is bad".into(),
        };
        let rendered = issue.to_string();
        assert!(!rendered.contains('\x1b'), "{rendered:?}");
        assert!(!rendered.contains('\x07'), "{rendered:?}");
        assert!(rendered.contains("evil"), "{rendered:?}");
    }

    #[test]
    fn rejects_denied_env_vars() {
        for bad in [
            "PATH",
            "path",
            "PaTh",
            "PYTHONPATH",
            "pYtHoNpAtH",
            "PYTHONSTARTUP",
            "VIRTUAL_ENV",
            "BASH_ENV",
            "IFS",
            "LD_PRELOAD",
            "DYLD_INSERT_LIBRARIES",
        ] {
            let issues = validate(&format!("env:\n  {bad}:\n    default: x\n"));
            assert_eq!(issues.len(), 1, "{bad} should be rejected: {issues:?}");
        }
        let issues = validate("env:\n  MODEL:\n    default: x\n");
        assert_eq!(issues, vec![]);
    }

    #[test]
    fn rejects_malformed_env_names() {
        // whitespace padding must not slip the deny-list
        for bad in ["\" PATH \"", "\"PATH\\t\"", "\"A B\"", "\"1LEADING\""] {
            let issues = validate(&format!("env:\n  {bad}:\n    default: x\n"));
            assert_eq!(issues.len(), 1, "{bad} should be rejected: {issues:?}");
        }
    }

    #[test]
    fn unknown_type_gets_suggestion() {
        let issues = validate("inputs:\n  image:\n    type: std/media/v1/Imag\n");
        assert_eq!(issues.len(), 1);
        assert!(issues[0].message.contains("did you mean"), "{issues:?}");
    }

    #[test]
    fn custom_type_must_match_namespace() {
        let issues = validate(
            r#"
types:
  other-ns/lidar/v1/PointCloud:
    arrow: Struct
    fields:
      - name: x
        type: Float32
"#,
        );
        assert_eq!(issues.len(), 1);
        assert!(issues[0].message.contains("namespace"), "{issues:?}");

        let issues = validate(
            r#"
types:
  dora-rs/lidar/v1/PointCloud:
    arrow: Struct
    fields:
      - name: x
        type: Float32
outputs:
  cloud:
    type: dora-rs/lidar/v1/PointCloud
"#,
        );
        assert_eq!(issues, vec![]);
    }

    #[test]
    fn env_default_must_match_declared_type() {
        let issues = validate("env:\n  CONFIDENCE:\n    type: float\n    default: not-a-number\n");
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].field, "env.CONFIDENCE.default");

        // integer default satisfies a float declaration
        let issues = validate("env:\n  CONFIDENCE:\n    type: float\n    default: 1\n");
        assert_eq!(issues, vec![]);

        // any scalar is a valid `type: string` default — `true`/`8080` are
        // unquoted YAML scalars the publisher means as strings
        for default in ["true", "8080", "1.5", "\"hello\""] {
            let issues = validate(&format!(
                "env:\n  V:\n    type: string\n    default: {default}\n"
            ));
            assert_eq!(
                issues,
                vec![],
                "string default `{default}` should be accepted"
            );
        }
    }

    #[test]
    fn shipped_type_bodies_are_validated() {
        // a shipped type referencing an unresolvable field type is rejected
        let issues = validate(
            r#"
types:
  dora-rs/lidar/v1/Bad:
    arrow: Struct
    fields:
      - name: x
        type: DefinitelyNotAType
outputs:
  cloud:
    type: dora-rs/lidar/v1/Bad
"#,
        );
        assert!(
            issues
                .iter()
                .any(|i| i.field == "types.dora-rs/lidar/v1/Bad.fields.x"),
            "{issues:?}"
        );

        // fields referencing primitives, std types, and sibling shipped types
        // all resolve
        let issues = validate(
            r#"
types:
  dora-rs/lidar/v1/Point:
    arrow: Struct
    fields:
      - name: x
        type: Float32
  dora-rs/lidar/v1/Cloud:
    arrow: Struct
    fields:
      - name: first
        type: dora-rs/lidar/v1/Point
      - name: raw
        type: std/core/v1/Bytes
      - name: points
        type: List<dora-rs/lidar/v1/Point>
outputs:
  cloud:
    type: dora-rs/lidar/v1/Cloud
"#,
        );
        assert_eq!(issues, vec![]);
    }

    #[test]
    fn shipped_type_keys_are_format_checked() {
        // parameterized keys can never be referenced (the port checker strips
        // params before lookup) — reject them at declaration
        let issues = validate("types:\n  \"dora-rs/lidar/v1/PC[res=high]\":\n    arrow: Struct\n");
        assert_eq!(issues.len(), 1, "{issues:?}");
        assert!(issues[0].field.starts_with("types."), "{issues:?}");

        // parameterized *references* to a declared base type resolve fine
        let issues = validate(
            r#"
types:
  dora-rs/lidar/v1/PC:
    arrow: Struct
outputs:
  cloud:
    type: "dora-rs/lidar/v1/PC[res=high]"
"#,
        );
        assert_eq!(issues, vec![]);
    }

    #[test]
    fn std_shipped_type_rejected() {
        let issues = validate(
            r#"
types:
  std/lidar/v1/PointCloud:
    arrow: Struct
"#,
        );
        assert_eq!(issues.len(), 1);
        assert!(issues[0].message.contains("std/"), "{issues:?}");
    }

    #[test]
    fn own_namespace_type_without_declaration_hints_types_block() {
        let issues = validate("outputs:\n  cloud:\n    type: dora-rs/lidar/v1/PointCloud\n");
        assert_eq!(issues.len(), 1);
        assert!(issues[0].message.contains("types:"), "{issues:?}");
    }

    #[test]
    fn rejects_bad_platform_and_semver() {
        let issues = validate("platforms: [linux-x86_64, freebsd-x86_64]\ndora: \"not-a-req\"\n");
        assert_eq!(issues.len(), 2, "{issues:?}");
    }

    #[test]
    fn rejects_bad_port_names() {
        let issues = validate("outputs:\n  \"a/b\": {}\n");
        assert_eq!(issues.len(), 1);
        assert!(issues[0].message.contains("/"), "{issues:?}");
    }

    #[test]
    fn rejects_invalid_example_yaml() {
        let issues = validate("example: \"- id: [unclosed\"\n");
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].field, "example");
    }

    #[test]
    fn validate_strict_lists_all_problems() {
        let mut m = minimal("");
        m.api_version = 2;
        m.entrypoint = "../evil".into();
        let err = m.validate_strict(&TypeRegistry::new()).unwrap_err();
        let msg = format!("{err:#}");
        assert!(msg.contains("apiVersion"), "{msg}");
        assert!(msg.contains("entrypoint"), "{msg}");
    }
}
