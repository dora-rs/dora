use arrow_schema::{DataType, Field, Schema};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::Path;

/// A field definition within a struct type.
#[derive(Debug, Clone, Deserialize)]
pub struct FieldDef {
    /// Field name
    pub name: String,
    /// Arrow type name or URN reference
    pub r#type: String,
    /// Whether the field is nullable (default: true)
    #[serde(default = "default_true")]
    pub nullable: bool,
}

fn default_true() -> bool {
    true
}

/// A type parameter declaration (e.g. `sample_type` on AudioFrame).
#[derive(Debug, Clone, Deserialize)]
pub struct TypeParam {
    /// Parameter name
    pub name: String,
    /// Default value (used when parameter is not specified)
    #[serde(default)]
    pub default: Option<String>,
}

/// Metadata key required on outputs of a given type.
#[derive(Debug, Clone, Deserialize)]
pub struct MetadataDef {
    /// Key name that must be present in message metadata
    pub key: String,
    /// Description of the metadata key
    #[serde(default)]
    pub description: Option<String>,
}

/// A single type definition from the standard type library.
#[derive(Debug, Clone, Deserialize)]
pub struct TypeDef {
    /// Arrow data type name (e.g. "Float32", "Struct", "LargeBinary")
    pub arrow: String,
    /// Human-readable description
    #[serde(default)]
    pub description: Option<String>,
    /// Type parameters (e.g. sample_type for AudioFrame)
    #[serde(default)]
    pub params: Vec<TypeParam>,
    /// Struct field definitions (only for Struct arrow types)
    #[serde(default)]
    pub fields: Vec<FieldDef>,
    /// Required metadata keys
    #[serde(default)]
    pub metadata: Vec<MetadataDef>,
}

impl TypeDef {
    /// Convert the arrow type name to an `arrow_schema::DataType`.
    ///
    /// Returns `None` for complex types (Struct, etc.) that cannot be validated
    /// from the type name alone.
    pub fn arrow_data_type(&self) -> Option<DataType> {
        arrow_type_from_name(&self.arrow)
    }

    /// Build an Arrow `Schema` from the field definitions.
    ///
    /// Returns `None` if the type has no field definitions.
    pub fn to_arrow_schema(&self) -> Option<Schema> {
        if self.fields.is_empty() {
            return None;
        }
        let fields: Vec<Field> = self
            .fields
            .iter()
            .filter_map(|f| {
                let dt = arrow_type_from_name(&f.r#type)?;
                Some(Field::new(&f.name, dt, f.nullable))
            })
            .collect();
        if fields.is_empty() {
            return None;
        }
        Some(Schema::new(fields))
    }
}

/// Convert an arrow type name string to a `DataType`.
fn arrow_type_from_name(name: &str) -> Option<DataType> {
    match name {
        "Float32" => Some(DataType::Float32),
        "Float64" => Some(DataType::Float64),
        "Int32" => Some(DataType::Int32),
        "Int64" => Some(DataType::Int64),
        "UInt8" => Some(DataType::UInt8),
        "UInt32" => Some(DataType::UInt32),
        "UInt64" => Some(DataType::UInt64),
        "Utf8" => Some(DataType::Utf8),
        "LargeBinary" => Some(DataType::LargeBinary),
        "Boolean" => Some(DataType::Boolean),
        _ => None, // Struct, FixedSizeBinary, etc. — skip runtime validation
    }
}

/// Parsed URN with optional type parameters.
///
/// Example: `std/media/v1/AudioFrame[sample_type=f32,channels=2]`
/// -> base = `std/media/v1/AudioFrame`, params = `{sample_type: f32, channels: 2}`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParsedUrn {
    /// Base URN without parameters
    pub base: String,
    /// Parameter key-value pairs
    pub params: BTreeMap<String, String>,
}

/// Parse a URN string into base + optional parameters.
///
/// Accepted formats:
/// - `std/core/v1/Float32` (no params)
/// - `std/media/v1/AudioFrame[sample_type=f32]` (with params)
/// - `std/media/v1/AudioFrame[sample_type=f32,channels=2]` (multiple params)
pub fn parse_urn(urn: &str) -> Option<ParsedUrn> {
    if urn.is_empty() {
        return None;
    }
    let Some(bracket_start) = urn.find('[') else {
        return Some(ParsedUrn {
            base: urn.to_string(),
            params: BTreeMap::new(),
        });
    };
    if !urn.ends_with(']') {
        return None; // malformed: has `[` but no closing `]`
    }
    let base = urn[..bracket_start].to_string();
    let params_str = &urn[bracket_start + 1..urn.len() - 1];
    if params_str.is_empty() {
        return None; // malformed: empty brackets
    }
    let mut params = BTreeMap::new();
    for part in params_str.split(',') {
        let part = part.trim();
        let (k, v) = part.split_once('=')?;
        let k = k.trim();
        let v = v.trim();
        if k.is_empty() || v.is_empty() {
            return None;
        }
        params.insert(k.to_string(), v.to_string());
    }
    Some(ParsedUrn { base, params })
}

/// Check if two type URNs are compatible (considering parameters).
///
/// Rules:
/// - Same base + same params -> compatible
/// - Same base + one side unparameterized -> compatible (wildcard)
/// - Same base + different param values -> mismatch
/// - Different base -> mismatch
pub fn types_match(a: &str, b: &str) -> bool {
    let Some(pa) = parse_urn(a) else {
        return a == b;
    };
    let Some(pb) = parse_urn(b) else {
        return a == b;
    };
    if pa.base != pb.base {
        return false;
    }
    // If either side has no params, treat as wildcard
    if pa.params.is_empty() || pb.params.is_empty() {
        return true;
    }
    // Both have params — all shared keys must agree
    for (k, va) in &pa.params {
        if let Some(vb) = pb.params.get(k) {
            if va != vb {
                return false;
            }
        }
    }
    true
}

/// YAML file format for a type package.
#[derive(Debug, Deserialize)]
struct TypePackage {
    types: BTreeMap<String, TypeDef>,
}

/// Extract the short type name from a URN (e.g. `std/media/v1/Image` -> `Image`).
pub fn urn_short_name(urn: &str) -> &str {
    // Strip params first
    let base = urn.split('[').next().unwrap_or(urn);
    base.rsplit('/').next().unwrap_or(base)
}

// --- Compatibility graph (Phase 4) ---

/// A user-defined type compatibility rule.
#[derive(Debug, Clone, Deserialize)]
pub struct TypeRule {
    /// Source type URN
    pub from: String,
    /// Target type URN
    pub to: String,
}

/// Graph of type compatibility rules for checking implicit conversions.
pub struct CompatibilityGraph {
    /// Adjacency list: from-URN -> set of to-URNs
    edges: BTreeMap<String, Vec<String>>,
}

impl CompatibilityGraph {
    /// Create a new compatibility graph with hardcoded rules + optional user rules.
    pub fn new(user_rules: &[TypeRule]) -> Self {
        let mut edges: BTreeMap<String, Vec<String>> = BTreeMap::new();

        // Hardcoded widening rules
        let builtins = [
            ("std/core/v1/UInt8", "std/core/v1/UInt32"),
            ("std/core/v1/UInt32", "std/core/v1/UInt64"),
            ("std/core/v1/Int32", "std/core/v1/Int64"),
            ("std/core/v1/Float32", "std/core/v1/Float64"),
        ];
        for (from, to) in builtins {
            edges
                .entry(from.to_string())
                .or_default()
                .push(to.to_string());
        }

        // User-defined rules
        for rule in user_rules {
            edges
                .entry(rule.from.clone())
                .or_default()
                .push(rule.to.clone());
        }

        Self { edges }
    }

    /// Check if `from` can be implicitly converted to `to`.
    ///
    /// Uses BFS with a depth limit of 3 to prevent surprise transitive chains.
    /// Also handles the universal `* -> Bytes` sink.
    pub fn is_compatible(&self, from: &str, to: &str) -> bool {
        // Universal sink: anything -> Bytes
        if to == "std/core/v1/Bytes" {
            return true;
        }

        // Parse URNs once and reuse for both matching and BFS
        let from_parsed = parse_urn(from);
        let to_parsed = parse_urn(to);
        let from_base = from_parsed
            .as_ref()
            .map(|p| p.base.as_str())
            .unwrap_or(from);
        let to_base = to_parsed.as_ref().map(|p| p.base.as_str()).unwrap_or(to);

        // Check parameterized match using already-parsed URNs
        if from_base == to_base {
            // Same base — check params. If either has no params, wildcard match.
            let from_params = from_parsed.as_ref().map(|p| &p.params);
            let to_params = to_parsed.as_ref().map(|p| &p.params);
            match (from_params, to_params) {
                (Some(fp), Some(tp)) if !fp.is_empty() && !tp.is_empty() => {
                    // Both have params — all shared keys must agree
                    for (k, va) in fp {
                        if let Some(vb) = tp.get(k) {
                            if va != vb {
                                return false;
                            }
                        }
                    }
                }
                _ => {}
            }
            return true;
        }

        // BFS with depth limit 3
        let mut queue = std::collections::VecDeque::new();
        let mut visited = std::collections::BTreeSet::new();
        queue.push_back((from_base.to_string(), 0u32));
        visited.insert(from_base.to_string());

        while let Some((current, depth)) = queue.pop_front() {
            if depth >= 3 {
                continue;
            }
            if let Some(neighbors) = self.edges.get(&current) {
                for next in neighbors {
                    if next == to_base {
                        return true;
                    }
                    if visited.insert(next.clone()) {
                        queue.push_back((next.clone(), depth + 1));
                    }
                }
            }
        }

        false
    }
}

// --- Arrow schema compatibility (Phase 6) ---

/// Check if `actual` schema is compatible with `expected` schema.
///
/// Rules:
/// - Actual superset of expected -> OK (additive fields fine)
/// - Missing expected field -> incompatible
/// - Wrong field type -> incompatible
/// - Field order is irrelevant (lookup by name)
pub fn schema_compatible(expected: &Schema, actual: &Schema) -> Result<(), SchemaError> {
    for expected_field in expected.fields() {
        match actual.field_with_name(expected_field.name()) {
            Ok(actual_field) => {
                if actual_field.data_type() != expected_field.data_type() {
                    return Err(SchemaError::TypeMismatch {
                        field: expected_field.name().to_string(),
                        expected: format!("{:?}", expected_field.data_type()),
                        actual: format!("{:?}", actual_field.data_type()),
                    });
                }
            }
            Err(_) => {
                return Err(SchemaError::MissingField {
                    field: expected_field.name().to_string(),
                });
            }
        }
    }
    Ok(())
}

/// Schema compatibility error.
#[derive(Debug)]
pub enum SchemaError {
    /// A required field is missing
    MissingField { field: String },
    /// A field has the wrong type
    TypeMismatch {
        field: String,
        expected: String,
        actual: String,
    },
}

impl std::fmt::Display for SchemaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SchemaError::MissingField { field } => write!(f, "missing field \"{field}\""),
            SchemaError::TypeMismatch {
                field,
                expected,
                actual,
            } => write!(
                f,
                "field \"{field}\" type mismatch: expected {expected}, got {actual}"
            ),
        }
    }
}

// --- Metadata pattern resolution (Phase 5) ---

/// Resolve a pattern shorthand to required metadata keys.
pub fn pattern_metadata_keys(pattern: &str) -> Option<&'static [&'static str]> {
    match pattern {
        "service-server" | "service-client" => Some(&["request_id"]),
        "action-server" => Some(&["goal_id", "goal_status"]),
        "action-client" => Some(&["goal_id"]),
        _ => None,
    }
}

/// Registry of known type URNs, loaded from embedded YAML files.
///
/// URN format: `std/<category>/v<version>/<TypeName>`
pub struct TypeRegistry {
    types: BTreeMap<String, TypeDef>,
}

const PACKAGES: &[(&str, &str)] = &[
    (
        "std/core/v1",
        include_str!("../../../types/std/core/v1.yml"),
    ),
    (
        "std/math/v1",
        include_str!("../../../types/std/math/v1.yml"),
    ),
    (
        "std/control/v1",
        include_str!("../../../types/std/control/v1.yml"),
    ),
    (
        "std/media/v1",
        include_str!("../../../types/std/media/v1.yml"),
    ),
    (
        "std/vision/v1",
        include_str!("../../../types/std/vision/v1.yml"),
    ),
];

impl TypeRegistry {
    /// Create a new registry with all built-in standard types loaded.
    pub fn new() -> Self {
        let mut types = BTreeMap::new();
        for (prefix, yaml) in PACKAGES {
            let pkg: TypePackage =
                serde_yaml::from_str(yaml).expect("built-in type YAML should be valid");
            for (name, def) in pkg.types {
                let urn = format!("{prefix}/{name}");
                types.insert(urn, def);
            }
        }
        Self { types }
    }

    /// Resolve a type URN to its definition.
    ///
    /// Handles parameterized URNs by stripping parameters before lookup.
    pub fn resolve(&self, urn: &str) -> Option<&TypeDef> {
        // Try exact match first
        if let Some(def) = self.types.get(urn) {
            return Some(def);
        }
        // Try stripping parameters
        if let Some(parsed) = parse_urn(urn) {
            if parsed.base != urn {
                return self.types.get(&parsed.base);
            }
        }
        None
    }

    /// Resolve a type URN to its Arrow DataType. Returns `None` if unknown or complex.
    pub fn resolve_arrow_type(&self, urn: &str) -> Option<DataType> {
        self.resolve(urn).and_then(|def| def.arrow_data_type())
    }

    /// Return all known URNs (sorted).
    pub fn all_urns(&self) -> impl Iterator<Item = &str> {
        self.types.keys().map(String::as_str)
    }

    /// Load user-defined types from a directory tree.
    ///
    /// Convention: `types/<prefix>/<category>/v<N>.yml`
    /// URN prefix derived from directory structure.
    /// The `std/` prefix is rejected.
    pub fn load_from_dir(&mut self, dir: &Path) -> Result<usize, String> {
        if !dir.is_dir() {
            return Ok(0);
        }
        let mut count = 0;
        for path in walkdir_iter(dir) {
            if path.extension().and_then(|e| e.to_str()) != Some("yml") {
                continue;
            }
            let rel = path.strip_prefix(dir).map_err(|e| e.to_string())?;
            let prefix = rel
                .parent()
                .map(|p| p.to_string_lossy().replace('\\', "/"))
                .unwrap_or_default();
            if prefix.starts_with("std/") || prefix == "std" {
                return Err(format!(
                    "user types cannot use the \"std/\" prefix: {}",
                    path.display()
                ));
            }
            let stem = rel
                .file_stem()
                .and_then(|s| s.to_str())
                .ok_or_else(|| format!("invalid file name: {}", path.display()))?;
            let urn_prefix = if prefix.is_empty() {
                stem.to_string()
            } else {
                format!("{prefix}/{stem}")
            };
            let content =
                std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            let pkg: TypePackage =
                serde_yaml::from_str(&content).map_err(|e| format!("{}: {e}", path.display()))?;
            for (name, def) in pkg.types {
                let urn = format!("{urn_prefix}/{name}");
                self.types.insert(urn, def);
                count += 1;
            }
        }
        Ok(count)
    }

    /// Suggest the closest URN for a typo. Returns `None` if no close match.
    /// Prefers matches in the same package prefix (e.g. `std/media/v1`).
    pub fn suggest(&self, urn: &str) -> Option<&str> {
        let target_name = urn_short_name(urn).to_lowercase();
        let target_prefix = urn.split('[').next().unwrap_or(urn);
        let target_prefix = target_prefix.rsplit_once('/').map(|(p, _)| p);
        self.types
            .keys()
            .filter_map(|k| {
                let name = urn_short_name(k).to_lowercase();
                let dist = edit_distance(&target_name, &name);
                (dist <= 2).then(|| {
                    let prefix_penalty = if target_prefix == k.rsplit_once('/').map(|(p, _)| p) {
                        0
                    } else {
                        100
                    };
                    (k.as_str(), dist + prefix_penalty)
                })
            })
            .min_by_key(|&(_, score)| score)
            .map(|(k, _)| k)
    }
}

/// Walk a directory tree yielding file paths lazily.
///
/// Depth-limited to 8 levels. Uses `file_type()` to avoid following symlinks.
fn walkdir_iter(dir: &Path) -> Vec<std::path::PathBuf> {
    const MAX_DEPTH: u32 = 8;
    let mut result = Vec::new();
    let mut stack: Vec<(std::path::PathBuf, u32)> = vec![(dir.to_path_buf(), 0)];
    while let Some((path, depth)) = stack.pop() {
        if depth >= MAX_DEPTH {
            continue;
        }
        let Ok(entries) = std::fs::read_dir(&path) else {
            continue;
        };
        for entry in entries.flatten() {
            let is_dir = entry.file_type().map(|t| t.is_dir()).unwrap_or(false);
            if is_dir {
                stack.push((entry.path(), depth + 1));
            } else {
                result.push(entry.path());
            }
        }
    }
    result
}

impl Default for TypeRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Simple edit distance (Levenshtein) for typo suggestions.
/// Returns `usize::MAX` for inputs longer than 256 characters to prevent DoS.
fn edit_distance(a: &str, b: &str) -> usize {
    if a.len() > 256 || b.len() > 256 {
        return usize::MAX;
    }
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    let mut dp = vec![vec![0usize; b.len() + 1]; a.len() + 1];
    for (i, row) in dp.iter_mut().enumerate() {
        row[0] = i;
    }
    for (j, val) in dp[0].iter_mut().enumerate() {
        *val = j;
    }
    for i in 1..=a.len() {
        for j in 1..=b.len() {
            let cost = if a[i - 1] == b[j - 1] { 0 } else { 1 };
            dp[i][j] = (dp[i - 1][j] + 1)
                .min(dp[i][j - 1] + 1)
                .min(dp[i - 1][j - 1] + cost);
        }
    }
    dp[a.len()][b.len()]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_resolves_known_urns() {
        let reg = TypeRegistry::new();
        assert!(reg.resolve("std/core/v1/Float32").is_some());
        assert!(reg.resolve("std/media/v1/Image").is_some());
        assert!(reg.resolve("std/vision/v1/BoundingBox").is_some());
    }

    #[test]
    fn registry_returns_none_for_unknown() {
        let reg = TypeRegistry::new();
        assert!(reg.resolve("std/core/v1/NonExistent").is_none());
        assert!(reg.resolve("custom/MyType").is_none());
    }

    #[test]
    fn all_embedded_yaml_loads() {
        // TypeRegistry::new() panics if any YAML is invalid
        let reg = TypeRegistry::new();
        assert!(reg.types.len() > 10);
    }

    #[test]
    fn suggest_finds_typo() {
        let reg = TypeRegistry::new();
        let suggestion = reg.suggest("std/media/v1/Imag");
        assert_eq!(suggestion, Some("std/media/v1/Image"));
    }

    #[test]
    fn suggest_returns_none_for_unrelated() {
        let reg = TypeRegistry::new();
        assert!(reg.suggest("std/core/v1/Xyzzy").is_none());
    }

    #[test]
    fn type_def_has_arrow_field() {
        let reg = TypeRegistry::new();
        let def = reg.resolve("std/core/v1/Float32").unwrap();
        assert_eq!(def.arrow, "Float32");
    }

    #[test]
    fn arrow_data_type_primitives() {
        let reg = TypeRegistry::new();
        assert_eq!(
            reg.resolve_arrow_type("std/core/v1/Float32"),
            Some(DataType::Float32)
        );
        assert_eq!(
            reg.resolve_arrow_type("std/core/v1/UInt64"),
            Some(DataType::UInt64)
        );
        assert_eq!(
            reg.resolve_arrow_type("std/core/v1/String"),
            Some(DataType::Utf8)
        );
        assert_eq!(
            reg.resolve_arrow_type("std/core/v1/Bool"),
            Some(DataType::Boolean)
        );
        assert_eq!(
            reg.resolve_arrow_type("std/core/v1/Bytes"),
            Some(DataType::LargeBinary)
        );
    }

    #[test]
    fn arrow_data_type_complex_returns_none() {
        let reg = TypeRegistry::new();
        assert_eq!(reg.resolve_arrow_type("std/media/v1/Image"), None);
        assert_eq!(reg.resolve_arrow_type("std/math/v1/Vector3"), None);
    }

    #[test]
    fn arrow_data_type_unknown_urn_returns_none() {
        let reg = TypeRegistry::new();
        assert_eq!(reg.resolve_arrow_type("std/core/v1/NonExistent"), None);
    }

    // --- Phase 2: parse_urn tests ---

    #[test]
    fn parse_urn_no_params() {
        let parsed = parse_urn("std/media/v1/Image").unwrap();
        assert_eq!(parsed.base, "std/media/v1/Image");
        assert!(parsed.params.is_empty());
    }

    #[test]
    fn parse_urn_single_param() {
        let parsed = parse_urn("std/media/v1/AudioFrame[sample_type=f32]").unwrap();
        assert_eq!(parsed.base, "std/media/v1/AudioFrame");
        assert_eq!(parsed.params.len(), 1);
        assert_eq!(parsed.params["sample_type"], "f32");
    }

    #[test]
    fn parse_urn_multiple_params() {
        let parsed = parse_urn("std/media/v1/AudioFrame[sample_type=f32,channels=2]").unwrap();
        assert_eq!(parsed.base, "std/media/v1/AudioFrame");
        assert_eq!(parsed.params.len(), 2);
        assert_eq!(parsed.params["sample_type"], "f32");
        assert_eq!(parsed.params["channels"], "2");
    }

    #[test]
    fn parse_urn_malformed() {
        assert!(parse_urn("").is_none());
        assert!(parse_urn("foo[").is_none()); // no closing bracket
        assert!(parse_urn("foo[]").is_none()); // empty brackets
        assert!(parse_urn("foo[=val]").is_none()); // empty key
        assert!(parse_urn("foo[key=]").is_none()); // empty value
    }

    #[test]
    fn types_match_exact() {
        assert!(types_match("std/core/v1/Float32", "std/core/v1/Float32"));
    }

    #[test]
    fn types_match_wildcard() {
        assert!(types_match(
            "std/media/v1/AudioFrame",
            "std/media/v1/AudioFrame[sample_type=f32]"
        ));
        assert!(types_match(
            "std/media/v1/AudioFrame[sample_type=f32]",
            "std/media/v1/AudioFrame"
        ));
    }

    #[test]
    fn types_match_param_mismatch() {
        assert!(!types_match(
            "std/media/v1/AudioFrame[sample_type=f32]",
            "std/media/v1/AudioFrame[sample_type=i16]"
        ));
    }

    #[test]
    fn types_match_different_base() {
        assert!(!types_match("std/core/v1/Float32", "std/core/v1/Float64"));
    }

    // --- Fallback path: parse_urn returns None ---
    //
    // The two `let Some(...) else { return a == b; }` branches in
    // types_match() were uncovered (caught by mutation testing: mutants at
    // lines 168 and 171 escaped the existing suite). These tests exercise
    // empty-string and malformed-URN inputs so that the fallback equality
    // check is both reachable and semantically load-bearing.

    #[test]
    fn types_match_first_unparseable_falls_back_to_equality() {
        // `a` is empty -> parse_urn(a) is None -> falls back to a == b.
        // These assertions fail for both the unmutated code and mutations
        // that would change `==` to `!=` only if the result differs.
        assert!(!types_match("", "std/core/v1/Float32"));
        assert!(!types_match("std/core/v1/Float32[", "std/core/v1/Float32"));
    }

    #[test]
    fn types_match_second_unparseable_falls_back_to_equality() {
        // `a` parses, `b` does not -> falls back to a == b (= false since
        // a is a valid URN and b is empty/malformed).
        assert!(!types_match("std/core/v1/Float32", ""));
        assert!(!types_match("std/core/v1/Float32", "std/core/v1/Float32["));
    }

    #[test]
    fn types_match_both_unparseable_equal_strings() {
        // Both fail to parse but are byte-identical: fallback says true.
        assert!(types_match("", ""));
        assert!(types_match("malformed[", "malformed["));
    }

    #[test]
    fn types_match_both_unparseable_different_strings() {
        // Both fail to parse and differ: fallback says false.
        assert!(!types_match("malformed[", "other["));
        assert!(!types_match("", "[empty-base]"));
    }

    // --- Phase 4: compatibility graph tests ---

    #[test]
    fn compat_uint8_to_uint32() {
        let g = CompatibilityGraph::new(&[]);
        assert!(g.is_compatible("std/core/v1/UInt8", "std/core/v1/UInt32"));
    }

    #[test]
    fn compat_uint8_to_uint64_transitive() {
        let g = CompatibilityGraph::new(&[]);
        assert!(g.is_compatible("std/core/v1/UInt8", "std/core/v1/UInt64"));
    }

    #[test]
    fn compat_uint8_to_int32_mismatch() {
        let g = CompatibilityGraph::new(&[]);
        assert!(!g.is_compatible("std/core/v1/UInt8", "std/core/v1/Int32"));
    }

    #[test]
    fn compat_any_to_bytes() {
        let g = CompatibilityGraph::new(&[]);
        assert!(g.is_compatible("std/media/v1/Image", "std/core/v1/Bytes"));
        assert!(g.is_compatible("std/core/v1/Float32", "std/core/v1/Bytes"));
    }

    #[test]
    fn compat_depth_limit() {
        // Create a chain of 4: A->B->C->D->E
        let rules = vec![
            TypeRule {
                from: "a/A".into(),
                to: "a/B".into(),
            },
            TypeRule {
                from: "a/B".into(),
                to: "a/C".into(),
            },
            TypeRule {
                from: "a/C".into(),
                to: "a/D".into(),
            },
            TypeRule {
                from: "a/D".into(),
                to: "a/E".into(),
            },
        ];
        let g = CompatibilityGraph::new(&rules);
        assert!(g.is_compatible("a/A", "a/C")); // depth 2
        assert!(g.is_compatible("a/A", "a/D")); // depth 3
        assert!(!g.is_compatible("a/A", "a/E")); // depth 4 — exceeds limit
    }

    #[test]
    fn compat_user_defined_rule() {
        let rules = vec![TypeRule {
            from: "myproject/SensorV1".into(),
            to: "myproject/SensorV2".into(),
        }];
        let g = CompatibilityGraph::new(&rules);
        assert!(g.is_compatible("myproject/SensorV1", "myproject/SensorV2"));
    }

    // --- Phase 5: metadata pattern tests ---

    #[test]
    fn pattern_service_server() {
        let keys = pattern_metadata_keys("service-server").unwrap();
        assert!(keys.contains(&"request_id"));
    }

    #[test]
    fn pattern_action_server() {
        let keys = pattern_metadata_keys("action-server").unwrap();
        assert!(keys.contains(&"goal_id"));
        assert!(keys.contains(&"goal_status"));
    }

    #[test]
    fn pattern_unknown() {
        assert!(pattern_metadata_keys("unknown").is_none());
    }

    // --- Phase 6: schema compatibility tests ---

    #[test]
    fn schema_superset_compatible() {
        let expected = Schema::new(vec![Field::new("x", DataType::Float64, true)]);
        let actual = Schema::new(vec![
            Field::new("x", DataType::Float64, true),
            Field::new("y", DataType::Float64, true),
        ]);
        assert!(schema_compatible(&expected, &actual).is_ok());
    }

    #[test]
    fn schema_missing_field() {
        let expected = Schema::new(vec![
            Field::new("x", DataType::Float64, true),
            Field::new("y", DataType::Float64, true),
        ]);
        let actual = Schema::new(vec![Field::new("x", DataType::Float64, true)]);
        let err = schema_compatible(&expected, &actual).unwrap_err();
        assert!(matches!(err, SchemaError::MissingField { .. }));
    }

    #[test]
    fn schema_wrong_type() {
        let expected = Schema::new(vec![Field::new("x", DataType::Float64, true)]);
        let actual = Schema::new(vec![Field::new("x", DataType::Int32, true)]);
        let err = schema_compatible(&expected, &actual).unwrap_err();
        assert!(matches!(err, SchemaError::TypeMismatch { .. }));
    }

    // --- Phase 7: user-defined types tests ---

    #[test]
    fn load_user_types_from_dir() {
        let dir = tempfile::tempdir().unwrap();
        let pkg_dir = dir.path().join("myproject").join("sensors");
        std::fs::create_dir_all(&pkg_dir).unwrap();
        std::fs::write(
            pkg_dir.join("v1.yml"),
            "types:\n  MySensor:\n    arrow: Struct\n    description: test\n",
        )
        .unwrap();
        let mut reg = TypeRegistry::new();
        let count = reg.load_from_dir(dir.path()).unwrap();
        assert_eq!(count, 1);
        assert!(reg.resolve("myproject/sensors/v1/MySensor").is_some());
    }

    #[test]
    fn load_user_types_std_prefix_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let pkg_dir = dir.path().join("std").join("foo");
        std::fs::create_dir_all(&pkg_dir).unwrap();
        std::fs::write(
            pkg_dir.join("v1.yml"),
            "types:\n  Foo:\n    arrow: Struct\n",
        )
        .unwrap();
        let mut reg = TypeRegistry::new();
        let err = reg.load_from_dir(dir.path());
        assert!(err.is_err());
        assert!(err.unwrap_err().contains("std/"));
    }

    // --- Phase 2: resolve parameterized URN ---

    #[test]
    fn resolve_parameterized_urn() {
        let reg = TypeRegistry::new();
        // AudioFrame exists as std/media/v1/AudioFrame
        let def = reg.resolve("std/media/v1/AudioFrame[sample_type=f32]");
        assert!(def.is_some());
        assert_eq!(def.unwrap().arrow, "Struct");
    }
}
