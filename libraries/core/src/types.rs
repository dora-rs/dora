use arrow_schema::DataType;
use serde::Deserialize;
use std::collections::BTreeMap;

/// A single type definition from the standard type library.
#[derive(Debug, Clone, Deserialize)]
pub struct TypeDef {
    /// Arrow data type name (e.g. "Float32", "Struct", "LargeBinary")
    pub arrow: String,
    /// Human-readable description
    #[serde(default)]
    pub description: Option<String>,
}

impl TypeDef {
    /// Convert the arrow type name to an `arrow_schema::DataType`.
    ///
    /// Returns `None` for complex types (Struct, etc.) that cannot be validated
    /// from the type name alone.
    pub fn arrow_data_type(&self) -> Option<DataType> {
        match self.arrow.as_str() {
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
}

/// YAML file format for a type package.
#[derive(Debug, Deserialize)]
struct TypePackage {
    types: BTreeMap<String, TypeDef>,
}

/// Extract the short type name from a URN (e.g. `std/media/v1/Image` -> `Image`).
pub fn urn_short_name(urn: &str) -> &str {
    urn.rsplit('/').next().unwrap_or(urn)
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

    /// Resolve a type URN to its definition. Returns `None` if unknown.
    pub fn resolve(&self, urn: &str) -> Option<&TypeDef> {
        self.types.get(urn)
    }

    /// Resolve a type URN to its Arrow DataType. Returns `None` if unknown or complex.
    pub fn resolve_arrow_type(&self, urn: &str) -> Option<DataType> {
        self.resolve(urn).and_then(|def| def.arrow_data_type())
    }

    /// Return all known URNs (sorted).
    pub fn all_urns(&self) -> impl Iterator<Item = &str> {
        self.types.keys().map(String::as_str)
    }

    /// Suggest the closest URN for a typo. Returns `None` if no close match.
    /// Prefers matches in the same package prefix (e.g. `std/media/v1`).
    pub fn suggest(&self, urn: &str) -> Option<&str> {
        let target_name = urn_short_name(urn).to_lowercase();
        let target_prefix = urn.rsplit_once('/').map(|(p, _)| p);
        self.types
            .keys()
            .filter_map(|k| {
                let name = urn_short_name(k).to_lowercase();
                let dist = edit_distance(&target_name, &name);
                (dist <= 2).then(|| {
                    // Penalize cross-package matches so same-package wins on ties
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

impl Default for TypeRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Simple edit distance (Levenshtein) for typo suggestions.
fn edit_distance(a: &str, b: &str) -> usize {
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
        // Struct types cannot be validated from name alone
        assert_eq!(reg.resolve_arrow_type("std/media/v1/Image"), None);
        assert_eq!(reg.resolve_arrow_type("std/math/v1/Vector3"), None);
    }

    #[test]
    fn arrow_data_type_unknown_urn_returns_none() {
        let reg = TypeRegistry::new();
        assert_eq!(reg.resolve_arrow_type("std/core/v1/NonExistent"), None);
    }
}
