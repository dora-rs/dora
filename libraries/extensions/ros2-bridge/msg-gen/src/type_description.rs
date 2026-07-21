use std::{env, fs, path::PathBuf};

use serde::Deserialize;
use thiserror::Error;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InterfaceKind {
    Message,
    Service,
    Action,
}

impl InterfaceKind {
    const fn directory(self) -> &'static str {
        match self {
            Self::Message => "msg",
            Self::Service => "srv",
            Self::Action => "action",
        }
    }
}

#[derive(Debug)]
pub struct ResolvedTypeDescription {
    pub type_name: String,
    pub hash: [u8; 32],
    pub referenced_type_descriptions: Vec<serde_json::Value>,
}

#[derive(Debug, Error)]
pub enum TypeDescriptionError {
    #[error("type description for {type_name} was not found")]
    DescriptionNotFound { type_name: String },
    #[error("failed to read type description {path}: {source}")]
    Read {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("malformed type description {path}: {source}")]
    Malformed {
        path: PathBuf,
        source: serde_json::Error,
    },
    #[error("type description declared {actual}, expected {expected}")]
    TypeNameMismatch { expected: String, actual: String },
    #[error("official RIHS01 hash is absent or malformed for {type_name}")]
    InvalidHash { type_name: String },
}

#[derive(Debug, Default)]
pub struct TypeDescriptionResolver {
    prefixes: Vec<PathBuf>,
}

impl TypeDescriptionResolver {
    pub fn from_ament_prefix_path() -> Self {
        Self::from_prefixes(env::split_paths(
            &env::var_os("AMENT_PREFIX_PATH").unwrap_or_default(),
        ))
    }

    pub fn from_prefixes(prefixes: impl IntoIterator<Item = PathBuf>) -> Self {
        Self {
            prefixes: prefixes.into_iter().collect(),
        }
    }

    pub fn resolve(
        &self,
        package: &str,
        kind: InterfaceKind,
        name: &str,
    ) -> Result<ResolvedTypeDescription, TypeDescriptionError> {
        let expected = format!("{package}/{}/{name}", kind.directory());
        let mut artifact_names = vec![name];
        if kind == InterfaceKind::Action {
            artifact_names.extend(name.match_indices('_').map(|(index, _)| &name[..index]));
        }
        let candidates = self.prefixes.iter().flat_map(|prefix| {
            artifact_names.iter().flat_map(move |artifact_name| {
                [
                    prefix
                        .join("share")
                        .join(package)
                        .join(kind.directory())
                        .join(format!("{artifact_name}.json")),
                    prefix.join(format!(
                        "{package}__{}__{artifact_name}.json",
                        kind.directory()
                    )),
                ]
            })
        });
        let path = candidates
            .into_iter()
            .find(|path| path.is_file())
            .or_else(|| {
                self.prefixes.iter().find_map(|prefix| {
                    fs::read_dir(prefix)
                        .ok()?
                        .flatten()
                        .map(|entry| entry.path())
                        .find(|path| {
                            path.extension().is_some_and(|ext| ext == "json")
                                && fs::read(path)
                                    .ok()
                                    .and_then(|bytes| {
                                        serde_json::from_slice::<Artifact>(&bytes).ok()
                                    })
                                    .is_some_and(|artifact| {
                                        artifact
                                            .type_hashes
                                            .iter()
                                            .any(|entry| entry.type_name == expected)
                                    })
                        })
                })
            })
            .ok_or_else(|| TypeDescriptionError::DescriptionNotFound {
                type_name: expected.clone(),
            })?;
        let bytes = fs::read(&path).map_err(|source| TypeDescriptionError::Read {
            path: path.clone(),
            source,
        })?;
        let artifact: Artifact =
            serde_json::from_slice(&bytes).map_err(|source| TypeDescriptionError::Malformed {
                path: path.clone(),
                source,
            })?;
        let actual = artifact.type_description_msg.type_description.type_name;
        if actual != expected
            && !artifact
                .type_hashes
                .iter()
                .any(|entry| entry.type_name == expected)
        {
            return Err(TypeDescriptionError::TypeNameMismatch { expected, actual });
        }
        let hash_string = artifact
            .type_hashes
            .iter()
            .find(|entry| entry.type_name == expected)
            .map(|entry| entry.hash_string.as_str())
            .ok_or_else(|| TypeDescriptionError::InvalidHash {
                type_name: expected.clone(),
            })?;
        let hash = parse_hash(hash_string).ok_or_else(|| TypeDescriptionError::InvalidHash {
            type_name: expected.clone(),
        })?;
        Ok(ResolvedTypeDescription {
            type_name: expected,
            hash,
            referenced_type_descriptions: artifact
                .type_description_msg
                .referenced_type_descriptions,
        })
    }
}

fn parse_hash(value: &str) -> Option<[u8; 32]> {
    let hex = value.strip_prefix("RIHS01_")?;
    if hex.len() != 64 {
        return None;
    }
    let mut result = [0; 32];
    for (index, byte) in result.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&hex[index * 2..index * 2 + 2], 16).ok()?;
    }
    Some(result)
}

#[derive(Deserialize)]
struct Artifact {
    type_description_msg: Description,
    type_hashes: Vec<HashEntry>,
}
#[derive(Deserialize)]
struct Description {
    type_description: IndividualDescription,
    referenced_type_descriptions: Vec<serde_json::Value>,
}
#[derive(Deserialize)]
struct IndividualDescription {
    type_name: String,
}
#[derive(Deserialize)]
struct HashEntry {
    type_name: String,
    hash_string: String,
}

#[cfg(test)]
mod tests {
    use super::{InterfaceKind, TypeDescriptionError, TypeDescriptionResolver};
    use std::{
        fs,
        time::{SystemTime, UNIX_EPOCH},
    };

    fn fixtures() -> TypeDescriptionResolver {
        TypeDescriptionResolver::from_prefixes([std::path::PathBuf::from(
            env!("CARGO_MANIFEST_DIR").to_owned() + "/test_type_descriptions",
        )])
    }

    #[test]
    fn type_description_resolves_official_string_hash() {
        let description = fixtures()
            .resolve("std_msgs", InterfaceKind::Message, "String")
            .unwrap();
        assert_eq!(description.type_name, "std_msgs/msg/String");
        assert_eq!(
            description.hash,
            [
                0xdf, 0x66, 0x8c, 0x74, 0x04, 0x82, 0xbb, 0xd4, 0x8f, 0xb3, 0x9d, 0x76, 0xa7, 0x0d,
                0xfd, 0x4b, 0xd5, 0x9d, 0xb1, 0x28, 0x80, 0x21, 0x74, 0x35, 0x03, 0x25, 0x9e, 0x94,
                0x8f, 0x6b, 0x1a, 0x18
            ]
        );
    }

    #[test]
    fn type_description_resolves_action_component_from_parent_artifact() {
        let root = scratch("action-component");
        let action_dir = root.join("share/example_interfaces/action");
        fs::create_dir_all(&action_dir).unwrap();
        fs::copy(
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("test_type_descriptions/example_interfaces__action__Fibonacci.json"),
            action_dir.join("Fibonacci.json"),
        )
        .unwrap();
        let description = TypeDescriptionResolver::from_prefixes([root.clone()])
            .resolve(
                "example_interfaces",
                InterfaceKind::Action,
                "Fibonacci_Goal",
            )
            .unwrap();
        assert_eq!(
            description.type_name,
            "example_interfaces/action/Fibonacci_Goal"
        );
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn type_description_missing_fails_closed() {
        assert!(matches!(
            fixtures().resolve("missing_pkg", InterfaceKind::Message, "Missing"),
            Err(TypeDescriptionError::DescriptionNotFound { .. })
        ));
    }

    #[test]
    fn type_description_truncated_fixture_is_typed_error() {
        let root = scratch("truncated");
        fs::write(root.join("std_msgs__msg__String.json"), b"{").unwrap();
        let error = TypeDescriptionResolver::from_prefixes([root.clone()])
            .resolve("std_msgs", InterfaceKind::Message, "String")
            .unwrap_err();
        assert!(matches!(error, TypeDescriptionError::Malformed { .. }));
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn type_description_mismatched_fixture_is_typed_error() {
        let root = scratch("mismatch");
        fs::copy(
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("test_type_descriptions/std_msgs__msg__String.json"),
            root.join("wrong_pkg__msg__Wrong.json"),
        )
        .unwrap();
        let error = TypeDescriptionResolver::from_prefixes([root.clone()])
            .resolve("wrong_pkg", InterfaceKind::Message, "Wrong")
            .unwrap_err();
        assert!(matches!(
            error,
            TypeDescriptionError::TypeNameMismatch { .. }
        ));
        fs::remove_dir_all(root).unwrap();
    }

    fn scratch(label: &str) -> std::path::PathBuf {
        let path = std::env::temp_dir().join(format!(
            "dora-rmw-zenoh-{label}-{}",
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        fs::create_dir(&path).unwrap();
        path
    }
}
