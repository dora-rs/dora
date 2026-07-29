use dora_message::descriptor::RmwZenohCompatibility;
pub use dora_ros2_bridge_msg_gen::type_description::TypeDescriptionResolver;
use dora_ros2_bridge_msg_gen::type_description::{InterfaceKind, TypeDescriptionError};
use thiserror::Error;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TypeHash {
    HumbleUnsupported,
    Rep2016([u8; 32]),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RosTypeIdentity {
    pub ros_name: String,
    pub dds_name: String,
    pub hash: TypeHash,
}

impl RosTypeIdentity {
    pub fn key_hash_component(&self) -> String {
        match self.hash {
            TypeHash::HumbleUnsupported => "TypeHashNotSupported".into(),
            TypeHash::Rep2016(bytes) => format!("RIHS01_{}", encode_hex(&bytes)),
        }
    }
}

#[derive(Debug, Error)]
pub enum TypeIdentityError {
    #[error(transparent)]
    Description(#[from] TypeDescriptionError),
}

pub fn resolve_message(
    profile: RmwZenohCompatibility,
    package: &str,
    name: &str,
    resolver: &TypeDescriptionResolver,
) -> Result<RosTypeIdentity, TypeIdentityError> {
    resolve(
        profile,
        package,
        InterfaceKind::Message,
        name,
        format!("{package}::msg::dds_::{name}_"),
        resolver,
    )
}

pub fn resolve_service(
    profile: RmwZenohCompatibility,
    package: &str,
    name: &str,
    resolver: &TypeDescriptionResolver,
) -> Result<RosTypeIdentity, TypeIdentityError> {
    resolve(
        profile,
        package,
        InterfaceKind::Service,
        name,
        format!("{package}::srv::dds_::{name}_"),
        resolver,
    )
}

pub fn resolve_action(
    profile: RmwZenohCompatibility,
    package: &str,
    name: &str,
    resolver: &TypeDescriptionResolver,
) -> Result<Vec<RosTypeIdentity>, TypeIdentityError> {
    let mut identities = vec![resolve(
        profile,
        package,
        InterfaceKind::Action,
        name,
        format!("{package}::action::dds_::{name}_"),
        resolver,
    )?];
    for component in dora_ros2_bridge_msg_gen::types::action_component_names(name) {
        identities.push(resolve(
            profile,
            package,
            InterfaceKind::Action,
            &component,
            format!("{package}::action::dds_::{component}_"),
            resolver,
        )?);
    }
    Ok(identities)
}

fn resolve(
    profile: RmwZenohCompatibility,
    package: &str,
    kind: InterfaceKind,
    name: &str,
    dds_name: String,
    resolver: &TypeDescriptionResolver,
) -> Result<RosTypeIdentity, TypeIdentityError> {
    let ros_name = format!(
        "{package}/{}/{name}",
        match kind {
            InterfaceKind::Message => "msg",
            InterfaceKind::Service => "srv",
            InterfaceKind::Action => "action",
        }
    );
    let hash = match profile {
        RmwZenohCompatibility::Humble => TypeHash::HumbleUnsupported,
        RmwZenohCompatibility::Rep2016 => {
            TypeHash::Rep2016(resolver.resolve(package, kind, name)?.hash)
        }
    };
    Ok(RosTypeIdentity {
        ros_name,
        dds_name,
        hash,
    })
}

fn encode_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::{TypeDescriptionResolver, TypeHash, resolve_message};
    use dora_message::descriptor::RmwZenohCompatibility::{Humble, Rep2016};

    fn fixtures() -> TypeDescriptionResolver {
        TypeDescriptionResolver::from_prefixes([std::path::PathBuf::from(env!(
            "CARGO_MANIFEST_DIR"
        ))
        .join("msg-gen/test_type_descriptions")])
    }

    #[test]
    fn compatibility_humble_topic_identity_uses_unsupported_hash_literal() {
        let id = resolve_message(Humble, "std_msgs", "String", &fixtures()).unwrap();
        assert_eq!(id.dds_name, "std_msgs::msg::dds_::String_");
        assert_eq!(id.hash, TypeHash::HumbleUnsupported);
        assert_eq!(id.key_hash_component(), "TypeHashNotSupported");
    }

    #[test]
    fn compatibility_rep2016_identity_matches_installed_fixture() {
        let id = resolve_message(Rep2016, "std_msgs", "String", &fixtures()).unwrap();
        assert_eq!(
            id.key_hash_component(),
            "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
        );
    }

    #[test]
    fn compatibility_rep2016_missing_description_fails_closed() {
        assert!(resolve_message(Rep2016, "missing_pkg", "Missing", &fixtures()).is_err());
    }

    #[test]
    fn compatibility_rep2016_action_expands_all_generated_components() {
        let ids =
            super::resolve_action(Rep2016, "example_interfaces", "Fibonacci", &fixtures()).unwrap();
        assert_eq!(ids.len(), 7);
        assert!(
            ids.iter()
                .all(|identity| matches!(identity.hash, TypeHash::Rep2016(_)))
        );
        assert!(
            ids.iter()
                .any(|identity| identity.ros_name.ends_with("Fibonacci_SendGoal"))
        );
        assert!(
            ids.iter()
                .any(|identity| identity.ros_name.ends_with("Fibonacci_FeedbackMessage"))
        );
    }
}
