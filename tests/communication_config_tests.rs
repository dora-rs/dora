use dora_message::config::CommunicationConfig;
use serde_yaml;

#[test]
fn test_valid_tcp_config() {
    let yaml = r#"
_unstable_local: tcp
_unstable_remote: tcp
"#;

    let config: CommunicationConfig = serde_yaml::from_str(yaml).unwrap();
    assert_eq!(config.local.to_string(), "tcp");
    assert_eq!(config.remote.to_string(), "tcp");
}

#[test]
fn test_valid_unixdomain_config() {
    let yaml = r#"
_unstable_local: unixdomain
_unstable_remote: tcp
"#;

    let config: CommunicationConfig = serde_yaml::from_str(yaml).unwrap();
    assert_eq!(config.local.to_string(), "unixdomain");
}

#[test]
fn test_invalid_local_config() {
    let yaml = r#"
_unstable_local: bluetooth
"#;

    let result: Result<CommunicationConfig, _> = serde_yaml::from_str(yaml);
    assert!(result.is_err());
}

#[test]
fn test_unknown_field_should_fail() {
    let yaml = r#"
_unstable_local: tcp
unknown_field: test
"#;

    let result: Result<CommunicationConfig, _> = serde_yaml::from_str(yaml);
    assert!(result.is_err());
}
