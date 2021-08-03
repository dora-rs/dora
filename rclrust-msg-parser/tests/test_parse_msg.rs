use std::path::PathBuf;

use anyhow::Result;
use rclrust_msg_parser::parse_message_file;
use rclrust_msg_types::*;

fn parse_msg_def(msg_name: &str) -> Result<Message> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join(format!("tests/test_msgs/msg/{}.msg", msg_name));
    parse_message_file("test_msgs", path)
}

#[test]
fn parse_arrays() -> Result<()> {
    let message = parse_msg_def("Arrays")?;

    assert_eq!(message.package, "test_msgs".to_string());
    assert_eq!(message.name, "Arrays".to_string());
    assert_eq!(
        message.members[0],
        Member {
            name: "bool_values".into(),
            r#type: MemberType::Array(Array {
                value_type: BasicType::Bool.into(),
                size: 3,
            }),
            default: None,
        }
    );

    Ok(())
}

#[test]
fn parse_basic_types() -> Result<()> {
    let result = parse_msg_def("BasicTypes")?;

    assert_eq!(result.members[0].name, "bool_value".to_string());
    assert_eq!(
        result.members[0].r#type,
        MemberType::BasicType(BasicType::Bool)
    );
    assert_eq!(result.members[0].default, None);

    Ok(())
}

#[test]
fn parse_bounded_sequences() -> Result<()> {
    let _result = parse_msg_def("BoundedSequences")?;
    Ok(())
}

#[test]
fn parse_constants() -> Result<()> {
    let _result = parse_msg_def("Constants")?;
    Ok(())
}

#[test]
fn parse_defaults() -> Result<()> {
    let _result = parse_msg_def("Defaults")?;
    Ok(())
}

#[test]
fn parse_empty() -> Result<()> {
    let _result = parse_msg_def("Empty")?;
    Ok(())
}

#[test]
fn parse_multi_nested() -> Result<()> {
    let _result = parse_msg_def("MultiNested")?;
    Ok(())
}

#[test]
fn parse_nested() -> Result<()> {
    let _result = parse_msg_def("Nested")?;
    Ok(())
}

#[test]
fn parse_strings() -> Result<()> {
    let _result = parse_msg_def("Strings")?;
    Ok(())
}

#[test]
fn parse_unbounded_sequences() -> Result<()> {
    let _result = parse_msg_def("UnboundedSequences")?;
    Ok(())
}

#[test]
fn parse_wstrings() -> Result<()> {
    let _result = parse_msg_def("WStrings")?;
    Ok(())
}
