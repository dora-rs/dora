use std::path::PathBuf;

use anyhow::Result;
use rclrust_msg_parser::parse_action_file;
use rclrust_msg_types::*;

fn parse_action_def(srv_name: &str) -> Result<Action> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join(format!("tests/test_msgs/action/{}.action", srv_name));
    parse_action_file("test_msgs", path)
}

#[test]
fn parse_fibonacci() -> Result<()> {
    let action = parse_action_def("Fibonacci")?;
    assert_eq!(action.package, "test_msgs".to_string());
    assert_eq!(action.name, "Fibonacci".to_string());

    assert_eq!(action.goal.name, "Fibonacci_Goal".to_string());
    assert_eq!(action.goal.members.len(), 1);
    assert_eq!(action.goal.members[0].name, "order".to_string());
    assert_eq!(
        action.goal.members[0].r#type,
        MemberType::BasicType(BasicType::I32)
    );
    assert_eq!(action.goal.constants.len(), 0);

    assert_eq!(action.result.name, "Fibonacci_Result".to_string());
    assert_eq!(action.result.members.len(), 1);
    assert_eq!(action.result.members[0].name, "sequence".to_string());
    assert_eq!(
        action.result.members[0].r#type,
        MemberType::Sequence(Sequence {
            value_type: NestableType::BasicType(BasicType::I32)
        })
    );
    assert_eq!(action.result.constants.len(), 0);

    assert_eq!(action.feedback.name, "Fibonacci_Feedback".to_string());
    assert_eq!(action.feedback.members.len(), 1);
    assert_eq!(action.feedback.members[0].name, "sequence".to_string());
    assert_eq!(
        action.feedback.members[0].r#type,
        MemberType::Sequence(Sequence {
            value_type: NestableType::BasicType(BasicType::I32)
        })
    );
    assert_eq!(action.feedback.constants.len(), 0);

    Ok(())
}
