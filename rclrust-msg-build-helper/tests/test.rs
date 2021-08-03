use anyhow::Result;
use rclrust_msg_build_helper::parse::get_packages_msgs;
use std::path::PathBuf;

#[test]
fn get_ros_msgs_msg() -> Result<()> {
    let test_file_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/test_data/msg");

    let result = get_packages_msgs(&[&test_file_path])?;
    let item = result.get("std_msgs").unwrap();

    assert_eq!(item.msgs.len(), 16);
    assert!(item.srvs.is_empty());
    assert!(item.actions.is_empty());
    assert!(item.msgs.iter().find(|&v| v.name == "Bool").is_some());

    Ok(())
}

#[test]
fn get_ros_msgs_srv() -> Result<()> {
    let test_file_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/test_data/srv");

    let result = get_packages_msgs(&[&test_file_path])?;
    let item = result.get("std_srvs").unwrap();

    assert!(item.msgs.is_empty());
    assert_eq!(item.srvs.len(), 3);
    assert!(item.actions.is_empty());
    assert!(item.srvs.iter().find(|&v| v.name == "SetBool").is_some());

    Ok(())
}

#[test]
fn get_ros_msgs_action() -> Result<()> {
    let test_file_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/test_data/action");

    let result = get_packages_msgs(&[&test_file_path])?;
    let item = result.get("action_tutorials_interfaces").unwrap();

    assert!(item.msgs.is_empty());
    assert!(item.srvs.is_empty());
    assert_eq!(item.actions.len(), 1);
    assert!(item
        .actions
        .iter()
        .find(|&v| v.name == "Fibonacci")
        .is_some());

    Ok(())
}
