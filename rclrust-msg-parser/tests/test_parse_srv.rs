use std::path::PathBuf;

use anyhow::Result;
use rclrust_msg_parser::parse_service_file;
use rclrust_msg_types::*;

fn parse_srv_def(srv_name: &str) -> Result<Service> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join(format!("tests/test_msgs/srv/{}.srv", srv_name));
    parse_service_file("test_msgs", path)
}

#[test]
fn parse_arrays() -> Result<()> {
    let result = parse_srv_def("Arrays")?;
    assert_eq!(result.package, "test_msgs".to_string());
    assert_eq!(result.name, "Arrays".to_string());
    assert_eq!(result.request.name, "Arrays_Request".to_string());
    assert_eq!(result.response.name, "Arrays_Response".to_string());
    Ok(())
}

#[test]
fn parse_basic_types() -> Result<()> {
    let _result = parse_srv_def("BasicTypes")?;
    Ok(())
}

#[test]
fn parse_empty() -> Result<()> {
    let _result = parse_srv_def("Empty")?;
    Ok(())
}
