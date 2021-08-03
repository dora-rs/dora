use std::fs;
use std::path::Path;

use anyhow::{Context, Result};
use rclrust_msg_types::Service;
use regex::Regex;

use crate::error::RclMsgError;
use crate::msg::parse_message_string;

const SERVICE_REQUEST_SUFFIX: &str = "_Request";
const SERVICE_RESPONSE_SUFFIX: &str = "_Response";

pub fn parse_service_file<P: AsRef<Path>>(pkg_name: &str, interface_file: P) -> Result<Service> {
    parse_service_string(
        pkg_name,
        interface_file
            .as_ref()
            .file_stem()
            .unwrap()
            .to_str()
            .unwrap(),
        fs::read_to_string(interface_file.as_ref())?.as_str(),
    )
    .with_context(|| format!("Parse file error: {}", interface_file.as_ref().display()))
}

pub fn parse_service_string(
    pkg_name: &str,
    srv_name: &str,
    service_string: &str,
) -> Result<Service> {
    let re = Regex::new(r"(?m)^---$").unwrap();
    let service_blocks: Vec<_> = re.split(service_string).collect();
    if service_blocks.len() != 2 {
        return Err(RclMsgError::InvalidServiceSpecification(
            "Number of '---' separators nonconformant with service definition".into(),
        )
        .into());
    }

    Ok(Service {
        package: pkg_name.into(),
        name: srv_name.into(),
        request: parse_message_string(
            pkg_name,
            &format!("{}{}", srv_name, SERVICE_REQUEST_SUFFIX),
            service_blocks[0],
        )?,
        response: parse_message_string(
            pkg_name,
            &format!("{}{}", srv_name, SERVICE_RESPONSE_SUFFIX),
            service_blocks[1],
        )?,
    })
}
