use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{BufRead, BufReader};
use std::path::Path;

use anyhow::Result;
use rclrust_msg_parser::{parse_action_file, parse_message_file, parse_service_file};
use rclrust_msg_types::{Action, Message, Service};

#[derive(Debug, Clone)]
pub struct RosPackageMsgs {
    pub msgs: Vec<Message>,
    pub srvs: Vec<Service>,
    pub actions: Vec<Action>,
}

impl RosPackageMsgs {
    const fn new() -> Self {
        Self {
            msgs: Vec::new(),
            srvs: Vec::new(),
            actions: Vec::new(),
        }
    }

    fn is_empty(&self) -> bool {
        self.msgs.is_empty() && self.srvs.is_empty() && self.actions.is_empty()
    }
}

pub type RosPackageMsgsMap = HashMap<String, RosPackageMsgs>;

const ROSIDL_INTERFACES: &str = "share/ament_index/resource_index/rosidl_interfaces";

const NAMESPACES: &[&str] = &["msg", "srv", "action"];

fn parse_line(line: &str) -> Option<(&str, &str)> {
    if !line.ends_with(".idl") {
        return None;
    }
    for &namespace in NAMESPACES {
        if line.starts_with(&format!("{}/", namespace)) {
            let name = &line[namespace.len() + 1..line.len() - 4];
            return Some((namespace, name));
        }
    }
    println!("Unknown type: {:?}", line);
    None
}

fn get_ros_msgs_each_package<P: AsRef<Path>>(root_dir: P) -> Result<Vec<(String, RosPackageMsgs)>> {
    let dir = root_dir.as_ref().join(ROSIDL_INTERFACES);

    let mut msgs = Vec::new();

    let paths = match fs::read_dir(dir) {
        Ok(paths) => paths,
        Err(_) => {
            return Ok(msgs);
        }
    };

    for path in paths {
        let path = path?.path();
        let file_name = path
            .clone()
            .file_name()
            .unwrap()
            .to_str()
            .unwrap()
            .to_string();
        // Hack
        if file_name == "libstatistics_collector" {
            continue;
        }

        let mut local_msgs = RosPackageMsgs::new();
        for line in BufReader::new(File::open(path)?).lines() {
            match parse_line(&line?) {
                Some(("msg", v)) => {
                    let msg = parse_message_file(
                        &file_name,
                        root_dir
                            .as_ref()
                            .join(format!("share/{}/msg/{}.msg", file_name, v)),
                    )?;
                    local_msgs.msgs.push(msg);
                }
                Some(("srv", v)) => {
                    let srv = parse_service_file(
                        &file_name,
                        root_dir
                            .as_ref()
                            .join(format!("share/{}/srv/{}.srv", file_name, v)),
                    )?;
                    local_msgs.srvs.push(srv);
                }
                Some(("action", v)) => {
                    let action = parse_action_file(
                        &file_name,
                        root_dir
                            .as_ref()
                            .join(format!("share/{}/action/{}.action", file_name, v)),
                    )?;
                    local_msgs.actions.push(action);
                }
                Some(_) => unreachable!(),
                None => {}
            }
        }
        msgs.push((file_name, local_msgs));
    }
    Ok(msgs)
}

pub fn get_packages_msgs(paths: &[&Path]) -> Result<RosPackageMsgsMap> {
    Ok(paths
        .iter()
        .map(|&path| get_ros_msgs_each_package(path))
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .flatten()
        .fold(RosPackageMsgsMap::new(), |mut acc, item| {
            if item.1.is_empty() || acc.contains_key(&item.0) {
                return acc;
            }
            acc.insert(item.0, item.1);
            acc
        }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_line_msg() {
        let result = parse_line("msg/TestHoge.idl").unwrap();

        assert_eq!(result.0, "msg");
        assert_eq!(result.1, "TestHoge");
    }

    #[test]
    fn parse_line_srv() {
        let result = parse_line("srv/TestHoge.idl").unwrap();

        assert_eq!(result.0, "srv");
        assert_eq!(result.1, "TestHoge");
    }

    #[test]
    fn parse_line_action() {
        let result = parse_line("action/TestHoge.idl").unwrap();

        assert_eq!(result.0, "action");
        assert_eq!(result.1, "TestHoge");
    }

    #[test]
    fn parse_line_wrong_namespace() {
        assert!(parse_line("test/Test.msg").is_none());
        assert!(parse_line("test/Test.srv").is_none());
        assert!(parse_line("test/Test.action").is_none());
    }

    #[test]
    fn parse_line_wrong_suffix() {
        assert!(parse_line("msg/Test.test").is_none());
        assert!(parse_line("srv/Test.test").is_none());
        assert!(parse_line("action/Test.test").is_none());
    }
}
