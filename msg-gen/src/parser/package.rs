use std::{
    fs::{self, File},
    io::{BufRead, BufReader},
    path::Path,
};

use anyhow::Result;

use super::{action::parse_action_file, message::parse_message_file, service::parse_service_file};
use crate::types::Package;

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

fn get_ros_msgs_each_package<P: AsRef<Path>>(root_dir: P) -> Result<Vec<Package>> {
    let dir = root_dir.as_ref().join(ROSIDL_INTERFACES);

    let mut packages = Vec::new();

    let paths = match fs::read_dir(dir) {
        Ok(paths) => paths,
        Err(_) => {
            return Ok(packages);
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

        let mut package = Package::new(file_name.clone());
        for line in BufReader::new(File::open(path)?).lines() {
            match parse_line(&line?) {
                Some(("msg", v)) => {
                    let msg = parse_message_file(
                        &file_name,
                        root_dir
                            .as_ref()
                            .join(format!("share/{}/msg/{}.msg", file_name, v)),
                    )?;
                    package.messages.push(msg);
                }
                Some(("srv", v)) => {
                    let srv = parse_service_file(
                        &file_name,
                        root_dir
                            .as_ref()
                            .join(format!("share/{}/srv/{}.srv", file_name, v)),
                    )?;
                    package.services.push(srv);
                }
                Some(("action", v)) => {
                    let action = parse_action_file(
                        &file_name,
                        root_dir
                            .as_ref()
                            .join(format!("share/{}/action/{}.action", file_name, v)),
                    )?;
                    package.actions.push(action);
                }
                Some(_) => unreachable!(),
                None => {}
            }
        }
        packages.push(package);
    }
    Ok(packages)
}

pub fn get_packages(paths: &[&Path]) -> Result<Vec<Package>> {
    let mut packages = paths
        .iter()
        .map(|&path| get_ros_msgs_each_package(path))
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .flatten()
        .filter(|p| !p.is_empty())
        .collect::<Vec<_>>();

    packages.sort_by_key(|p| p.name.clone());
    packages.dedup_by_key(|p| p.name.clone());

    Ok(packages)
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
