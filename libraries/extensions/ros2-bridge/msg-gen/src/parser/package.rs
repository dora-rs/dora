use std::{collections::HashMap, path::Path};

use anyhow::{Context, Result};
use glob::glob;

use super::{action::parse_action_file, message::parse_message_file, service::parse_service_file};
use crate::types::Package;

fn get_ros_msgs_each_package<P: AsRef<Path>>(_root_dir: P) -> Result<Vec<Package>> {
    let mut map: HashMap<String, Package> = HashMap::new();
    if let Ok(ament_prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
        let pattern = ament_prefix_path.clone() + "/**/msg/*.msg";
        for entry in glob(&pattern).expect("Failed to read glob pattern") {
            match entry {
                Ok(path) => {
                    let package = path
                        .parent()
                        .context("Should have a msg folder")?
                        .parent()
                        .context("should have a package folder")?
                        .file_name()
                        .context("folder name should exist")?
                        .to_string_lossy()
                        .to_string();
                    match map.get_mut(&package) {
                        Some(p) => {
                            p.messages.push(parse_message_file(&package, path.clone())?);
                        }
                        None => {
                            let mut p = Package::new(package.clone());
                            p.messages.push(parse_message_file(&package, path.clone())?);
                            map.insert(package, p);
                        }
                    };
                }
                Err(e) => eprintln!("{:?}", e),
            }
        }
        let pattern = ament_prefix_path.clone() + "/**/srv/*.srv";
        for entry in glob(&pattern).expect("Failed to read glob pattern") {
            match entry {
                Ok(path) => {
                    let package = path
                        .parent()
                        .context("Should have a msg folder")?
                        .parent()
                        .context("should have a package folder")?
                        .file_name()
                        .context("folder name should exist")?
                        .to_string_lossy()
                        .to_string();
                    match map.get_mut(&package) {
                        Some(p) => {
                            p.services.push(parse_service_file(&package, path.clone())?);
                        }
                        None => {
                            let mut p = Package::new(package.clone());
                            p.services.push(parse_service_file(&package, path.clone())?);
                            map.insert(package, p);
                        }
                    };
                }
                Err(e) => eprintln!("{:?}", e),
            }
        }
        let pattern = ament_prefix_path + "/**/action/*.action";
        for entry in glob(&pattern).expect("Failed to read glob pattern") {
            match entry {
                Ok(path) => {
                    let package = path
                        .clone()
                        .parent()
                        .context("Should have a msg folder")?
                        .parent()
                        .context("should have a package folder")?
                        .file_name()
                        .context("folder name should exist")?
                        .to_string_lossy()
                        .to_string();
                    match map.get_mut(&package) {
                        Some(p) => {
                            p.actions.push(
                                parse_action_file(&package, path.clone())
                                    .context("could not parse action")?,
                            );
                        }
                        None => {
                            let mut p = Package::new(package.clone());
                            p.actions.push(
                                parse_action_file(&package, path.clone())
                                    .context("could not parse message")?,
                            );
                            map.insert(package, p);
                        }
                    };
                }
                Err(e) => eprintln!("{:?}", e),
            }
        }
        debug_assert!(
            map.len() > 0,
            "it seens that no package was generated from your AMENT_PREFIX_PATH directory"
        );
    }
    let packages = map.into_values().collect();
    Ok(packages)
}

pub fn get_packages<P>(paths: &[P]) -> Result<Vec<Package>>
where
    P: AsRef<Path>,
{
    let mut packages = paths
        .iter()
        .map(get_ros_msgs_each_package)
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .flatten()
        .filter(|p| !p.is_empty())
        .collect::<Vec<_>>();

    packages.sort_by_key(|p| p.name.clone());
    packages.dedup_by_key(|p| p.name.clone());

    Ok(packages)
}
