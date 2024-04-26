use std::{collections::HashMap, io::Empty, path::Path};

use anyhow::{Context, Result};
use glob::glob;
use tracing::warn;

use super::{action::parse_action_file, message::parse_message_file, service::parse_service_file};
use crate::types::Package;

fn get_ros_msgs_each_package<P: AsRef<Path>>(root_dir: P) -> Result<Vec<Package>> {
    let mut map: HashMap<String, Package> = HashMap::new();

    let ros_formats = vec!["msg", "srv", "action"];

    if root_dir.as_ref() != Path::new("") {
        let empty_vec: Vec<Package> = vec![];
        warn!("AMENT_PREFIX_PATH pointed to ''");
        return Ok(empty_vec);
    }

    for ros_format in ros_formats {
        let pattern = root_dir.as_ref().to_string_lossy().to_string()
            + "/**/"
            + ros_format
            + "/*."
            + ros_format;
        let mut visited_files = vec![];
        for entry in glob(&pattern).expect("Failed to read glob pattern") {
            match entry {
                Ok(path) => {
                    let file_name = path
                        .clone()
                        .file_name()
                        .unwrap()
                        .to_str()
                        .unwrap()
                        .to_string();

                    let package = path
                        .parent()
                        .context("Should have a msg folder")?
                        .parent()
                        .context("should have a package folder")?
                        .file_name()
                        .context("folder name should exist")?
                        .to_string_lossy()
                        .to_string();

                    // Hack
                    if file_name == "libstatistics_collector" {
                        continue;
                    } else if visited_files.contains(&(package.clone(), file_name.clone())) {
                        warn!(
                        "found two versions of package: {:?}, message: {:?}. will skip the one in: {:#?}",
                        package, file_name, path
                    );
                        continue;
                    } else {
                        visited_files.push((package.clone(), file_name.clone()));
                    }

                    match map.get_mut(&package) {
                        Some(p) => match ros_format {
                            "msg" => {
                                p.messages.push(parse_message_file(&package, path.clone())?);
                            }
                            "srv" => {
                                p.services.push(parse_service_file(&package, path.clone())?);
                            }
                            "action" => {
                                p.actions.push(parse_action_file(&package, path.clone())?);
                            }
                            _ => todo!(),
                        },
                        None => {
                            let mut p = Package::new(package.clone());
                            match ros_format {
                                "msg" => {
                                    p.messages.push(parse_message_file(&package, path.clone())?);
                                }
                                "srv" => {
                                    p.services.push(parse_service_file(&package, path.clone())?);
                                }
                                "action" => {
                                    p.actions.push(parse_action_file(&package, path.clone())?);
                                }
                                _ => todo!(),
                            }
                            map.insert(package, p);
                        }
                    };
                }
                Err(e) => eprintln!("{:?}", e),
            }
        }
    }
    debug_assert!(
        !map.is_empty(),
        "it seens that no package was generated from your AMENT_PREFIX_PATH directory"
    );

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
