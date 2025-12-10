use std::{
    collections::{BTreeSet, HashMap},
    path::{Path, PathBuf},
};

use anyhow::{Context, Result};
use glob::glob;

use super::{action::parse_action_file, message::parse_message_file, service::parse_service_file};
use crate::types::Package;

fn get_ros_msgs_each_package<P: AsRef<Path>>(root_dirs: &[P]) -> Result<Vec<Package>> {
    let mut map: HashMap<String, Package> = HashMap::new();

    let ros_formats = vec!["msg", "srv", "action"];

    // Return empty vector if root_dir is empty
    for root_dir in root_dirs {
        if root_dir.as_ref() == Path::new("") {
            let empty_vec: Vec<Package> = vec![];
            println!("cargo::warning=AMENT_PREFIX_PATH pointed to ''");
            return Ok(empty_vec);
        }

        for ros_format in ros_formats.iter() {
            let pattern = root_dir.as_ref().to_string_lossy().to_string()
                + "/**/"
                + ros_format
                + "/*."
                + ros_format;
            let mut visited_files = vec![];
            for entry in glob(&pattern).context("Failed to read glob pattern")? {
                let path = entry.context("Could not glob given path")?;
                let file_name = path
                    .clone()
                    .file_name()
                    .unwrap()
                    .to_str()
                    .unwrap()
                    .to_string();

                let package_path = path
                    .parent()
                    .context("Should have a msg folder")?
                    .parent()
                    .context("should have a package folder")?;

                let package = package_path
                    .file_name()
                    .context("folder name should exist")?
                    .to_string_lossy()
                    .to_string();

                // Hack
                if file_name == "libstatistics_collector" {
                    continue;
                } else if visited_files.contains(&(package.clone(), file_name.clone())) {
                    println!(
                        "cargo::warning=found two versions of package: {:?}, message: {:?}. will skip the one in: {:#?}",
                        package, file_name, path
                    );
                    continue;
                } else {
                    visited_files.push((package.clone(), file_name.clone()));
                }

                let p = map
                    .entry(package.clone())
                    .or_insert_with(|| Package::new(package.clone()));
                p.path = PathBuf::from(package_path);

                match *ros_format {
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
            }
        }
    }

    if map.is_empty() {
        println!(
            "cargo::warning=it seems that no package was generated from your AMENT_PREFIX_PATH directory"
        );
    }

    let mut packages = Vec::new();
    let mut dependencies = HashMap::<String, BTreeSet<String>>::new();
    for (_pkg_name, pkg) in &map {
        packages.push(pkg.clone());
    }
    for package in &mut packages {
        let xml = package.path.join("package.xml");

        let mut deps = parse_dependencies(xml)?;
        if !package.actions.is_empty() {
            deps.insert("action_msgs".to_owned());
        }
        deps.retain(|dep| map.contains_key(dep));
        dependencies.insert(package.name.clone(), deps);
    }

    // flatten the dependencies
    let dependencies = {
        let mut flattened_dependencies = HashMap::<String, BTreeSet<String>>::new();
        let mut unfinished_set: BTreeSet<String> = dependencies.keys().cloned().collect();
        // recursively flatten the dependencies
        while let Some(first) = unfinished_set.first().cloned() {
            flatten_dependencies(
                &first,
                &dependencies,
                &mut flattened_dependencies,
                &mut unfinished_set,
            );
        }
        flattened_dependencies
    };

    for package in &mut packages {
        package.dependencies = dependencies
            .get(&package.name)
            .unwrap()
            .iter()
            .filter_map(|dep| map.get(dep).map(|pkg| pkg.clone()))
            .collect();
    }

    Ok(packages)
}

/// Flatten the dependencies of packages recursively.
/// The name is the name of the package to expand.
/// The dependencies is the dependency trees of the packages with its direct dependencies.
/// flattened_dependencies records the flattened dependencies of the packages.
/// finished_set is the set of names of packages that have been finished.
fn flatten_dependencies(
    name: &str,
    dependencies: &HashMap<String, BTreeSet<String>>,
    flattened_dependencies: &mut HashMap<String, BTreeSet<String>>,
    unfinished_set: &mut BTreeSet<String>,
) -> BTreeSet<String> {
    if let Some(add_deps) = flattened_dependencies.get(name) {
        return add_deps.clone();
    }

    let mut add_deps = BTreeSet::new();
    let direct_deps: BTreeSet<String> = dependencies.get(name).unwrap().iter().cloned().collect();
    for dep_name in &direct_deps {
        let indirect_deps = flatten_dependencies(
            &dep_name,
            dependencies,
            flattened_dependencies,
            unfinished_set,
        );
        add_deps.extend(indirect_deps);
    }
    add_deps.extend(direct_deps);
    flattened_dependencies.insert(name.to_string(), add_deps.clone());
    unfinished_set.remove(name);
    add_deps
}

pub fn get_packages<P>(paths: &[P]) -> Result<Vec<Package>>
where
    P: AsRef<Path>,
{
    let mut packages = get_ros_msgs_each_package(paths)?;
    packages.retain(|p| !p.is_empty());

    packages.sort_by_key(|p| p.name.clone());
    packages.dedup_by_key(|p| p.name.clone());

    Ok(packages)
}

fn parse_dependencies(path: PathBuf) -> Result<BTreeSet<String>> {
    use quick_xml::events::Event;
    let mut deps = BTreeSet::new();
    let file = std::fs::File::open(path)?;
    let file_reader = std::io::BufReader::new(file);
    let mut reader = quick_xml::Reader::from_reader(file_reader);
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Err(e) => panic!("Error at position {}: {:?}", reader.buffer_position(), e),
            Ok(Event::Eof) => break, // Reached the end of file

            // Check for the start of any dependency tag
            Ok(Event::Start(e)) => {
                let name = e.name();
                let name = name.as_ref();
                if name == b"depend"
                    || name == b"exec_depend"
                    || name == b"buildtool_depend"
                    || name == b"build_depend"
                {
                    // Read the next event to get the inner text
                    match reader.read_event_into(&mut buf) {
                        Ok(Event::Text(t)) => {
                            // Convert the text to a String and add it
                            if let Ok(dep) = t.decode() {
                                deps.insert(dep.into_owned());
                            }
                        }
                        // Continue if it's not text (e.g., a comment, CDATA)
                        _ => continue,
                    }
                }
            }

            // Ignore other events (End, Empty, Comment, etc.)
            _ => (),
        }
        buf.clear(); // Clear the buffer for the next event
    }
    Ok(deps)
}
