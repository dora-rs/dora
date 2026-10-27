use std::{
    collections::{BTreeSet, HashMap},
    path::{Path, PathBuf},
};

use anyhow::{Context, Result};
use glob::glob;

use super::{action::parse_action_file, message::parse_message_file, service::parse_service_file};
use crate::types::Package;

fn get_ros_msgs_each_package<P: AsRef<Path>>(root_dirs_in: &[P]) -> Result<Vec<Package>> {
    let mut map: HashMap<String, Package> = HashMap::new();

    let ros_formats = ["msg", "srv", "action"];

    // An empty entry, as a leading, trailing or doubled `:` in
    // AMENT_PREFIX_PATH leaves, names no prefix. Skip it rather than
    // discarding the packages found under the other entries. A prefix listed
    // twice is scanned once: its packages would otherwise pass the
    // first-prefix check below and have every message parsed twice.
    let mut root_dirs: Vec<&Path> = Vec::new();
    for root_dir in root_dirs_in.iter().map(AsRef::as_ref) {
        if root_dir != Path::new("") && !root_dirs.contains(&root_dir) {
            root_dirs.push(root_dir);
        }
    }
    if root_dirs.is_empty() {
        println!("cargo::warning=AMENT_PREFIX_PATH pointed to ''");
        return Ok(Vec::new());
    }

    for root_dir in root_dirs {
        for ros_format in ros_formats.iter() {
            let pattern =
                root_dir.to_string_lossy().to_string() + "/**/" + ros_format + "/*." + ros_format;
            for entry in glob(&pattern).context("Failed to read glob pattern")? {
                let path = entry.context("Could not glob given path")?;
                let file_name = path.file_name().unwrap().to_str().unwrap().to_string();

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
                }
                // Each `Package` keeps the directory it was first found in.
                // AMENT_PREFIX_PATH lists overlays before underlays, so the
                // first copy of a package shadows every later one, as it does
                // in ROS itself. Without this, a package rebuilt in an overlay
                // and also installed in `/opt/ros/<distro>` was parsed from
                // both prefixes into one `Package`, holding each message
                // twice, and the generated bindings defined every struct twice.
                let p = map.entry(package.clone()).or_insert_with(|| {
                    let mut p = Package::new(package.clone());
                    p.path = package_path.to_owned();
                    p
                });
                if p.path != package_path {
                    println!(
                        "cargo::warning=found two versions of package: {:?}, message: {:?}. will skip the one in: {:#?}",
                        package, file_name, path
                    );
                    continue;
                }

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
                    _ => unreachable!("ros_formats only contains msg, srv, action"),
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
    for pkg in map.values() {
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
    let dependencies: HashMap<String, BTreeSet<String>> = dependencies
        .keys()
        .map(|name| (name.clone(), transitive_dependencies(name, &dependencies)))
        .collect();

    for package in &mut packages {
        package.dependencies = dependencies
            .get(&package.name)
            .unwrap()
            .iter()
            .filter_map(|dep| map.get(dep).cloned())
            .collect();
        package.sort_messages();
    }

    Ok(packages)
}

/// Every package `name` depends on, directly or indirectly, given each
/// package's direct dependencies. Walks the graph with a visited set, so a
/// dependency cycle (legal for `exec_depend`) terminates, and a package is
/// never its own dependency.
fn transitive_dependencies(
    name: &str,
    dependencies: &HashMap<String, BTreeSet<String>>,
) -> BTreeSet<String> {
    let mut found = BTreeSet::new();
    let mut pending: Vec<&str> = dependencies
        .get(name)
        .into_iter()
        .flatten()
        .map(String::as_str)
        .collect();
    while let Some(dep) = pending.pop() {
        if dep == name || !found.insert(dep.to_owned()) {
            continue;
        }
        pending.extend(
            dependencies
                .get(dep)
                .into_iter()
                .flatten()
                .map(String::as_str),
        );
    }
    found
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
            Err(e) => anyhow::bail!(
                "XML parse error at position {}: {:?}",
                reader.buffer_position(),
                e
            ),
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

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::get_ros_msgs_each_package;

    /// An ament prefix with one message per package and the given
    /// `<depend>`s, under a fresh temp directory.
    fn prefix(test: &str, packages: &[(&str, &[&str])]) -> PathBuf {
        let root = std::env::temp_dir().join(format!("dora-msg-gen-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        for (name, deps) in packages {
            let package = root.join("share").join(name);
            std::fs::create_dir_all(package.join("msg")).unwrap();
            std::fs::write(package.join("msg").join("Value.msg"), "int32 value\n").unwrap();
            let deps: String = deps
                .iter()
                .map(|dep| format!("<depend>{dep}</depend>"))
                .collect();
            std::fs::write(
                package.join("package.xml"),
                format!("<package format=\"3\"><name>{name}</name>{deps}</package>"),
            )
            .unwrap();
        }
        root
    }

    fn dependencies_of(packages: &[crate::types::Package], name: &str) -> Vec<String> {
        let package = packages.iter().find(|p| p.name == name).unwrap();
        let mut deps: Vec<_> = package
            .dependencies
            .iter()
            .map(|d| d.name.clone())
            .collect();
        deps.sort();
        deps
    }

    #[test]
    fn dependencies_are_transitive() {
        let root = prefix("chain", &[("a", &["b"]), ("b", &["c"]), ("c", &[])]);
        let packages = get_ros_msgs_each_package(&[&root]).unwrap();
        assert_eq!(dependencies_of(&packages, "a"), ["b", "c"]);
        assert_eq!(dependencies_of(&packages, "b"), ["c"]);
        assert!(dependencies_of(&packages, "c").is_empty());
        std::fs::remove_dir_all(root).unwrap();
    }

    /// `exec_depend` cycles are legal in ROS (only build dependencies must be
    /// acyclic); they used to recurse until the stack overflowed.
    #[test]
    fn a_dependency_cycle_does_not_overflow_the_stack() {
        let root = prefix("cycle", &[("a", &["b"]), ("b", &["a"])]);
        let packages = get_ros_msgs_each_package(&[&root]).unwrap();
        assert_eq!(dependencies_of(&packages, "a"), ["b"]);
        assert_eq!(dependencies_of(&packages, "b"), ["a"]);
        std::fs::remove_dir_all(root).unwrap();
    }

    /// An overlay that rebuilds a package also installed in an underlay must
    /// shadow it, not be merged with it: each message is parsed once, from the
    /// first prefix on the path.
    #[test]
    fn a_package_in_two_prefixes_is_taken_from_the_first() {
        let overlay = prefix("overlay", &[("a", &[])]);
        let underlay = prefix("underlay", &[("a", &[])]);
        let packages = get_ros_msgs_each_package(&[&overlay, &underlay]).unwrap();
        let a = packages.iter().find(|p| p.name == "a").unwrap();
        assert_eq!(a.messages.len(), 1);
        assert!(a.path.starts_with(&overlay), "path: {}", a.path.display());
        std::fs::remove_dir_all(overlay).unwrap();
        std::fs::remove_dir_all(underlay).unwrap();
    }

    #[test]
    fn a_prefix_listed_twice_is_parsed_once() {
        let root = prefix("twice", &[("a", &[])]);
        let packages = get_ros_msgs_each_package(&[&root, &root]).unwrap();
        let a = packages.iter().find(|p| p.name == "a").unwrap();
        assert_eq!(a.messages.len(), 1);
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn an_empty_prefix_entry_does_not_discard_the_others() {
        let root = prefix("empty-entry", &[("a", &[])]);
        let empty = PathBuf::new();
        for roots in [[&root, &empty], [&empty, &root]] {
            let packages = get_ros_msgs_each_package(&roots).unwrap();
            assert_eq!(packages.len(), 1);
            assert_eq!(packages[0].name, "a");
        }
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn a_package_depending_on_itself_is_not_its_own_dependency() {
        let root = prefix("self", &[("a", &["a"])]);
        let packages = get_ros_msgs_each_package(&[&root]).unwrap();
        assert!(dependencies_of(&packages, "a").is_empty());
        std::fs::remove_dir_all(root).unwrap();
    }
}
