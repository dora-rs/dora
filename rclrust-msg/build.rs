use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

use heck::SnakeCase;
use rclrust_msg_build_helper::codegen;
use rclrust_msg_build_helper::parse::{get_packages_msgs, RosPackageMsgs};
use sailfish::TemplateOnce;

#[derive(Debug, TemplateOnce)]
#[template(path = "mod.rs.stpl", delimiter = '@', escape = false)]
struct ModuleTemplate<'a> {
    package: &'a str,
    msgs: RosPackageMsgs,
}

fn main() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-changed=templates/mod.rs.stpl");
    println!("cargo:rerun-if-changed=templates/msg.rs.stpl");
    println!("cargo:rerun-if-changed=templates/srv.rs.stpl");
    println!("cargo:rerun-if-changed=templates/action.rs.stpl");

    let ament_prefix_paths =
        std::env::var("AMENT_PREFIX_PATH").expect("$AMENT_PREFIX_PATH is supposed to be set.");
    for ament_prefix_path in ament_prefix_paths.split(':') {
        println!("cargo:rustc-link-search=native={}/lib", ament_prefix_path);
    }

    let paths = ament_prefix_paths
        .split(':')
        .map(Path::new)
        .collect::<Vec<_>>();

    let mut f =
        File::create(PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("gen.rs")).unwrap();

    for (package, msgs) in get_packages_msgs(&paths).unwrap() {
        let ctx = ModuleTemplate {
            package: &package,
            msgs,
        };

        write!(f, "{}", ctx.render_once().unwrap()).unwrap();
    }
}
