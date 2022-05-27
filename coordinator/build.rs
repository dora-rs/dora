use std::{
    path::{Path, PathBuf},
    process::Command,
};

fn main() {
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());

    build_runtime(&out_dir);

    println!("cargo:rerun-if-changed=.");
}

fn build_runtime(out_dir: &Path) {
    let mut cmd = Command::new(std::env::var("CARGO").unwrap());
    cmd.arg("build");
    cmd.arg("-p").arg("dora-runtime");
    cmd.arg("--release");
    let target_dir = out_dir.join("runtime");
    cmd.arg("--target-dir").arg(&target_dir);
    let status = cmd.status().unwrap();
    if !status.success() {
        panic!("runtime build failed");
    }
    println!(
        "cargo:rustc-env=DORA_RUNTIME_PATH={}",
        target_dir.join("release").join("dora-runtime").display()
    );
    println!("cargo:rerun-if-changed=../runtime");
}
