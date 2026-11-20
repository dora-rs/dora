use std::process::Command;

#[test]
fn start_warns_for_top_level_http_distribution() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(
        dir.path().join("dataflow.yml"),
        "deploy:\n  distribute: http\nnodes:\n  - id: worker\n    path: dynamic\n",
    )
    .unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_dora"))
        .current_dir(dir.path())
        .args(["start", "dataflow.yml", "--coordinator-port", "0"])
        .output()
        .unwrap();
    assert!(
        String::from_utf8_lossy(&output.stderr).contains("warning: deploy.distribute"),
        "{output:?}"
    );
}

#[test]
fn build_warns_only_for_unimplemented_distribution() {
    let dir = tempfile::tempdir().unwrap();
    for strategy in ["local", "scp", "http"] {
        let descriptor = dir.path().join("dataflow.yml");
        std::fs::write(
            &descriptor,
            format!("nodes:\n  - id: worker\n    path: dynamic\n    deploy:\n      distribute: {strategy}\n"),
        )
        .unwrap();
        let output = Command::new(env!("CARGO_BIN_EXE_dora"))
            .current_dir(dir.path())
            .args(["build", "--local", "dataflow.yml"])
            .output()
            .unwrap();
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{strategy}: {stderr}");
        assert_eq!(
            stderr.contains("warning: deploy.distribute"),
            strategy != "local",
            "{strategy}: {stderr}"
        );
    }
}
