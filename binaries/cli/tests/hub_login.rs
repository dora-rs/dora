//! `dora hub login` is a token-paste flow (like `cargo login`), not a
//! username/password sign-in: the token is created on the Hub website and the
//! CLI only stores it. These tests cover the non-interactive half — the
//! positional argument and piped stdin — plus the surface that would tell a
//! user the old flow is back.

use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

fn login(token_file: &Path) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_dora"));
    command
        .args(["hub", "login"])
        .env("DORA_HUB_TOKEN_FILE", token_file)
        .env_remove("DORA_HUB_REGISTRY_URL")
        .env_remove("DORA_HUB_TOKEN");
    command
}

fn temp_token_file(dir: &tempfile::TempDir) -> PathBuf {
    dir.path().join("hub-token")
}

#[test]
fn saves_a_token_piped_on_stdin() {
    let dir = tempfile::tempdir().unwrap();
    let token_file = temp_token_file(&dir);
    let mut child = login(&token_file)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    stdin.write_all(b"dora_secret_token\n").unwrap();
    drop(stdin);

    let output = child.wait_with_output().unwrap();
    assert!(
        output.status.success(),
        "login failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("please paste the token found on https://dora.pub/account/tokens below"),
        "prompt missing from stdout: {stdout}"
    );
    assert!(
        stdout.contains(&format!("token saved to {}", token_file.display())),
        "saved path missing from stdout: {stdout}"
    );
    assert_eq!(
        fs::read_to_string(&token_file).unwrap(),
        "dora_secret_token\n"
    );

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mode = fs::metadata(&token_file).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, 0o600, "the token file must be private");
    }
}

#[test]
fn accepts_the_token_as_an_argument() {
    let dir = tempfile::tempdir().unwrap();
    let token_file = temp_token_file(&dir);
    let output = login(&token_file)
        .arg("dora_argument_token")
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "login failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        !stdout.contains("please paste"),
        "an argument should skip the prompt: {stdout}"
    );
    assert_eq!(
        fs::read_to_string(&token_file).unwrap(),
        "dora_argument_token\n"
    );
}

#[test]
fn prompts_for_the_configured_registry() {
    let dir = tempfile::tempdir().unwrap();
    let token_file = temp_token_file(&dir);
    let mut child = login(&token_file)
        .env("DORA_HUB_REGISTRY_URL", "http://127.0.0.1:8080")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    stdin.write_all(b"mirror_token\n").unwrap();
    drop(stdin);

    let output = child.wait_with_output().unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout
            .contains("please paste the token found on http://127.0.0.1:8080/account/tokens below"),
        "prompt should follow DORA_HUB_REGISTRY_URL: {stdout}"
    );
    assert_eq!(fs::read_to_string(&token_file).unwrap(), "mirror_token\n");
}

#[test]
fn rejects_an_empty_token() {
    let dir = tempfile::tempdir().unwrap();
    let token_file = temp_token_file(&dir);
    let output = login(&token_file).arg("   ").output().unwrap();
    assert!(!output.status.success(), "an empty token must be rejected");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("token must not be empty"),
        "unexpected error: {stderr}"
    );
    assert!(!token_file.exists(), "a rejected token must not be saved");
}

#[test]
fn help_describes_the_token_paste_flow_only() {
    let output = Command::new(env!("CARGO_BIN_EXE_dora"))
        .args(["hub", "login", "--help"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let help = String::from_utf8_lossy(&output.stdout);
    assert!(help.contains("TOKEN"), "TOKEN argument missing: {help}");
    assert!(
        help.contains("account/tokens"),
        "the token page should be discoverable from help: {help}"
    );
    for stale in ["--email", "--password", "--token-name", "--scopes"] {
        assert!(
            !help.contains(stale),
            "`{stale}` is part of the removed sign-in flow: {help}"
        );
    }
}
