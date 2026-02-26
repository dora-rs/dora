//! Shared-token authentication for coordinator connections.
//!
//! On startup the coordinator generates a random hex token and writes it to
//! a well-known file. The CLI and daemons read this file before connecting
//! and append `?token=<hex>` to the WebSocket URL.

use std::{
    fmt, fs,
    io::Write,
    path::{Path, PathBuf},
};

/// Length of the raw token in bytes (32 bytes = 64 hex chars).
const TOKEN_BYTES: usize = 32;

/// File name used for storing the auth token.
const TOKEN_FILENAME: &str = ".adora-token";

/// Opaque authentication token.
///
/// `PartialEq`/`Eq` are intentionally not derived to prevent accidental
/// non-constant-time comparisons. Use [`constant_time_eq`] instead.
#[derive(Clone)]
pub struct AuthToken(String);

impl AuthToken {
    /// Create a token from a hex string (e.g. read from file or env var).
    pub fn from_hex(hex: impl Into<String>) -> Self {
        Self(hex.into())
    }

    /// Return the hex representation.
    pub fn as_hex(&self) -> &str {
        &self.0
    }
}

impl fmt::Debug for AuthToken {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "AuthToken(***)")
    }
}

/// Constant-time byte comparison to prevent timing side-channel attacks.
pub fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff = 0u8;
    for (x, y) in a.iter().zip(b.iter()) {
        diff |= x ^ y;
    }
    diff == 0
}

/// Generate a cryptographically random 32-byte token.
pub fn generate_token() -> AuthToken {
    let mut buf = [0u8; TOKEN_BYTES];
    getrandom::getrandom(&mut buf).expect("failed to generate random bytes");
    let hex: String = buf.iter().map(|b| format!("{b:02x}")).collect();
    AuthToken(hex)
}

/// Compute the token file path for a given working directory.
pub fn token_path(working_dir: &Path) -> PathBuf {
    working_dir.join(TOKEN_FILENAME)
}

/// Write the token to `<working_dir>/.adora-token` with owner-only permissions.
///
/// On Unix, the file is created with mode `0o600` atomically to prevent
/// a TOCTOU window where the file is briefly world-readable.
pub fn write_token(working_dir: &Path, token: &AuthToken) -> std::io::Result<()> {
    let path = token_path(working_dir);

    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        let mut file = fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .mode(0o600)
            .open(&path)?;
        file.write_all(token.as_hex().as_bytes())?;
        file.write_all(b"\n")?;
    }
    #[cfg(not(unix))]
    {
        let mut file = fs::File::create(&path)?;
        file.write_all(token.as_hex().as_bytes())?;
        file.write_all(b"\n")?;
    }

    Ok(())
}

/// Read the token from `<working_dir>/.adora-token`.
///
/// Returns `None` if the file does not exist.
pub fn read_token(working_dir: &Path) -> std::io::Result<Option<AuthToken>> {
    let path = token_path(working_dir);
    match fs::read_to_string(&path) {
        Ok(content) => Ok(Some(AuthToken(content.trim().to_string()))),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e),
    }
}

/// Remove the token file if it exists.
pub fn remove_token(working_dir: &Path) {
    let path = token_path(working_dir);
    let _ = fs::remove_file(path);
}

/// Attempt to read the auth token from (in order):
/// 1. `ADORA_AUTH_TOKEN` environment variable
/// 2. `<cwd>/.adora-token` file
///
/// Returns `None` if no token is found.
pub fn discover_token() -> Option<AuthToken> {
    // 1. Environment variable override
    if let Ok(val) = std::env::var("ADORA_AUTH_TOKEN") {
        if !val.is_empty() {
            return Some(AuthToken(val));
        }
    }

    // 2. Token file in current working directory
    if let Ok(cwd) = std::env::current_dir() {
        if let Ok(Some(token)) = read_token(&cwd) {
            return Some(token);
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generate_token_is_64_hex_chars() {
        let token = generate_token();
        assert_eq!(token.as_hex().len(), 64);
        assert!(token.as_hex().chars().all(|c| c.is_ascii_hexdigit()));
    }

    #[test]
    fn generate_tokens_are_unique() {
        let a = generate_token();
        let b = generate_token();
        assert_ne!(a.as_hex(), b.as_hex());
    }

    #[test]
    fn write_and_read_token() {
        let dir = tempfile::tempdir().unwrap();

        let token = generate_token();
        write_token(dir.path(), &token).unwrap();

        let read_back = read_token(dir.path()).unwrap().unwrap();
        assert_eq!(token.as_hex(), read_back.as_hex());

        remove_token(dir.path());
        assert!(read_token(dir.path()).unwrap().is_none());
    }

    #[test]
    fn constant_time_eq_works() {
        assert!(constant_time_eq(b"hello", b"hello"));
        assert!(!constant_time_eq(b"hello", b"world"));
        assert!(!constant_time_eq(b"hello", b"hell"));
        assert!(!constant_time_eq(b"", b"x"));
        assert!(constant_time_eq(b"", b""));
    }
}
