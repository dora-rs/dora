//! Shared-token handshake for the direct-TCP cross-machine data plane.

use super::*;

/// Auth token for the direct-TCP data plane. When set (on every
/// participating daemon), each direct-TCP connection performs a token
/// handshake before the first data frame; a mismatched or missing token
/// drops the connection (the origin degrades to the zenoh relay). Unset
/// = no handshake (the daemon then relies on the port's reachability).
/// The token is NOT injected into spawned nodes (the handshake runs
/// entirely in the daemon; node-side code never reads it — injecting it
/// would leak the shared secret into every user node process, bot
/// review 5301862843).
///
/// Scope: the token authenticates the direct-TCP data plane only. The
/// zenoh control/relay plane (RegisterPool/FreePool/MemoryPoolWrite over
/// `dataflow_memory_pool_topic`) is unauthenticated — dora's mesh-daemon
/// model treats peers as trusted. When this token is relied on over an
/// untrusted link (edge↔cloud WAN), the zenoh transport itself must be
/// secured (e.g. TLS via the zenoh config), or the control plane must be
/// firewalled to trusted peers.
pub(crate) const CROSS_DATA_AUTH_TOKEN_ENV: &str = "DORA_MEMORY_POOL_AUTH_TOKEN";

pub(crate) fn cross_data_auth_token() -> Option<String> {
    std::env::var(CROSS_DATA_AUTH_TOKEN_ENV)
        .ok()
        .filter(|t| !t.is_empty())
}

/// Handshake frame: `[8-byte magic][u32 token_len][token]`, answered with
/// a single byte (1 = accepted, 0 = rejected + connection close).
pub(crate) const CROSS_DATA_AUTH_MAGIC: [u8; 9] = *b"DORA_AUTH";
pub(crate) const AUTH_OK: u8 = 1;
pub(crate) const AUTH_FAIL: u8 = 0;

/// Send-side handshake: runs once right after connecting, before any
/// data frame. The peer closes the connection on rejection, which
/// surfaces here as a read error or `AUTH_FAIL` byte.
pub(crate) async fn auth_handshake_send(
    stream: &mut tokio::net::TcpStream,
    token: &str,
) -> Result<(), String> {
    use tokio::io::AsyncWriteExt;
    stream
        .write_all(&CROSS_DATA_AUTH_MAGIC)
        .await
        .map_err(|e| format!("auth handshake write failed: {e}"))?;
    let token = token.as_bytes();
    stream
        .write_all(&(token.len() as u32).to_be_bytes())
        .await
        .map_err(|e| format!("auth handshake write failed: {e}"))?;
    stream
        .write_all(token)
        .await
        .map_err(|e| format!("auth handshake write failed: {e}"))?;
    stream
        .flush()
        .await
        .map_err(|e| format!("auth handshake flush failed: {e}"))?;
    let mut reply = [0u8; 1];
    tokio::io::AsyncReadExt::read_exact(stream, &mut reply)
        .await
        .map_err(|e| format!("auth handshake reply read failed: {e}"))?;
    if reply[0] != AUTH_OK {
        return Err("peer rejected the auth token".to_string());
    }
    Ok(())
}

/// Receive-side handshake: validates the peer's token. Bounded by
/// [`CROSS_DATA_READ_TIMEOUT`] so a connection that never sends the
/// handshake cannot pin a task (or an fd) forever.
pub(crate) async fn auth_handshake_verify(
    stream: &mut tokio::net::TcpStream,
) -> Result<(), String> {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    let mut magic = [0u8; 9];
    stream
        .read_exact(&mut magic)
        .await
        .map_err(|e| format!("auth handshake read failed: {e}"))?;
    if magic != CROSS_DATA_AUTH_MAGIC {
        let _ = stream.write_all(&[AUTH_FAIL]).await;
        return Err("bad auth handshake magic (peer without the shared token?)".to_string());
    }
    let mut len_bytes = [0u8; 4];
    stream
        .read_exact(&mut len_bytes)
        .await
        .map_err(|e| format!("auth handshake read failed: {e}"))?;
    let len = u32::from_be_bytes(len_bytes) as usize;
    if len > 4096 {
        let _ = stream.write_all(&[AUTH_FAIL]).await;
        return Err(format!("auth token too long ({len} bytes)"));
    }
    let mut token = vec![0u8; len];
    stream
        .read_exact(&mut token)
        .await
        .map_err(|e| format!("auth handshake read failed: {e}"))?;
    let expected = cross_data_auth_token().unwrap_or_default();
    // Constant-time comparison (parity with the message-layer auth): the
    // handshake token is a shared deployment secret, so a timing side
    // channel on the comparison would leak it byte by byte.
    let token_ok = token.len() == expected.len()
        && token
            .iter()
            .zip(expected.as_bytes())
            .fold(0u8, |acc, (a, b)| acc | (a ^ b))
            == 0;
    if token_ok {
        stream
            .write_all(&[AUTH_OK])
            .await
            .map_err(|e| format!("auth handshake reply write failed: {e}"))?;
        Ok(())
    } else {
        let _ = stream.write_all(&[AUTH_FAIL]).await;
        Err("peer auth token mismatch".to_string())
    }
}

/// Gate a fresh direct-TCP connection on the auth handshake.
///
/// Returns `Ok(())` when the peer passed the handshake (or no token is
/// configured — a token-less daemon accepts token-less peers), and
/// `Err(reason)` when the connection must be dropped *before* any frame
/// is served. This is the listener's consumption of
/// [`auth_handshake_verify`]; factored out so the accept loop's
/// reject-on-mismatch behavior is testable through a real socket.
///
/// Note the nested-result flatten: `timeout(...).await` is
/// `Result<Result<(), String>, Elapsed>` — an `Err` from the verify
/// (bad magic, over-long or wrong token) is the *inner* result and must
/// be rejected here, not just the outer timeout.
pub(crate) async fn cross_data_auth_gate(stream: &mut tokio::net::TcpStream) -> Result<(), String> {
    if cross_data_auth_token().is_none() {
        return Ok(());
    }
    match tokio::time::timeout(CROSS_DATA_READ_TIMEOUT, auth_handshake_verify(stream)).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(e)) => Err(format!("auth handshake rejected: {e}")),
        Err(_) => Err("auth handshake timed out".to_string()),
    }
}
