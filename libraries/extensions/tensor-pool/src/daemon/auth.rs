//! Shared-token handshake for the direct-TCP cross-machine data plane.

#[cfg(test)]
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
/// zenoh control/relay plane (`PeerMessage` Register/Free/Write over
/// `dataflow_extension_topic`) is unauthenticated — dora's mesh-daemon
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

/// Handshake frame: `[9-byte magic][u32 token_len][token]`, answered with
/// a single byte (1 = accepted, 0 = rejected + connection close). The magic
/// is the 9 bytes `DORA_AUTH`.
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

/// Receive-side handshake: validates the peer's token and accepts it. The
/// listener bounds it by [`CROSS_DATA_AUTH_TIMEOUT`] (in [`auth_gate_then`])
/// so a connection that never sends the handshake cannot pin a task (or an
/// fd) for long.
#[cfg(test)]
pub(crate) async fn auth_handshake_verify(
    stream: &mut tokio::net::TcpStream,
) -> Result<(), String> {
    auth_handshake_check(stream).await?;
    auth_handshake_reply(stream, AUTH_OK).await
}

/// Answers a handshake that passed [`auth_handshake_check`].
async fn auth_handshake_reply(stream: &mut tokio::net::TcpStream, reply: u8) -> Result<(), String> {
    tokio::io::AsyncWriteExt::write_all(stream, &[reply])
        .await
        .map_err(|e| format!("auth handshake reply write failed: {e}"))
}

/// The receive-side handshake up to, not including, the `AUTH_OK` reply, so
/// the caller can still refuse a peer whose token matched. A mismatch is
/// answered with `AUTH_FAIL` here.
async fn auth_handshake_check(stream: &mut tokio::net::TcpStream) -> Result<(), String> {
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
/// is served. The listener itself goes through [`auth_gate_then`], which
/// this wraps, so the reject-on-mismatch behavior is testable through a
/// real socket.
///
/// Note the nested-result flatten: `timeout(...).await` is
/// `Result<Result<(), String>, Elapsed>` — an `Err` from the verify
/// (bad magic, over-long or wrong token) is the *inner* result and must
/// be rejected here, not just the outer timeout.
#[cfg(test)]
pub(crate) async fn cross_data_auth_gate(stream: &mut tokio::net::TcpStream) -> Result<(), String> {
    if cross_data_auth_token().is_none() {
        return Ok(());
    }
    auth_gate_within(stream, CROSS_DATA_AUTH_TIMEOUT).await
}

/// [`cross_data_auth_gate`] with an explicit bound, so tests can check the
/// timeout path without waiting out the real one.
#[cfg(test)]
pub(crate) async fn auth_gate_within(
    stream: &mut tokio::net::TcpStream,
    timeout: std::time::Duration,
) -> Result<(), String> {
    // `admit` always lets the peer in, so `Ok` is always `Ok(Some(()))`.
    auth_gate_then(stream, timeout, || Some(())).await.map(drop)
}

/// The listener's auth gate, with `admit` deciding between a matching token and
/// the `AUTH_OK` reply whether the peer is let in after all: `Ok(None)`, with
/// `AUTH_FAIL` sent, when it returns `None`. The listener takes its connection
/// permit there, so a peer must hold the token to take one, and an origin
/// refused for want of room sees a rejected handshake (#4065).
pub(crate) async fn auth_gate_then<T>(
    stream: &mut tokio::net::TcpStream,
    timeout: std::time::Duration,
    admit: impl FnOnce() -> Option<T>,
) -> Result<Option<T>, String> {
    match tokio::time::timeout(timeout, auth_handshake_check(stream)).await {
        Ok(Ok(())) => {}
        Ok(Err(e)) => return Err(format!("auth handshake rejected: {e}")),
        Err(_) => return Err("auth handshake timed out".to_string()),
    }
    let Some(admitted) = admit() else {
        // Best effort, like the other rejections: the close follows anyway.
        let _ = auth_handshake_reply(stream, AUTH_FAIL).await;
        return Ok(None);
    };
    auth_handshake_reply(stream, AUTH_OK).await?;
    Ok(Some(admitted))
}
