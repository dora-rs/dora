use crate::{
    artifacts::ArtifactStore, events::Event, ws_control::handle_control_ws,
    ws_daemon::handle_daemon_ws,
};
use adora_core::uhlc::HLC;
use adora_message::auth::AuthToken;
use axum::{
    Router,
    extract::{ConnectInfo, Path, Query, State, ws::WebSocketUpgrade},
    http::{HeaderMap, StatusCode},
    response::IntoResponse,
    routing::get,
};
use std::{
    collections::HashMap,
    net::{IpAddr, SocketAddr},
    sync::{Arc, Mutex},
    time::Instant,
};
use tokio::{net::TcpListener, sync::mpsc};
use tower::{ServiceBuilder, limit::ConcurrencyLimitLayer};

/// Maximum message size for control plane JSON messages (1 MiB).
const MAX_CONTROL_MESSAGE_BYTES: usize = 1024 * 1024;

/// Maximum concurrent WebSocket connections.
const MAX_WS_CONNECTIONS: usize = 256;

/// Maximum new connections per IP within the rate window.
const MAX_CONNECTIONS_PER_IP: u32 = 20;

/// Rate-limiting window duration (60 seconds).
const RATE_WINDOW_SECS: u64 = 60;

/// Fixed-window per-IP rate limiter for WebSocket connections.
#[derive(Clone)]
pub(crate) struct IpRateLimiter {
    windows: Arc<Mutex<HashMap<IpAddr, (u32, Instant)>>>,
}

impl IpRateLimiter {
    pub fn new() -> Self {
        Self {
            windows: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Returns `true` if the connection should be allowed.
    pub fn check(&self, ip: IpAddr) -> bool {
        let mut map = self.windows.lock().unwrap_or_else(|e| e.into_inner());
        let now = Instant::now();
        let window = std::time::Duration::from_secs(RATE_WINDOW_SECS);

        let entry = map.entry(ip).or_insert((0, now));
        if now.duration_since(entry.1) >= window {
            // Reset window
            *entry = (1, now);
            true
        } else if entry.0 < MAX_CONNECTIONS_PER_IP {
            entry.0 += 1;
            true
        } else {
            false
        }
    }
}

#[derive(Clone)]
pub(crate) struct WsState {
    pub event_tx: mpsc::Sender<Event>,
    pub clock: Arc<HLC>,
    pub auth_token: Option<AuthToken>,
    pub artifact_store: Arc<ArtifactStore>,
    pub rate_limiter: IpRateLimiter,
}

/// Query parameters for backward compatibility — old clients may send `?token=...`.
/// The token field is deserialized but intentionally ignored; auth is via Bearer header only.
#[derive(serde::Deserialize)]
pub(crate) struct TokenQuery {
    #[serde(default)]
    #[allow(dead_code)]
    token: Option<String>,
}

/// Extract the auth token from the Authorization header ("Bearer <hex>").
pub(crate) fn extract_token(headers: &HeaderMap) -> Option<String> {
    if let Some(auth_header) = headers.get("authorization") {
        if let Ok(value) = auth_header.to_str() {
            if let Some(token) = value.strip_prefix("Bearer ") {
                return Some(token.to_string());
            }
        }
    }
    None
}

/// Validate the provided token against the expected token.
/// Returns `Ok(())` if auth is disabled or token matches.
pub(crate) fn validate_token(
    expected: &Option<AuthToken>,
    provided: &Option<String>,
) -> Result<(), StatusCode> {
    let Some(expected) = expected else {
        return Ok(()); // auth disabled
    };
    match provided {
        Some(t)
            if adora_message::auth::constant_time_eq(
                t.as_bytes(),
                expected.as_hex().as_bytes(),
            ) =>
        {
            Ok(())
        }
        _ => {
            tracing::warn!("rejected WebSocket connection: invalid or missing auth token");
            Err(StatusCode::UNAUTHORIZED)
        }
    }
}

/// Build the axum router with WS routes.
pub(crate) fn router(
    state: WsState,
    #[cfg(feature = "prometheus")] prometheus: crate::prometheus_metrics::SharedMetrics,
) -> Router {
    #[cfg(feature = "prometheus")]
    let prom_auth = state.auth_token.clone();

    let app = Router::new()
        .route("/api/control", get(ws_control_handler))
        .route("/api/daemon", get(ws_daemon_handler))
        .route("/api/artifacts/{build_id}/{node_id}", get(artifact_handler))
        .route("/health", get(health))
        .with_state(Arc::new(state))
        .layer(ServiceBuilder::new().layer(ConcurrencyLimitLayer::new(MAX_WS_CONNECTIONS)));

    #[cfg(feature = "prometheus")]
    let app = app.route(
        "/metrics",
        get(crate::prometheus_metrics::metrics_handler).with_state((prometheus, prom_auth)),
    );

    app
}

async fn health() -> &'static str {
    "ok"
}

async fn ws_control_handler(
    State(state): State<Arc<WsState>>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Query(_query): Query<TokenQuery>,
    ws: WebSocketUpgrade,
) -> Result<impl IntoResponse, StatusCode> {
    if !state.rate_limiter.check(addr.ip()) {
        tracing::warn!("rate-limited WS control connection from {}", addr.ip());
        return Err(StatusCode::TOO_MANY_REQUESTS);
    }
    let token = extract_token(&headers);
    validate_token(&state.auth_token, &token)?;
    Ok(ws
        .max_message_size(MAX_CONTROL_MESSAGE_BYTES)
        .on_upgrade(move |socket| {
            handle_control_ws(socket, state.event_tx.clone(), state.clock.clone())
        }))
}

async fn ws_daemon_handler(
    State(state): State<Arc<WsState>>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Query(_query): Query<TokenQuery>,
    ws: WebSocketUpgrade,
) -> Result<impl IntoResponse, StatusCode> {
    if !state.rate_limiter.check(addr.ip()) {
        tracing::warn!("rate-limited WS daemon connection from {}", addr.ip());
        return Err(StatusCode::TOO_MANY_REQUESTS);
    }
    let token = extract_token(&headers);
    validate_token(&state.auth_token, &token)?;
    Ok(ws
        .max_message_size(MAX_CONTROL_MESSAGE_BYTES)
        .on_upgrade(move |socket| {
            handle_daemon_ws(socket, state.event_tx.clone(), state.clock.clone())
        }))
}

async fn artifact_handler(
    State(state): State<Arc<WsState>>,
    ConnectInfo(addr): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
    Query(_query): Query<TokenQuery>,
    Path((build_id, node_id)): Path<(String, String)>,
) -> Result<impl IntoResponse, StatusCode> {
    if !state.rate_limiter.check(addr.ip()) {
        return Err(StatusCode::TOO_MANY_REQUESTS);
    }
    let token = extract_token(&headers);
    validate_token(&state.auth_token, &token)?;

    let build_uuid: uuid::Uuid = build_id.parse().map_err(|_| StatusCode::BAD_REQUEST)?;

    let path = state
        .artifact_store
        .artifact_path(&build_uuid, &node_id)
        .ok_or(StatusCode::BAD_REQUEST)?;

    let data = tokio::fs::read(&path).await.map_err(|e| {
        if e.kind() == std::io::ErrorKind::NotFound {
            StatusCode::NOT_FOUND
        } else {
            StatusCode::INTERNAL_SERVER_ERROR
        }
    })?;

    Ok((
        [(axum::http::header::CONTENT_TYPE, "application/octet-stream")],
        data,
    ))
}

/// Start the axum WS server. Returns the bound port, a shutdown trigger, and a future to await.
pub(crate) async fn serve(
    bind: SocketAddr,
    event_tx: mpsc::Sender<Event>,
    clock: Arc<HLC>,
    auth_token: Option<AuthToken>,
    artifact_store: Arc<ArtifactStore>,
    #[cfg(feature = "prometheus")] prometheus: crate::prometheus_metrics::SharedMetrics,
) -> eyre::Result<(
    u16,
    ShutdownTrigger,
    impl std::future::Future<Output = eyre::Result<()>>,
)> {
    let listener = TcpListener::bind(bind).await?;
    let port = listener.local_addr()?.port();
    let state = WsState {
        event_tx,
        clock,
        auth_token,
        artifact_store,
        rate_limiter: IpRateLimiter::new(),
    };
    let app = router(
        state,
        #[cfg(feature = "prometheus")]
        prometheus,
    );
    let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel::<()>();

    let future = async move {
        axum::serve(
            listener,
            app.into_make_service_with_connect_info::<SocketAddr>(),
        )
        .with_graceful_shutdown(async {
            let _ = shutdown_rx.await;
        })
        .await
        .map_err(|e| eyre::eyre!("axum server error: {e}"))?;
        Ok(())
    };

    Ok((port, ShutdownTrigger(shutdown_tx), future))
}

/// Trigger for graceful WS server shutdown.
pub(crate) struct ShutdownTrigger(tokio::sync::oneshot::Sender<()>);

impl ShutdownTrigger {
    pub(crate) fn shutdown(self) {
        let _ = self.0.send(());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rate_limiter_allows_within_limit() {
        let rl = IpRateLimiter::new();
        let ip: IpAddr = "10.0.0.1".parse().unwrap();
        for _ in 0..MAX_CONNECTIONS_PER_IP {
            assert!(rl.check(ip));
        }
    }

    #[test]
    fn rate_limiter_rejects_over_limit() {
        let rl = IpRateLimiter::new();
        let ip: IpAddr = "10.0.0.2".parse().unwrap();
        for _ in 0..MAX_CONNECTIONS_PER_IP {
            rl.check(ip);
        }
        assert!(!rl.check(ip));
    }

    #[test]
    fn rate_limiter_isolates_ips() {
        let rl = IpRateLimiter::new();
        let ip1: IpAddr = "10.0.0.3".parse().unwrap();
        let ip2: IpAddr = "10.0.0.4".parse().unwrap();
        for _ in 0..MAX_CONNECTIONS_PER_IP {
            rl.check(ip1);
        }
        assert!(!rl.check(ip1));
        assert!(rl.check(ip2));
    }
}
