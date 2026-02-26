use crate::{events::Event, ws_control::handle_control_ws, ws_daemon::handle_daemon_ws};
use adora_core::uhlc::HLC;
use adora_message::auth::AuthToken;
use axum::{
    Router,
    extract::{Query, State, ws::WebSocketUpgrade},
    http::StatusCode,
    response::IntoResponse,
    routing::get,
};
use std::{net::SocketAddr, sync::Arc};
use tokio::{net::TcpListener, sync::mpsc};
use tower::{ServiceBuilder, limit::ConcurrencyLimitLayer};

/// Maximum message size for control plane JSON messages (1 MiB).
const MAX_CONTROL_MESSAGE_BYTES: usize = 1024 * 1024;

/// Maximum concurrent WebSocket connections.
const MAX_WS_CONNECTIONS: usize = 256;

#[derive(Clone)]
pub(crate) struct WsState {
    pub event_tx: mpsc::Sender<Event>,
    pub clock: Arc<HLC>,
    pub auth_token: Option<AuthToken>,
}

#[derive(serde::Deserialize)]
struct TokenQuery {
    #[serde(default)]
    token: Option<String>,
}

/// Validate the token from query params against the expected token.
/// Returns `Ok(())` if auth is disabled or token matches.
fn validate_token(
    expected: &Option<AuthToken>,
    provided: &Option<String>,
) -> Result<(), StatusCode> {
    let Some(expected) = expected else {
        return Ok(()); // auth disabled
    };
    match provided {
        Some(t) if t == expected.as_hex() => Ok(()),
        _ => {
            tracing::warn!("rejected WebSocket connection: invalid or missing auth token");
            Err(StatusCode::UNAUTHORIZED)
        }
    }
}

/// Build the axum router with WS routes.
pub(crate) fn router(state: WsState) -> Router {
    Router::new()
        .route("/api/control", get(ws_control_handler))
        .route("/api/daemon", get(ws_daemon_handler))
        .route("/health", get(health))
        .with_state(Arc::new(state))
        .layer(ServiceBuilder::new().layer(ConcurrencyLimitLayer::new(MAX_WS_CONNECTIONS)))
}

async fn health() -> &'static str {
    "ok"
}

async fn ws_control_handler(
    State(state): State<Arc<WsState>>,
    Query(query): Query<TokenQuery>,
    ws: WebSocketUpgrade,
) -> Result<impl IntoResponse, StatusCode> {
    validate_token(&state.auth_token, &query.token)?;
    Ok(ws
        .max_message_size(MAX_CONTROL_MESSAGE_BYTES)
        .on_upgrade(move |socket| handle_control_ws(socket, state.event_tx.clone())))
}

async fn ws_daemon_handler(
    State(state): State<Arc<WsState>>,
    Query(query): Query<TokenQuery>,
    ws: WebSocketUpgrade,
) -> Result<impl IntoResponse, StatusCode> {
    validate_token(&state.auth_token, &query.token)?;
    Ok(ws
        .max_message_size(MAX_CONTROL_MESSAGE_BYTES)
        .on_upgrade(move |socket| {
            handle_daemon_ws(socket, state.event_tx.clone(), state.clock.clone())
        }))
}

/// Start the axum WS server. Returns the bound port, a shutdown trigger, and a future to await.
pub(crate) async fn serve(
    bind: SocketAddr,
    event_tx: mpsc::Sender<Event>,
    clock: Arc<HLC>,
    auth_token: Option<AuthToken>,
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
    };
    let app = router(state);
    let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel::<()>();

    let future = async move {
        axum::serve(listener, app)
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
