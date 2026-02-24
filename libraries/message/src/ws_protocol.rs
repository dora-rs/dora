//! WebSocket message protocol for the control plane.
//!
//! Three message types over JSON text frames:
//!
//! - **Request** (client -> server): `{"id": "uuid", "method": "...", "params": {...}}`
//! - **Response** (server -> client): `{"id": "uuid", "result": {...}}` or `{"id": "uuid", "error": "..."}`
//! - **Event** (either direction): `{"event": "...", "payload": {...}}`

use serde::{Deserialize, Serialize};
use serde_json::Value;
use uuid::Uuid;

/// A request from client to server, expecting a response with the same `id`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WsRequest {
    pub id: Uuid,
    pub method: String,
    pub params: Value,
}

/// A response from server to client, matching a request `id`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WsResponse {
    pub id: Uuid,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

impl WsResponse {
    pub fn ok(id: Uuid, result: Value) -> Self {
        Self {
            id,
            result: Some(result),
            error: None,
        }
    }

    pub fn err(id: Uuid, error: String) -> Self {
        Self {
            id,
            result: None,
            error: Some(error),
        }
    }
}

/// A fire-and-forget event in either direction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WsEvent {
    pub event: String,
    pub payload: Value,
}

/// Discriminated union for parsing any incoming WS text frame.
///
/// We distinguish by the presence of fields:
/// - Has `method` -> Request
/// - Has `result` or `error` (with `id`) -> Response
/// - Has `event` -> Event
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum WsMessage {
    Request(WsRequest),
    Response(WsResponse),
    Event(WsEvent),
}
