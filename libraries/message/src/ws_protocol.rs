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
/// Uses `#[serde(untagged)]` which tries variants in order.
/// Discriminating fields (order matters):
/// 1. `Request` — matched first because it has a required `method` field
/// 2. `Response` — has `id` + optional `result`/`error` (no `method`)
/// 3. `Event` — has `event` field, no `id` or `method`
///
/// A message with both `method` and `result` would match `Request`.
/// This is correct because responses never have a `method` field.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum WsMessage {
    Request(WsRequest),
    Response(WsResponse),
    Event(WsEvent),
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn ws_request_roundtrip() {
        let id = Uuid::new_v4();
        let req = WsRequest {
            id,
            method: "control".into(),
            params: json!({"foo": 42}),
        };
        let json = serde_json::to_string(&req).unwrap();
        let decoded: WsRequest = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.id, id);
        assert_eq!(decoded.method, "control");
        assert_eq!(decoded.params, json!({"foo": 42}));
    }

    #[test]
    fn ws_response_ok_roundtrip() {
        let id = Uuid::new_v4();
        let resp = WsResponse::ok(id, json!({"status": "running"}));
        let json = serde_json::to_string(&resp).unwrap();
        let decoded: WsResponse = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.id, id);
        assert_eq!(decoded.result, Some(json!({"status": "running"})));
        assert!(decoded.error.is_none());
    }

    #[test]
    fn ws_response_err_roundtrip() {
        let id = Uuid::new_v4();
        let resp = WsResponse::err(id, "something failed".into());
        let json = serde_json::to_string(&resp).unwrap();
        let decoded: WsResponse = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.id, id);
        assert!(decoded.result.is_none());
        assert_eq!(decoded.error.as_deref(), Some("something failed"));
    }

    #[test]
    fn ws_event_roundtrip() {
        let evt = WsEvent {
            event: "log".into(),
            payload: json!({"line": "hello"}),
        };
        let json = serde_json::to_string(&evt).unwrap();
        let decoded: WsEvent = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.event, "log");
        assert_eq!(decoded.payload, json!({"line": "hello"}));
    }

    #[test]
    fn ws_message_dispatches_to_response() {
        let id = Uuid::new_v4();
        let json = json!({"id": id, "result": {"ok": true}}).to_string();
        let msg: WsMessage = serde_json::from_str(&json).unwrap();
        assert!(matches!(msg, WsMessage::Response(_)));
    }

    #[test]
    fn ws_message_dispatches_to_event() {
        let json = json!({"event": "log", "payload": {}}).to_string();
        let msg: WsMessage = serde_json::from_str(&json).unwrap();
        assert!(matches!(msg, WsMessage::Event(_)));
    }

    #[test]
    fn ws_response_ok_helper() {
        let id = Uuid::new_v4();
        let resp = WsResponse::ok(id, json!("done"));
        assert_eq!(resp.result, Some(json!("done")));
        assert!(resp.error.is_none());
    }

    #[test]
    fn ws_response_err_helper() {
        let id = Uuid::new_v4();
        let resp = WsResponse::err(id, "bad".into());
        assert!(resp.result.is_none());
        assert_eq!(resp.error.as_deref(), Some("bad"));
    }

    #[test]
    fn ws_request_nil_id_rejected() {
        // Malformed JSON (missing required fields) -> serde error
        let bad = r#"{"id": null}"#;
        assert!(serde_json::from_str::<WsRequest>(bad).is_err());
    }

    #[test]
    fn ws_message_unknown_shape() {
        // JSON that matches neither Request, Response, nor Event
        let bad = json!({"random": "stuff"}).to_string();
        assert!(serde_json::from_str::<WsMessage>(&bad).is_err());
    }
}
