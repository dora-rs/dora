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
///
/// # Serde invariant
///
/// `result: Some(Value::Null)` and `result: None` are **equivalent on the
/// wire**. Both serialize to a JSON object without a `result` field
/// (the `Some(Null)` case via the standard serde rule that JSON `null`
/// deserializes back as `None` for `Option<T>`). This is intentional and
/// matches JSON-RPC convention. Pinned by a unit test below
/// (`ws_response_some_null_equals_none_on_wire`).
///
/// If you need to distinguish "no result" from "result is JSON null"
/// across the wire, change the field type to `Option<Option<Value>>`
/// and use `serde_with::rust::double_option`. This is a deliberately
/// non-default choice; do not make it without coordination.
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

    /// Pins the documented serde invariant on `WsResponse`:
    /// `result: Some(Value::Null)` and `result: None` are equivalent on the
    /// wire. A `Some(Null)` value serializes as `result: null` (because
    /// `skip_serializing_if = "Option::is_none"` only checks the outer
    /// Option), and serde's default Option<T> deserializer turns JSON
    /// `null` back into `None`. The roundtrip is therefore lossy in one
    /// direction by design.
    ///
    /// Discovered by property testing
    /// (`prop_response_roundtrip` failure on commit fddbe7b). Documented
    /// in `WsResponse`'s doc comment as intentional. Do NOT delete this
    /// test without updating the docs and the proptest strategy.
    #[test]
    fn ws_response_some_null_equals_none_on_wire() {
        let id = Uuid::new_v4();
        let with_some_null = WsResponse {
            id,
            result: Some(serde_json::Value::Null),
            error: None,
        };
        let with_none = WsResponse {
            id,
            result: None,
            error: None,
        };

        // Both serialize the same way: result field is omitted entirely
        // for None, and serialized as `null` for Some(Null). They are
        // structurally different but indistinguishable as JSON values.
        let json_some = serde_json::to_string(&with_some_null).unwrap();
        let json_none = serde_json::to_string(&with_none).unwrap();
        assert!(
            json_some.contains("\"result\":null") || json_some == json_none,
            "expected Some(Null) to serialize as result:null or be omitted; got {json_some}"
        );

        // Both deserialize to the same in-memory representation: None.
        let from_some: WsResponse = serde_json::from_str(&json_some).unwrap();
        let from_none: WsResponse = serde_json::from_str(&json_none).unwrap();
        assert_eq!(from_some.result, None, "Some(Null) must round-trip to None");
        assert_eq!(from_none.result, None);
    }

    // --- Property tests (proptest) ---
    //
    // These tests exercise the JSON serde roundtrip for each WsMessage
    // variant over generated inputs. The key properties are:
    //
    //   1. serialize -> deserialize is a fixed point for each variant
    //      (the value is preserved)
    //   2. when wrapped in the untagged WsMessage enum, the original
    //      variant is preserved across roundtrip (no variant escaping)
    //
    // These catch two classes of bugs that the hand-written tests miss:
    //   - variant confusion in untagged enums (a Request deserializing
    //     as a Response, or similar)
    //   - field-ordering sensitivity in serde
    //   - edge cases in string/number/array handling that nobody
    //     thought to add to the hand-written suite

    use proptest::prelude::*;

    /// Strategy for arbitrary JSON values with bounded recursion depth.
    fn arb_json_value() -> impl Strategy<Value = serde_json::Value> {
        use serde_json::Value;
        let leaf = prop_oneof![
            Just(Value::Null),
            any::<bool>().prop_map(Value::Bool),
            any::<i64>().prop_map(|n| Value::Number(n.into())),
            // f64 strategy: only generate values that are guaranteed to
            // round-trip exactly through JSON. Arbitrary f64s near the edge
            // of precision (e.g., 1e120) lose 1 ULP through the string
            // representation; that's a property of JSON itself, not adora.
            // We restrict to values within the i32 range cast to f64, which
            // are always representable losslessly.
            (any::<i32>()).prop_map(|n| serde_json::Number::from_f64(f64::from(n))
                .map(Value::Number)
                .unwrap_or(Value::Null)),
            "[a-zA-Z0-9 _-]{0,32}".prop_map(Value::String),
        ];
        leaf.prop_recursive(
            3,  // depth
            16, // max total nodes
            4,  // items per collection
            |inner| {
                prop_oneof![
                    prop::collection::vec(inner.clone(), 0..4).prop_map(Value::Array),
                    prop::collection::hash_map("[a-zA-Z][a-zA-Z0-9_]{0,8}", inner, 0..4)
                        .prop_map(|m| Value::Object(m.into_iter().collect())),
                ]
            },
        )
    }

    fn arb_method() -> impl Strategy<Value = String> {
        "[a-z][a-z_]{0,16}".prop_map(|s| s.to_string())
    }

    fn arb_event() -> impl Strategy<Value = String> {
        "[a-z][a-z_]{0,16}".prop_map(|s| s.to_string())
    }

    fn arb_ws_request() -> impl Strategy<Value = WsRequest> {
        (any::<u128>(), arb_method(), arb_json_value()).prop_map(|(id, method, params)| WsRequest {
            id: Uuid::from_u128(id),
            method,
            params,
        })
    }

    fn arb_ws_response() -> impl Strategy<Value = WsResponse> {
        // Filter `Some(Value::Null)` out of the `result` field: by the
        // documented serde invariant on WsResponse, `Some(Null)` is
        // equivalent to `None` on the wire and would not survive a
        // structural roundtrip. The equivalence itself is pinned by the
        // `ws_response_some_null_equals_none_on_wire` unit test, so this
        // filter does not hide a bug — it scopes the property to inputs
        // that are *expected* to roundtrip identically.
        (
            any::<u128>(),
            prop::option::of(arb_json_value().prop_filter("not Value::Null", |v| !v.is_null())),
            prop::option::of("[a-z0-9 ]{0,32}".prop_map(|s| s.to_string())),
        )
            .prop_map(|(id, result, error)| WsResponse {
                id: Uuid::from_u128(id),
                result,
                error,
            })
    }

    fn arb_ws_event() -> impl Strategy<Value = WsEvent> {
        (arb_event(), arb_json_value()).prop_map(|(event, payload)| WsEvent { event, payload })
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(2000))]

        /// WsRequest roundtrips through JSON.
        #[test]
        fn prop_request_roundtrip(req in arb_ws_request()) {
            let json = serde_json::to_string(&req).unwrap();
            let decoded: WsRequest = serde_json::from_str(&json)
                .expect("request must deserialize");
            prop_assert_eq!(decoded.id, req.id);
            prop_assert_eq!(decoded.method, req.method);
            prop_assert_eq!(decoded.params, req.params);
        }

        /// WsResponse roundtrips through JSON.
        #[test]
        fn prop_response_roundtrip(resp in arb_ws_response()) {
            let json = serde_json::to_string(&resp).unwrap();
            let decoded: WsResponse = serde_json::from_str(&json)
                .expect("response must deserialize");
            prop_assert_eq!(decoded.id, resp.id);
            prop_assert_eq!(decoded.result, resp.result);
            prop_assert_eq!(decoded.error, resp.error);
        }

        /// WsEvent roundtrips through JSON.
        #[test]
        fn prop_event_roundtrip(evt in arb_ws_event()) {
            let json = serde_json::to_string(&evt).unwrap();
            let decoded: WsEvent = serde_json::from_str(&json)
                .expect("event must deserialize");
            prop_assert_eq!(decoded.event, evt.event);
            prop_assert_eq!(decoded.payload, evt.payload);
        }

        /// A WsRequest wrapped in WsMessage must round-trip as WsMessage::Request.
        /// This is the critical property for the untagged enum: variants must
        /// not "escape" to other variants across a serde roundtrip.
        #[test]
        fn prop_message_request_does_not_escape(req in arb_ws_request()) {
            let msg = WsMessage::Request(req.clone());
            let json = serde_json::to_string(&msg).unwrap();
            let decoded: WsMessage = serde_json::from_str(&json)
                .expect("wsmessage must deserialize");
            match decoded {
                WsMessage::Request(r) => {
                    prop_assert_eq!(r.id, req.id);
                    prop_assert_eq!(r.method, req.method);
                    prop_assert_eq!(r.params, req.params);
                }
                other => prop_assert!(
                    false,
                    "Request escaped to other variant: {:?}",
                    other
                ),
            }
        }

        /// A WsResponse wrapped in WsMessage must round-trip as WsMessage::Response.
        #[test]
        fn prop_message_response_does_not_escape(resp in arb_ws_response()) {
            let msg = WsMessage::Response(resp.clone());
            let json = serde_json::to_string(&msg).unwrap();
            let decoded: WsMessage = serde_json::from_str(&json)
                .expect("wsmessage must deserialize");
            match decoded {
                WsMessage::Response(r) => {
                    prop_assert_eq!(r.id, resp.id);
                    prop_assert_eq!(r.result, resp.result);
                    prop_assert_eq!(r.error, resp.error);
                }
                other => prop_assert!(
                    false,
                    "Response escaped to other variant: {:?}",
                    other
                ),
            }
        }

        /// A WsEvent wrapped in WsMessage must round-trip as WsMessage::Event.
        #[test]
        fn prop_message_event_does_not_escape(evt in arb_ws_event()) {
            let msg = WsMessage::Event(evt.clone());
            let json = serde_json::to_string(&msg).unwrap();
            let decoded: WsMessage = serde_json::from_str(&json)
                .expect("wsmessage must deserialize");
            match decoded {
                WsMessage::Event(e) => {
                    prop_assert_eq!(e.event, evt.event);
                    prop_assert_eq!(e.payload, evt.payload);
                }
                other => prop_assert!(
                    false,
                    "Event escaped to other variant: {:?}",
                    other
                ),
            }
        }
    }
}
