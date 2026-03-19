use dora_message::daemon_to_coordinator::{StateGetRequest, StateSetRequest};

#[test]
fn state_get_request_roundtrip_json() {
    let request = StateGetRequest {
        namespace: "runtime".to_owned(),
        key: "session/build-123".to_owned(),
    };

    let encoded = serde_json::to_vec(&request).expect("serialize StateGetRequest");
    let decoded: StateGetRequest =
        serde_json::from_slice(&encoded).expect("deserialize StateGetRequest");

    assert_eq!(decoded, request);
}

#[test]
fn state_set_request_roundtrip_json_preserves_value_bytes() {
    let request = StateSetRequest {
        namespace: "runtime".to_owned(),
        key: "node/camera-01".to_owned(),
        value: vec![0, 1, 2, 254, 255],
    };

    let encoded = serde_json::to_vec(&request).expect("serialize StateSetRequest");
    let decoded: StateSetRequest =
        serde_json::from_slice(&encoded).expect("deserialize StateSetRequest");

    assert_eq!(decoded, request);
}
