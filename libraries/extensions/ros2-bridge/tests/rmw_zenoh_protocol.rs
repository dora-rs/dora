#![cfg(feature = "rmw-zenoh")]

use dora_ros2_bridge::transport::zenoh::compatibility::{RosTypeIdentity, TypeHash};
use dora_ros2_bridge::transport::{
    History, Reliability, Ros2Qos,
    zenoh::{
        attachment::Attachment,
        keyexpr::{DataKey, EntityKind, LivelinessKey, TopicToken},
        qos::ZenohQosMapping,
    },
};

#[test]
fn data_keys_cover_domains_and_nested_names() {
    let identity = RosTypeIdentity {
        ros_name: "std_msgs/msg/String".into(),
        dds_name: "std_msgs::msg::dds_::String_".into(),
        hash: TypeHash::HumbleUnsupported,
    };
    assert_eq!(
        DataKey::new(0, "/chatter", &identity).unwrap().as_str(),
        "0/chatter/std_msgs::msg::dds_::String_/TypeHashNotSupported"
    );
    assert_eq!(
        DataKey::new(42, "/robot/_hidden/chatter", &identity)
            .unwrap()
            .as_str(),
        "42/robot/_hidden/chatter/std_msgs::msg::dds_::String_/TypeHashNotSupported"
    );
}

#[test]
fn all_liveliness_entity_kinds_round_trip() {
    for kind in [
        EntityKind::Node,
        EntityKind::Publisher,
        EntityKind::Subscription,
        EntityKind::Service,
        EntityKind::Client,
    ] {
        let key = if kind == EntityKind::Node {
            LivelinessKey::node(42, "zid", "nid", "eid", "/", "/robot/ns", "node").unwrap()
        } else {
            LivelinessKey::endpoint(
                42,
                "zid",
                "nid",
                "eid",
                kind,
                "/",
                "/robot/ns",
                "node",
                TopicToken {
                    name: "/chatter".into(),
                    type_name: "std_msgs::msg::dds_::String_".into(),
                    type_hash: "TypeHashNotSupported".into(),
                    qos: "2::,1:,:,:,,,".into(),
                },
            )
            .unwrap()
        };
        assert_eq!(LivelinessKey::parse(key.as_str()).unwrap(), key);
    }
}

#[test]
fn attachment_matches_upstream_serializer_and_rejects_trailing_bytes() {
    let attachment = Attachment {
        sequence_number: 7,
        source_timestamp_ns: 1_725_000_000_000_000_000,
        gid: [0x2a; 16],
    };
    let bytes = attachment.encode().unwrap();
    assert_eq!(
        bytes
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>(),
        include_str!("fixtures/rmw_zenoh/attachment-seq7.hex").trim()
    );
    assert_eq!(Attachment::decode(&bytes).unwrap(), attachment);
    let mut extra = bytes;
    extra.push(0);
    assert!(Attachment::decode(&extra).is_err());
    assert!(Attachment::decode(&[0; 3]).is_err());
}

#[test]
fn attachment_accepts_sequence_and_timestamp_boundaries() {
    for value in [i64::MIN, -1, 0, i64::MAX] {
        let attachment = Attachment {
            sequence_number: value,
            source_timestamp_ns: value,
            gid: [0; 16],
        };
        assert_eq!(
            Attachment::decode(&attachment.encode().unwrap()).unwrap(),
            attachment
        );
    }
}

#[test]
fn qos_default_and_non_default_match_rmw_components() {
    assert_eq!(
        ZenohQosMapping::from_ros_qos(&Ros2Qos::default()).to_string(),
        "2::,1:,:,:,,"
    );
    let qos = Ros2Qos {
        reliability: Reliability::BestEffort,
        history: History::KeepLast { depth: 7 },
        ..Ros2Qos::default()
    };
    assert_eq!(
        ZenohQosMapping::from_ros_qos(&qos).to_string(),
        "2::,7:,:,:,,"
    );
}

#[test]
fn malformed_components_fail_closed() {
    assert!(LivelinessKey::parse("@ros2_lv/0/z/n/i/NN//%/node").is_err());
    assert!(
        DataKey::new(
            0,
            "/bad/**",
            &RosTypeIdentity {
                ros_name: "x/msg/X".into(),
                dds_name: "x::msg::dds_::X_".into(),
                hash: TypeHash::HumbleUnsupported
            }
        )
        .is_err()
    );
}
