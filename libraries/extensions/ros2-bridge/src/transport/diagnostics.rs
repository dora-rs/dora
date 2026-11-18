//! Diagnostics for the DDS transport: misconfigurations that otherwise make
//! dora and ROS 2 nodes silently stop talking (#4205 items 8 and 9).

use crate::rustdds::{
    QosPolicies,
    qos::{QosPolicyId, policy},
};

/// Why an offered QoS (a writer's) cannot serve a requested one (a reader's),
/// or `None` when they match. The rule is RustDDS's own matching check, so the
/// diagnosis cannot disagree with the actual matching; this only names the
/// first offending policy with both values.
pub fn qos_mismatch(offered: &QosPolicies, requested: &QosPolicies) -> Option<String> {
    let id = offered.compliance_failure_wrt(requested)?;
    let detail = match id {
        QosPolicyId::Reliability => {
            let name = |r: Option<policy::Reliability>| match r {
                Some(policy::Reliability::Reliable { .. }) => "RELIABLE",
                _ => "BEST_EFFORT",
            };
            Some((name(requested.reliability()), name(offered.reliability())))
        }
        QosPolicyId::Durability => {
            let name =
                |d: Option<policy::Durability>| match d.unwrap_or(policy::Durability::Volatile) {
                    policy::Durability::Volatile => "VOLATILE",
                    policy::Durability::TransientLocal => "TRANSIENT_LOCAL",
                    policy::Durability::Transient => "TRANSIENT",
                    policy::Durability::Persistent => "PERSISTENT",
                };
            Some((name(requested.durability()), name(offered.durability())))
        }
        _ => None,
    };
    let policy = format!("{id:?}").to_lowercase();
    Some(match detail {
        Some((requested, offered)) => {
            format!("{policy}: the reader requests {requested}, the writer offers {offered}")
        }
        None => format!(
            "{policy} (the reader requests {:?}, the writer offers {:?})",
            policy_value(requested, id),
            policy_value(offered, id)
        ),
    })
}

/// The raw value of one policy, for the less common mismatches.
fn policy_value(qos: &QosPolicies, id: QosPolicyId) -> String {
    match id {
        QosPolicyId::Deadline => format!("{:?}", qos.deadline()),
        QosPolicyId::Liveliness => format!("{:?}", qos.liveliness()),
        QosPolicyId::Ownership => format!("{:?}", qos.ownership()),
        _ => "-".to_owned(),
    }
}

/// The ROS 2 RMW a remote DDS participant most likely runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PeerRmw {
    FastDds,
    CycloneDds,
}

/// Guess a remote endpoint's RMW from its GUID. RTPS implementations put their
/// vendor id in the first two bytes of the GUID prefix: eProsima (Fast-DDS)
/// uses 01.0f, Eclipse (CycloneDDS) 01.10.
pub fn peer_rmw(guid: &crate::rustdds::GUID) -> Option<PeerRmw> {
    match guid.to_bytes()[..2] {
        [0x01, 0x0f] => Some(PeerRmw::FastDds),
        [0x01, 0x10] => Some(PeerRmw::CycloneDds),
        _ => None,
    }
}

/// Whether a DDS topic carries ROS 2 service requests or replies (services and
/// the action goal/result/cancel services), whose framing depends on the
/// service mapping.
pub fn is_service_topic(dds_topic: &str) -> bool {
    dds_topic.starts_with("rq/") || dds_topic.starts_with("rr/")
}

/// The fix when dora's service mapping cannot talk to a peer's RMW: both sides
/// see each other, but every request or reply fails to decode, so calls hang.
pub fn mapping_mismatch(
    mapping: crate::ros2_client::ServiceMapping,
    peer: PeerRmw,
) -> Option<String> {
    use crate::ros2_client::ServiceMapping;
    let (peer_name, rmw) = match (mapping, peer) {
        (ServiceMapping::Cyclone, PeerRmw::FastDds) => ("Fast-DDS", "rmw_fastrtps_cpp"),
        (ServiceMapping::Enhanced | ServiceMapping::Basic, PeerRmw::CycloneDds) => {
            ("CycloneDDS", "rmw_cyclonedds_cpp")
        }
        _ => return None,
    };
    Some(format!(
        "ROS 2 services/actions here are served by {peer_name}, but dora frames them for \
         {mapping:?}; calls between them will hang. Set RMW_IMPLEMENTATION={rmw} for dora too"
    ))
}

/// Set to `1` to run a DDS context even though `ROS_DISTRO` needs a different
/// Gid layout than the build.
pub const ALLOW_DISTRO_MISMATCH_ENV: &str = "DORA_ROS2_ALLOW_DISTRO_MISMATCH";

/// Outcome of comparing the build's ROS distro with `ROS_DISTRO`.
#[derive(Debug, PartialEq, Eq)]
pub enum DistroCheck {
    Ok,
    /// Possibly compatible; worth a warning.
    Warn(String),
    /// Different Gid layout: graph discovery cannot work.
    Mismatch(String),
}

/// The `rmw_dds_common` Gid width a distro uses: 24 bytes up to Humble, 16
/// from Iron on. `None` for a distro this check does not know.
fn gid_width(distro: &str) -> Option<u8> {
    match distro {
        "galactic" | "humble" => Some(24),
        "iron" | "jazzy" | "kilted" | "lyrical" => Some(16),
        _ => None,
    }
}

/// Compare the distro the bridge was built for with `ROS_DISTRO`. Only a
/// different Gid width is a hard mismatch: it is what makes graph discovery
/// (and with it services) fail between the two sides.
pub fn check_ros_distro(built: &str, env: Option<&str>) -> DistroCheck {
    let Some(env) = env else {
        return DistroCheck::Warn(format!(
            "ROS_DISTRO is not set; the ROS 2 bridge was built for '{built}'"
        ));
    };
    if env == built {
        return DistroCheck::Ok;
    }
    match (gid_width(built), gid_width(env)) {
        (Some(b), Some(e)) if b != e => DistroCheck::Mismatch(format!(
            "ROS_DISTRO='{env}' but the ROS 2 bridge was built for '{built}': they use \
             different DDS graph identifiers ({e}-byte vs {b}-byte Gid), so ROS 2 nodes \
             cannot discover dora. Rebuild the bridge with `--features ros2-{env}`, or set \
             {ALLOW_DISTRO_MISMATCH_ENV}=1 to run anyway"
        )),
        _ => DistroCheck::Warn(format!(
            "ROS_DISTRO='{env}' but the ROS 2 bridge was built for '{built}'; \
             they may not interoperate fully"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rustdds::{self, QosPolicyBuilder, policy};

    #[test]
    fn same_distro_is_ok() {
        assert_eq!(check_ros_distro("humble", Some("humble")), DistroCheck::Ok);
        assert_eq!(check_ros_distro("kilted", Some("kilted")), DistroCheck::Ok);
    }

    #[test]
    fn a_different_gid_width_is_a_mismatch() {
        // Humble/Galactic use the 24-byte Gid, Iron and later the 16-byte one.
        assert!(matches!(
            check_ros_distro("humble", Some("jazzy")),
            DistroCheck::Mismatch(_)
        ));
        assert!(matches!(
            check_ros_distro("kilted", Some("humble")),
            DistroCheck::Mismatch(_)
        ));
        assert!(matches!(
            check_ros_distro("galactic", Some("iron")),
            DistroCheck::Mismatch(_)
        ));
    }

    #[test]
    fn the_same_gid_width_only_warns() {
        assert!(matches!(
            check_ros_distro("jazzy", Some("kilted")),
            DistroCheck::Warn(_)
        ));
        assert!(matches!(
            check_ros_distro("humble", Some("galactic")),
            DistroCheck::Warn(_)
        ));
    }

    #[test]
    fn unset_rolling_or_unknown_only_warn() {
        assert!(matches!(
            check_ros_distro("humble", None),
            DistroCheck::Warn(_)
        ));
        assert!(matches!(
            check_ros_distro("humble", Some("rolling")),
            DistroCheck::Warn(_)
        ));
        assert!(matches!(
            check_ros_distro("humble", Some("foxy")),
            DistroCheck::Warn(_)
        ));
    }

    fn qos(f: impl FnOnce(QosPolicyBuilder) -> QosPolicyBuilder) -> QosPolicies {
        f(QosPolicyBuilder::new()).build()
    }

    #[test]
    fn compatible_qos_has_no_mismatch() {
        let reliable = |b: QosPolicyBuilder| {
            b.reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::ZERO,
            })
        };
        assert_eq!(qos_mismatch(&qos(reliable), &qos(reliable)), None);
        // A stronger offer than requested is fine.
        assert_eq!(qos_mismatch(&qos(reliable), &qos(|b| b)), None);
    }

    #[test]
    fn best_effort_offer_against_reliable_request_names_reliability() {
        let offered = qos(|b| b.reliability(policy::Reliability::BestEffort));
        let requested = qos(|b| {
            b.reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::ZERO,
            })
        });
        let message = qos_mismatch(&offered, &requested).expect("incompatible");
        assert!(message.contains("reliability"), "{message}");
        assert!(message.contains("requests RELIABLE"), "{message}");
        assert!(message.contains("offers BEST_EFFORT"), "{message}");
    }

    #[test]
    fn volatile_offer_against_transient_local_request_names_durability() {
        let offered = qos(|b| b.durability(policy::Durability::Volatile));
        let requested = qos(|b| b.durability(policy::Durability::TransientLocal));
        let message = qos_mismatch(&offered, &requested).expect("incompatible");
        assert!(message.contains("durability"), "{message}");
        assert!(message.contains("requests TRANSIENT_LOCAL"), "{message}");
        assert!(message.contains("offers VOLATILE"), "{message}");
    }

    #[test]
    fn a_longer_offered_deadline_names_deadline() {
        let deadline = |secs| {
            move |b: QosPolicyBuilder| {
                b.deadline(policy::Deadline(rustdds::Duration::from_secs(secs)))
            }
        };
        let message = qos_mismatch(&qos(deadline(2)), &qos(deadline(1))).expect("incompatible");
        assert!(message.contains("deadline"), "{message}");
    }

    fn guid_from_vendor(vendor: [u8; 2]) -> rustdds::GUID {
        let mut bytes = [0x2a_u8; 16];
        bytes[..2].copy_from_slice(&vendor);
        rustdds::GUID::from_bytes(bytes)
    }

    #[test]
    fn the_guid_prefix_names_the_peer_rmw() {
        assert_eq!(
            peer_rmw(&guid_from_vendor([0x01, 0x0f])),
            Some(PeerRmw::FastDds)
        );
        assert_eq!(
            peer_rmw(&guid_from_vendor([0x01, 0x10])),
            Some(PeerRmw::CycloneDds)
        );
        assert_eq!(peer_rmw(&guid_from_vendor([0x01, 0x12])), None);
    }

    #[test]
    fn a_mapping_that_does_not_fit_the_peer_names_the_rmw_to_set() {
        use crate::ros2_client::ServiceMapping;
        let message =
            mapping_mismatch(ServiceMapping::Enhanced, PeerRmw::CycloneDds).expect("mismatch");
        assert!(
            message.contains("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"),
            "{message}"
        );
        let message =
            mapping_mismatch(ServiceMapping::Cyclone, PeerRmw::FastDds).expect("mismatch");
        assert!(
            message.contains("RMW_IMPLEMENTATION=rmw_fastrtps_cpp"),
            "{message}"
        );
        assert_eq!(
            mapping_mismatch(ServiceMapping::Enhanced, PeerRmw::FastDds),
            None
        );
        assert_eq!(
            mapping_mismatch(ServiceMapping::Cyclone, PeerRmw::CycloneDds),
            None
        );
    }

    #[test]
    fn only_request_and_reply_topics_carry_the_service_mapping() {
        assert!(is_service_topic("rq/add_two_intsRequest"));
        assert!(is_service_topic("rr/fibonacci/_action/send_goalReply"));
        assert!(!is_service_topic("rt/chatter"));
        assert!(!is_service_topic("rt/fibonacci/_action/status"));
    }

    #[test]
    fn the_mismatch_message_names_the_fix() {
        let DistroCheck::Mismatch(message) = check_ros_distro("humble", Some("jazzy")) else {
            panic!("expected a mismatch");
        };
        assert!(message.contains("ros2-jazzy"), "{message}");
        assert!(message.contains(ALLOW_DISTRO_MISMATCH_ENV), "{message}");
    }
}
