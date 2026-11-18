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

/// Which side of a DDS topic an endpoint is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Role {
    Writer,
    Reader,
}

/// One of dora's own DDS endpoints.
#[derive(Clone, Debug, PartialEq)]
pub struct LocalEndpoint {
    pub dds_topic: String,
    pub role: Role,
    pub qos: QosPolicies,
}

/// A DDS endpoint of another participant, as discovery reports it.
#[derive(Clone, Debug)]
pub struct RemoteEndpoint {
    pub guid: crate::rustdds::GUID,
    pub dds_topic: String,
    pub role: Role,
    pub qos: QosPolicies,
}

/// Something to tell the user. `key` identifies it so it is reported once.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Finding {
    pub key: String,
    pub message: String,
}

/// Compare dora's endpoints with the remote ones: incompatible QoS on a shared
/// topic, and peers whose RMW does not fit dora's service mapping on a service
/// topic dora uses.
pub fn scan(
    locals: &[LocalEndpoint],
    remotes: &[RemoteEndpoint],
    mapping: crate::ros2_client::ServiceMapping,
) -> Vec<Finding> {
    let mut findings: Vec<Finding> = Vec::new();
    let mut push = |key: String, message: String| {
        if !findings.iter().any(|f| f.key == key) {
            findings.push(Finding { key, message });
        }
    };
    for local in locals {
        // Only endpoints in the other role would ever talk to this one.
        let peers = remotes
            .iter()
            .filter(|r| r.dds_topic == local.dds_topic && r.role != local.role);
        for remote in peers {
            let (offered, requested) = match local.role {
                Role::Writer => (&local.qos, &remote.qos),
                Role::Reader => (&remote.qos, &local.qos),
            };
            if let Some(detail) = qos_mismatch(offered, requested) {
                let (ours, theirs) = match local.role {
                    Role::Writer => ("publisher", "subscription"),
                    Role::Reader => ("subscription", "publisher"),
                };
                push(
                    format!(
                        "qos {} {:?} {:?} {detail}",
                        local.dds_topic, local.role, remote.guid
                    ),
                    format!(
                        "QoS mismatch on {}: dora {ours} and remote {theirs} {} never \
                         match; {detail}",
                        ros_name(&local.dds_topic),
                        guid_text(&remote.guid)
                    ),
                );
            }
            if is_service_topic(&local.dds_topic)
                && let Some(peer) = peer_rmw(&remote.guid)
                && let Some(message) = mapping_mismatch(mapping, peer)
            {
                push(format!("rmw {peer:?}"), message);
            }
        }
    }
    findings
}

/// A GUID as dotted hex bytes, the way `ros2 topic info -v` prints GIDs.
fn guid_text(guid: &crate::rustdds::GUID) -> String {
    let bytes: Vec<String> = guid.to_bytes().iter().map(|b| format!("{b:02x}")).collect();
    bytes.join(".")
}

/// Watches a DDS context in the background and reports, once each, the QoS
/// and RMW-mapping problems [`scan`] finds between dora's endpoints and the
/// remote ones. It reads RustDDS's discovery database, so it never consumes
/// the participant status events ros2-client's spinner depends on.
pub struct DdsDiagnostics {
    locals: std::sync::Arc<std::sync::Mutex<Vec<LocalEndpoint>>>,
    // Dropping the sender wakes the thread and stops it.
    stop: Option<std::sync::mpsc::Sender<()>>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl DdsDiagnostics {
    /// How often discovery is re-checked.
    pub const INTERVAL: std::time::Duration = std::time::Duration::from_secs(2);

    /// Start watching `participant`; `report` receives each new finding.
    pub fn start(
        participant: crate::rustdds::DomainParticipant,
        mapping: crate::ros2_client::ServiceMapping,
        report: impl Fn(&str) + Send + 'static,
    ) -> Self {
        Self::start_with(
            move || remote_endpoints(&participant),
            mapping,
            Self::INTERVAL,
            report,
        )
    }

    /// [`Self::start`] with the remote endpoints read from `remotes` every
    /// `interval`.
    fn start_with(
        remotes: impl Fn() -> Vec<RemoteEndpoint> + Send + 'static,
        mapping: crate::ros2_client::ServiceMapping,
        interval: std::time::Duration,
        report: impl Fn(&str) + Send + 'static,
    ) -> Self {
        use std::sync::mpsc::RecvTimeoutError;
        let locals: std::sync::Arc<std::sync::Mutex<Vec<LocalEndpoint>>> = Default::default();
        let (stop, stopped) = std::sync::mpsc::channel::<()>();
        let watched = locals.clone();
        let spawned = std::thread::Builder::new()
            .name("dora-dds-diagnostics".into())
            .spawn(move || {
                let mut reported = std::collections::HashSet::new();
                while let Err(RecvTimeoutError::Timeout) = stopped.recv_timeout(interval) {
                    let locals = watched.lock().map(|l| l.clone()).unwrap_or_default();
                    if locals.is_empty() {
                        continue;
                    }
                    for finding in scan(&locals, &remotes(), mapping) {
                        if reported.insert(finding.key) {
                            report(&finding.message);
                        }
                    }
                }
            });
        let thread = spawned
            .inspect_err(|error| {
                tracing::warn!("DDS diagnostics are off: cannot start their thread: {error}")
            })
            .ok();
        Self {
            locals,
            stop: Some(stop),
            thread,
        }
    }

    /// Watch a topic endpoint with an absolute ROS name.
    pub fn watch_topic(&self, name: &str, role: Role, qos: QosPolicies) {
        self.watch(format!("rt{name}"), role, qos);
    }

    /// Watch a service client or server with an absolute ROS name.
    pub fn watch_service(&self, name: &str, client: bool, qos: &QosPolicies) {
        for (topic, role) in service_endpoints(name, client) {
            self.watch(topic, role, qos.clone());
        }
    }

    /// Watch an action client or server with an absolute ROS name.
    pub fn watch_action(
        &self,
        name: &str,
        client: bool,
        qos: &QosPolicies,
        status_qos: &QosPolicies,
    ) {
        for (topic, role, status) in action_endpoints(name, client) {
            self.watch(topic, role, if status { status_qos } else { qos }.clone());
        }
    }

    /// Add one of dora's endpoints to the comparison. `qos` is what the
    /// endpoint was created with; like RustDDS, it is applied on top of
    /// ros2-client's publisher or subscriber defaults.
    pub fn watch(&self, dds_topic: impl Into<String>, role: Role, qos: QosPolicies) {
        let defaults = QosPolicies::from(match role {
            Role::Writer => crate::ros2_client::DEFAULT_PUBLISHER_QOS,
            Role::Reader => crate::ros2_client::DEFAULT_SUBSCRIPTION_QOS,
        });
        let endpoint = LocalEndpoint {
            dds_topic: dds_topic.into(),
            role,
            qos: defaults.modify_by(&qos),
        };
        if let Ok(mut locals) = self.locals.lock()
            && !locals.contains(&endpoint)
        {
            locals.push(endpoint);
        }
    }
}

impl Drop for DdsDiagnostics {
    fn drop(&mut self) {
        self.stop.take();
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// The DDS request and reply topics of a service with an absolute ROS name,
/// from the client's (`client`) or the server's side.
pub fn service_endpoints(name: &str, client: bool) -> Vec<(String, Role)> {
    let (request, reply) = if client {
        (Role::Writer, Role::Reader)
    } else {
        (Role::Reader, Role::Writer)
    };
    vec![
        (format!("rq{name}Request"), request),
        (format!("rr{name}Reply"), reply),
    ]
}

/// The DDS topics of an action with an absolute ROS name: its three services
/// and its feedback and status topics. The flag marks the status topic, which
/// uses its own QoS.
pub fn action_endpoints(name: &str, client: bool) -> Vec<(String, Role, bool)> {
    let mut endpoints: Vec<_> = ["send_goal", "cancel_goal", "get_result"]
        .iter()
        .flat_map(|service| service_endpoints(&format!("{name}/_action/{service}"), client))
        .map(|(topic, role)| (topic, role, false))
        .collect();
    let topic_role = if client { Role::Reader } else { Role::Writer };
    endpoints.push((format!("rt{name}/_action/feedback"), topic_role, false));
    endpoints.push((format!("rt{name}/_action/status"), topic_role, true));
    endpoints
}

/// Every endpoint discovery knows about that belongs to another participant.
fn remote_endpoints(participant: &crate::rustdds::DomainParticipant) -> Vec<RemoteEndpoint> {
    use crate::rustdds::RTPSEntity;
    let own = participant.guid().prefix;
    let readers = participant
        .discovered_readers()
        .into_iter()
        .map(|r| RemoteEndpoint {
            guid: r.reader_proxy.remote_reader_guid,
            dds_topic: r.subscription_topic_data.topic_name().clone(),
            role: Role::Reader,
            qos: r.subscription_topic_data.qos(),
        });
    let writers = participant
        .discovered_writers()
        .into_iter()
        .map(|w| RemoteEndpoint {
            guid: w.writer_proxy.remote_writer_guid,
            dds_topic: w.publication_topic_data.topic_name().clone(),
            role: Role::Writer,
            qos: w.publication_topic_data.qos(),
        });
    readers
        .chain(writers)
        .filter(|e| e.guid.prefix != own)
        .collect()
}

/// `rt/turtle1/pose` -> `/turtle1/pose`, for messages.
fn ros_name(dds_topic: &str) -> String {
    match dds_topic.split_once('/') {
        Some((_, rest)) => format!("/{rest}"),
        None => dds_topic.to_owned(),
    }
}

/// Outcome of comparing the build's ROS distro with `ROS_DISTRO`.
#[derive(Debug, PartialEq, Eq)]
pub enum DistroCheck {
    Ok,
    /// Possibly compatible; worth a log line.
    Warn(String),
    /// Different Gid layout: ROS 2 graph tools cannot see dora's nodes.
    Mismatch(String),
}

/// Check `ROS_DISTRO` against the build when a DDS context is created. A Gid
/// mismatch goes to stderr, which dora shows by default; the rest is logged.
pub fn report_ros_distro() {
    match check_ros_distro(
        crate::ros2_client::COMPILED_ROS_DISTRO.as_str(),
        std::env::var("ROS_DISTRO").ok().as_deref(),
    ) {
        DistroCheck::Ok => {}
        DistroCheck::Warn(message) => tracing::warn!("{message}"),
        DistroCheck::Mismatch(message) => eprintln!("warning: {message}"),
    }
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
/// different Gid width is a mismatch: ROS 2 nodes then cannot decode dora's
/// `ros_discovery_info`, so graph tools miss dora's nodes. Topics, services
/// and actions match on DDS discovery alone and keep working.
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
             different graph identifiers ({e}-byte vs {b}-byte Gid), so ROS 2 graph tools \
             like `ros2 node list` will not show dora's nodes; topics, services and actions \
             still work. Rebuild the bridge with `--features ros2-{env}` to fix this"
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

    fn reliable() -> QosPolicies {
        qos(|b| {
            b.reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::ZERO,
            })
        })
    }

    fn best_effort() -> QosPolicies {
        qos(|b| b.reliability(policy::Reliability::BestEffort))
    }

    fn local(topic: &str, role: Role, qos: QosPolicies) -> LocalEndpoint {
        LocalEndpoint {
            dds_topic: topic.into(),
            role,
            qos,
        }
    }

    fn remote(vendor: [u8; 2], topic: &str, role: Role, qos: QosPolicies) -> RemoteEndpoint {
        RemoteEndpoint {
            guid: guid_from_vendor(vendor),
            dds_topic: topic.into(),
            role,
            qos,
        }
    }

    const FAST: [u8; 2] = [0x01, 0x0f];
    const CYCLONE: [u8; 2] = [0x01, 0x10];

    #[test]
    fn scan_reports_an_incompatible_remote_on_the_same_topic() {
        use crate::ros2_client::ServiceMapping;
        let locals = [local("rt/chatter", Role::Reader, reliable())];
        let remotes = [remote(FAST, "rt/chatter", Role::Writer, best_effort())];
        let findings = scan(&locals, &remotes, ServiceMapping::Enhanced);
        assert_eq!(findings.len(), 1, "{findings:?}");
        assert!(findings[0].message.contains("/chatter"), "{findings:?}");
        assert!(
            findings[0].message.contains("dora subscription"),
            "{findings:?}"
        );
        assert!(findings[0].message.contains("reliability"), "{findings:?}");
        // The GUID reads like `ros2 topic info -v` prints it.
        assert!(
            findings[0].message.contains("remote publisher 01.0f."),
            "{findings:?}"
        );
    }

    #[test]
    fn scan_ignores_other_topics_same_roles_and_compatible_qos() {
        use crate::ros2_client::ServiceMapping;
        let locals = [local("rt/chatter", Role::Reader, reliable())];
        let remotes = [
            remote(FAST, "rt/other", Role::Writer, best_effort()),
            remote(FAST, "rt/chatter", Role::Reader, best_effort()),
            remote(FAST, "rt/chatter", Role::Writer, reliable()),
        ];
        assert_eq!(scan(&locals, &remotes, ServiceMapping::Enhanced), vec![]);
    }

    #[test]
    fn scan_reports_a_mapping_mismatch_once_per_rmw() {
        use crate::ros2_client::ServiceMapping;
        let locals = [local("rq/add_two_intsRequest", Role::Writer, reliable())];
        let remotes = [
            remote(CYCLONE, "rq/add_two_intsRequest", Role::Reader, reliable()),
            remote(CYCLONE, "rq/add_two_intsRequest", Role::Reader, reliable()),
            remote(CYCLONE, "rq/unrelatedRequest", Role::Reader, reliable()),
        ];
        let findings = scan(&locals, &remotes, ServiceMapping::Enhanced);
        assert_eq!(findings.len(), 1, "{findings:?}");
        assert!(
            findings[0].message.contains("rmw_cyclonedds_cpp"),
            "{findings:?}"
        );
        // Same RMW as the mapping: nothing to report.
        assert_eq!(scan(&locals, &remotes, ServiceMapping::Cyclone), vec![]);
    }

    #[test]
    fn scan_ignores_the_rmw_of_remotes_in_the_same_role() {
        use crate::ros2_client::ServiceMapping;
        // Another client's request writer never talks to dora's client.
        let locals = [local("rq/add_two_intsRequest", Role::Writer, reliable())];
        let remotes = [remote(
            FAST,
            "rq/add_two_intsRequest",
            Role::Writer,
            reliable(),
        )];
        assert_eq!(scan(&locals, &remotes, ServiceMapping::Cyclone), vec![]);
    }

    #[test]
    fn scan_reports_each_distinct_mismatch_with_the_same_remote() {
        use crate::ros2_client::ServiceMapping;
        let transient = qos(|b| {
            b.reliability(policy::Reliability::BestEffort)
                .durability(policy::Durability::TransientLocal)
        });
        let locals = [
            local("rt/chatter", Role::Reader, reliable()),
            local("rt/chatter", Role::Reader, transient),
        ];
        let volatile_best_effort = qos(|b| {
            b.reliability(policy::Reliability::BestEffort)
                .durability(policy::Durability::Volatile)
        });
        let remotes = [remote(
            FAST,
            "rt/chatter",
            Role::Writer,
            volatile_best_effort,
        )];
        let findings = scan(&locals, &remotes, ServiceMapping::Enhanced);
        assert_eq!(findings.len(), 2, "{findings:?}");
        assert_ne!(findings[0].key, findings[1].key);
    }

    fn quiet_watcher(
        remotes: Vec<RemoteEndpoint>,
    ) -> (DdsDiagnostics, std::sync::mpsc::Receiver<String>) {
        let (tx, rx) = std::sync::mpsc::channel();
        let watcher = DdsDiagnostics::start_with(
            move || remotes.clone(),
            crate::ros2_client::ServiceMapping::Enhanced,
            std::time::Duration::from_millis(10),
            move |message| {
                let _ = tx.send(message.to_owned());
            },
        );
        (watcher, rx)
    }

    #[test]
    fn the_watcher_reports_a_finding_once() {
        let (watcher, rx) = quiet_watcher(vec![remote(
            FAST,
            "rt/chatter",
            Role::Writer,
            best_effort(),
        )]);
        watcher.watch_topic("/chatter", Role::Reader, reliable());
        let first = rx.recv_timeout(std::time::Duration::from_secs(5)).unwrap();
        assert!(first.contains("reliability"), "{first}");
        assert!(
            rx.recv_timeout(std::time::Duration::from_millis(100))
                .is_err()
        );
    }

    #[test]
    fn watching_the_same_endpoint_twice_keeps_one() {
        let (watcher, _rx) = quiet_watcher(vec![]);
        watcher.watch_topic("/chatter", Role::Reader, reliable());
        watcher.watch_topic("/chatter", Role::Reader, reliable());
        assert_eq!(watcher.locals.lock().unwrap().len(), 1);
    }

    #[test]
    fn watched_qos_starts_from_ros2_client_defaults() {
        let (watcher, _rx) = quiet_watcher(vec![]);
        watcher.watch("rt/x", Role::Writer, QosPolicies::qos_none());
        watcher.watch("rt/x", Role::Reader, QosPolicies::qos_none());
        let locals = watcher.locals.lock().unwrap();
        let writer = QosPolicies::from(crate::ros2_client::DEFAULT_PUBLISHER_QOS);
        let reader = QosPolicies::from(crate::ros2_client::DEFAULT_SUBSCRIPTION_QOS);
        assert_eq!(locals[0].qos, writer);
        assert_eq!(locals[1].qos, reader);
    }

    #[test]
    fn dropping_the_watcher_stops_its_thread_promptly() {
        let (tx, rx) = std::sync::mpsc::channel::<()>();
        let watcher = DdsDiagnostics::start_with(
            move || {
                let _keep = &tx; // dropped with the thread's closure
                vec![]
            },
            crate::ros2_client::ServiceMapping::Enhanced,
            std::time::Duration::from_secs(60),
            |_| {},
        );
        let started = std::time::Instant::now();
        drop(watcher);
        // The thread (and everything it owns, like the participant) is gone.
        assert_eq!(
            rx.recv_timeout(std::time::Duration::from_secs(1)),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected)
        );
        assert!(started.elapsed() < std::time::Duration::from_secs(1));
    }

    #[test]
    fn service_endpoints_match_ros2_client_naming() {
        use crate::ros2_client::{Name, NodeName};
        let node = NodeName::new("/", "n").unwrap();
        let name = Name::parse("/add_two_ints").unwrap();
        let request = name.to_dds_name("rq", &node, "Request");
        let reply = name.to_dds_name("rr", &node, "Reply");
        assert_eq!(
            service_endpoints("/add_two_ints", true),
            vec![
                (request.clone(), Role::Writer),
                (reply.clone(), Role::Reader)
            ]
        );
        assert_eq!(
            service_endpoints("/add_two_ints", false),
            vec![(request, Role::Reader), (reply, Role::Writer)]
        );
    }

    #[test]
    fn action_endpoints_cover_the_three_services_and_two_topics() {
        let client = action_endpoints("/fibonacci", true);
        assert!(client.contains(&(
            "rq/fibonacci/_action/send_goalRequest".into(),
            Role::Writer,
            false
        )));
        assert!(client.contains(&(
            "rr/fibonacci/_action/get_resultReply".into(),
            Role::Reader,
            false
        )));
        assert!(client.contains(&(
            "rq/fibonacci/_action/cancel_goalRequest".into(),
            Role::Writer,
            false
        )));
        assert!(client.contains(&("rt/fibonacci/_action/feedback".into(), Role::Reader, false)));
        assert!(client.contains(&("rt/fibonacci/_action/status".into(), Role::Reader, true)));
        assert_eq!(client.len(), 8);
        let server = action_endpoints("/fibonacci", false);
        assert!(server.contains(&("rt/fibonacci/_action/status".into(), Role::Writer, true)));
        assert!(server.contains(&(
            "rq/fibonacci/_action/send_goalRequest".into(),
            Role::Reader,
            false
        )));
    }

    #[test]
    fn the_mismatch_message_names_the_effect_and_the_fix() {
        let DistroCheck::Mismatch(message) = check_ros_distro("humble", Some("jazzy")) else {
            panic!("expected a mismatch");
        };
        // Measured with the DDS interop matrix: only graph introspection breaks.
        assert!(message.contains("`ros2 node list`"), "{message}");
        assert!(
            message.contains("topics, services and actions still work"),
            "{message}"
        );
        assert!(message.contains("--features ros2-jazzy"), "{message}");
    }
}
