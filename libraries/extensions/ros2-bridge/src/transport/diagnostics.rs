//! Diagnostics for the DDS transport: misconfigurations that otherwise make
//! dora and ROS 2 nodes silently stop talking (#4205 items 8 and 9).
//!
//! Findings go to stderr, which dora shows by default (warnings logged through
//! `tracing` are below the default level of most dora processes).

use super::interfaces::Unrestricted;
use crate::{
    ros2_client::{
        self, Name, NodeName, QosProfile, RosDistro, ServiceMapping,
        action::{ActionClientQosPolicies, ActionServerQosPolicies},
    },
    rustdds::{
        self, GUID, QosPolicies,
        qos::{QosPolicyId, policy},
    },
};
use std::{
    collections::{BTreeMap, HashSet},
    net::{Ipv4Addr, SocketAddr},
    sync::{
        Arc, Mutex, Weak,
        atomic::{AtomicU64, Ordering},
        mpsc,
    },
    thread::JoinHandle,
    time::Duration,
};

/// Why an offered QoS (a writer's) cannot serve a requested one (a reader's),
/// or `None` when they match. The rule is RustDDS's own matching check, so the
/// diagnosis cannot disagree with the actual matching; this only names the
/// first offending policy with both values.
fn qos_mismatch(offered: &QosPolicies, requested: &QosPolicies) -> Option<String> {
    let id = offered.compliance_failure_wrt(requested)?;
    let reliability = |q: &QosPolicies| match q.reliability() {
        Some(policy::Reliability::Reliable { .. }) => "RELIABLE".to_owned(),
        _ => "BEST_EFFORT".to_owned(),
    };
    let durability = |q: &QosPolicies| {
        match q.durability().unwrap_or(policy::Durability::Volatile) {
            policy::Durability::Volatile => "VOLATILE",
            policy::Durability::TransientLocal => "TRANSIENT_LOCAL",
            policy::Durability::Transient => "TRANSIENT",
            policy::Durability::Persistent => "PERSISTENT",
        }
        .to_owned()
    };
    let value: &dyn Fn(&QosPolicies) -> String = match id {
        QosPolicyId::Reliability => &reliability,
        QosPolicyId::Durability => &durability,
        QosPolicyId::Deadline => &|q| format!("{:?}", q.deadline()),
        QosPolicyId::Liveliness => &|q| format!("{:?}", q.liveliness()),
        QosPolicyId::Ownership => &|q| format!("{:?}", q.ownership()),
        _ => &|_| "?".to_owned(),
    };
    Some(format!(
        "{}: the reader requests {}, the writer offers {}",
        format!("{id:?}").to_lowercase(),
        value(requested),
        value(offered)
    ))
}

/// The ROS 2 RMW a remote DDS participant most likely runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum PeerRmw {
    FastDds,
    CycloneDds,
}

/// Guess a remote endpoint's RMW from its GUID. RTPS implementations put their
/// vendor id in the first two bytes of the GUID prefix: eProsima (Fast-DDS)
/// uses 01.0f, Eclipse (CycloneDDS) 01.10.
fn peer_rmw(guid: &GUID) -> Option<PeerRmw> {
    match guid.to_bytes()[..2] {
        [0x01, 0x0f] => Some(PeerRmw::FastDds),
        [0x01, 0x10] => Some(PeerRmw::CycloneDds),
        _ => None,
    }
}

/// Whether a DDS topic carries ROS 2 service requests or replies (services and
/// the action goal/result/cancel services), whose framing depends on the
/// service mapping.
fn is_service_topic(dds_topic: &str) -> bool {
    dds_topic.starts_with("rq/") || dds_topic.starts_with("rr/")
}

/// The fix when dora's service mapping cannot talk to a peer's RMW: both sides
/// see each other, but every request or reply fails to decode, so calls hang.
fn mapping_mismatch(mapping: ServiceMapping, peer: PeerRmw) -> Option<String> {
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
struct LocalEndpoint {
    dds_topic: String,
    role: Role,
    qos: QosPolicies,
}

/// A DDS endpoint discovery reported from another participant.
#[derive(Clone, Debug)]
struct RemoteEndpoint {
    guid: GUID,
    dds_topic: String,
    role: Role,
    qos: QosPolicies,
    /// The IPv4 addresses it receives data on.
    addresses: Vec<Ipv4Addr>,
}

/// Something to tell the user. `key` identifies it so it is reported once.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Finding {
    key: String,
    message: String,
}

/// A GUID as dotted hex bytes, the way `ros2 topic info -v` prints GIDs.
fn guid_text(guid: &GUID) -> String {
    let bytes: Vec<String> = guid.to_bytes().iter().map(|b| format!("{b:02x}")).collect();
    bytes.join(".")
}

/// Compare dora's endpoints with the remote ones that would talk to them (same
/// topic, other role): incompatible QoS, peers whose RMW does not fit dora's
/// service mapping on a service topic, and, with `unrestricted`, peers on
/// another host. The caller reports each key once.
fn scan(
    locals: &[LocalEndpoint],
    remotes: &[RemoteEndpoint],
    mapping: ServiceMapping,
    unrestricted: Option<&Unrestricted>,
) -> Vec<Finding> {
    let mut findings = Vec::new();
    for local in locals {
        let peers = remotes
            .iter()
            .filter(|r| r.dds_topic == local.dds_topic && r.role != local.role);
        for remote in peers {
            let (offered, requested, ours, theirs) = match local.role {
                Role::Writer => (&local.qos, &remote.qos, "publisher", "subscription"),
                Role::Reader => (&remote.qos, &local.qos, "subscription", "publisher"),
            };
            if let Some(detail) = qos_mismatch(offered, requested) {
                findings.push(Finding {
                    key: format!(
                        "qos {} {:?} {:?} {detail}",
                        local.dds_topic, local.role, remote.guid
                    ),
                    message: format!(
                        "QoS mismatch on {}: dora {ours} and remote {theirs} {} never match; \
                         {detail}",
                        ros_name(&local.dds_topic),
                        guid_text(&remote.guid)
                    ),
                });
            }
            if is_service_topic(&local.dds_topic)
                && let Some(peer) = peer_rmw(&remote.guid)
                && let Some(message) = mapping_mismatch(mapping, peer)
            {
                findings.push(Finding {
                    key: format!("rmw {peer:?}"),
                    message,
                });
            }
            if let Some(unrestricted) = unrestricted
                && unrestricted.is_elsewhere(&remote.addresses)
            {
                findings.push(Finding {
                    key: "interfaces".into(),
                    message: unrestricted.message.clone(),
                });
            }
        }
    }
    findings
}

/// dora's endpoints by [`Watch`]: each watch adds its endpoints and removes
/// them when dropped.
type Locals = Arc<Mutex<BTreeMap<u64, Vec<LocalEndpoint>>>>;

/// Watches a DDS participant in the background and reports, once each, the
/// QoS and RMW-mapping problems [`scan`] finds between dora's endpoints and the
/// remote ones. It reads RustDDS's discovery database, so it never consumes
/// the participant status events ros2-client's spinner depends on.
///
/// [`super::dds::Context`] owns one. Register each endpoint after creating it
/// and keep the returned [`Watch`] as long as the endpoint lives (see
/// [`Watched`]). RustDDS's discovery database only lists dora's own endpoints
/// once they come back over the network, so they are registered by hand.
pub struct DdsDiagnostics {
    locals: Locals,
    next_watch: AtomicU64,
    stop: mpsc::Sender<()>,
    thread: Option<JoinHandle<()>>,
}

/// Keeps endpoints registered with [`DdsDiagnostics`]; dropping it removes
/// them, so a dropped endpoint is not compared with peers that appear later.
#[must_use = "the endpoints are unregistered when the watch is dropped"]
pub struct Watch {
    locals: Weak<Mutex<BTreeMap<u64, Vec<LocalEndpoint>>>>,
    id: u64,
}

impl Watch {
    /// Bundle `endpoint` with this watch, so it is watched exactly as long as
    /// it lives.
    pub fn attach<T>(self, endpoint: T) -> Watched<T> {
        Watched {
            endpoint,
            _watch: self,
        }
    }
}

impl Drop for Watch {
    fn drop(&mut self) {
        if let Some(locals) = self.locals.upgrade()
            && let Ok(mut locals) = locals.lock()
        {
            locals.remove(&self.id);
        }
    }
}

/// An endpoint together with its [`Watch`]. Derefs to the endpoint.
pub struct Watched<T> {
    endpoint: T,
    _watch: Watch,
}

impl<T> std::ops::Deref for Watched<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.endpoint
    }
}

impl<T> std::ops::DerefMut for Watched<T> {
    fn deref_mut(&mut self) -> &mut T {
        &mut self.endpoint
    }
}

impl DdsDiagnostics {
    /// How often discovery is re-checked.
    const INTERVAL: Duration = Duration::from_secs(2);

    /// Start watching `participant`, reporting findings on stderr.
    /// `unrestricted` is reported once a peer on another host talks to dora.
    pub(crate) fn start(
        participant: rustdds::DomainParticipant,
        mapping: ServiceMapping,
        unrestricted: Option<Unrestricted>,
    ) -> Self {
        Self::start_reporting(
            participant,
            mapping,
            unrestricted,
            Self::INTERVAL,
            |finding| eprintln!("warning: {finding}"),
        )
    }

    /// [`Self::start`], checking every `interval` and sending findings to
    /// `report`.
    fn start_reporting(
        participant: rustdds::DomainParticipant,
        mapping: ServiceMapping,
        unrestricted: Option<Unrestricted>,
        interval: Duration,
        report: impl Fn(&str) + Send + 'static,
    ) -> Self {
        use rustdds::RTPSEntity;
        let own = participant.guid().prefix;
        Self::start_with(
            move || {
                let mut remotes = remote_endpoints(&participant);
                remotes.retain(|r| r.guid.prefix != own);
                remotes
            },
            mapping,
            unrestricted,
            interval,
            report,
        )
    }

    /// [`Self::start`] with the remote endpoints read from `remotes` every
    /// `interval`, and findings sent to `report`.
    fn start_with(
        remotes: impl Fn() -> Vec<RemoteEndpoint> + Send + 'static,
        mapping: ServiceMapping,
        unrestricted: Option<Unrestricted>,
        interval: Duration,
        report: impl Fn(&str) + Send + 'static,
    ) -> Self {
        let locals = Locals::default();
        let (stop, stopped) = mpsc::channel();
        let watched = locals.clone();
        let thread = std::thread::Builder::new()
            .name("dora-dds-diagnostics".into())
            .spawn(move || {
                let mut reported = HashSet::new();
                while let Err(mpsc::RecvTimeoutError::Timeout) = stopped.recv_timeout(interval) {
                    let locals = flatten(&watched);
                    if locals.is_empty() {
                        continue;
                    }
                    let remotes = remotes();
                    for finding in scan(&locals, &remotes, mapping, unrestricted.as_ref()) {
                        if reported.insert(finding.key) {
                            report(&finding.message);
                        }
                    }
                }
            })
            .inspect_err(|error| {
                tracing::warn!("DDS diagnostics are off: cannot start their thread: {error}")
            })
            .ok();
        Self {
            locals,
            next_watch: AtomicU64::new(0),
            stop,
            thread,
        }
    }

    /// Watch a publisher (`Role::Writer`) or subscription (`Role::Reader`)
    /// created on `topic`, with `qos` as the override passed to ros2-client.
    pub fn watch_topic(
        &self,
        topic: &rustdds::Topic,
        role: Role,
        qos: Option<&QosProfile>,
    ) -> Watch {
        use rustdds::{TopicDescription, qos::HasQoSPolicy};
        let topic_qos = match qos {
            Some(qos) => topic.qos().modify_by(&QosPolicies::from(qos)),
            None => topic.qos(),
        };
        self.watch(vec![(topic.name(), role, topic_qos)])
    }

    /// Watch a service client or server that `node` created, with the QoS
    /// passed to ros2-client.
    pub fn watch_service(
        &self,
        node: &NodeName,
        name: &Name,
        client: bool,
        request_qos: &QosProfile,
        reply_qos: &QosProfile,
    ) -> Watch {
        self.watch(service_endpoints(node, name, "", client, request_qos, reply_qos).into())
    }

    /// Watch an action client that `node` created, with the QoS passed to
    /// ros2-client.
    pub fn watch_action_client(
        &self,
        node: &NodeName,
        name: &Name,
        qos: &ActionClientQosPolicies,
    ) -> Watch {
        self.watch_action(
            node,
            name,
            true,
            [&qos.goal_service, &qos.result_service, &qos.cancel_service],
            &qos.feedback_subscription,
            &qos.status_subscription,
        )
    }

    /// Watch an action server that `node` created, with the QoS passed to
    /// ros2-client.
    pub fn watch_action_server(
        &self,
        node: &NodeName,
        name: &Name,
        qos: &ActionServerQosPolicies,
    ) -> Watch {
        self.watch_action(
            node,
            name,
            false,
            [&qos.goal_service, &qos.result_service, &qos.cancel_service],
            &qos.feedback_publisher,
            &qos.status_publisher,
        )
    }

    /// An action is three services plus the feedback and status topics, all
    /// under `<name>/_action/`.
    fn watch_action(
        &self,
        node: &NodeName,
        name: &Name,
        client: bool,
        [goal, result, cancel]: [&QosProfile; 3],
        feedback: &QosProfile,
        status: &QosProfile,
    ) -> Watch {
        let mut endpoints = Vec::new();
        for (service, qos) in [
            ("send_goal", goal),
            ("get_result", result),
            ("cancel_goal", cancel),
        ] {
            let service = format!("/_action/{service}");
            endpoints.extend(service_endpoints(node, name, &service, client, qos, qos));
        }
        let role = if client { Role::Reader } else { Role::Writer };
        for (topic, qos) in [("feedback", feedback), ("status", status)] {
            let dds_topic = name.to_dds_name("rt", node, &format!("/_action/{topic}"));
            endpoints.push((dds_topic, role, qos.into()));
        }
        self.watch(endpoints)
    }

    /// Add dora's endpoints to the comparison until the returned watch is
    /// dropped. Like RustDDS, each QoS applies on top of ros2-client's
    /// publisher or subscriber defaults.
    fn watch(&self, endpoints: Vec<(String, Role, QosPolicies)>) -> Watch {
        let endpoints = endpoints
            .into_iter()
            .map(|(dds_topic, role, qos)| {
                let defaults = QosPolicies::from(match role {
                    Role::Writer => ros2_client::DEFAULT_PUBLISHER_QOS,
                    Role::Reader => ros2_client::DEFAULT_SUBSCRIPTION_QOS,
                });
                LocalEndpoint {
                    dds_topic,
                    role,
                    qos: defaults.modify_by(&qos),
                }
            })
            .collect();
        let id = self.next_watch.fetch_add(1, Ordering::Relaxed);
        if let Ok(mut locals) = self.locals.lock() {
            locals.insert(id, endpoints);
        }
        Watch {
            locals: Arc::downgrade(&self.locals),
            id,
        }
    }

    /// Every endpoint currently watched.
    #[cfg(test)]
    fn local_endpoints(&self) -> Vec<LocalEndpoint> {
        flatten(&self.locals)
    }
}

fn flatten(locals: &Locals) -> Vec<LocalEndpoint> {
    locals
        .lock()
        .map(|l| l.values().flatten().cloned().collect())
        .unwrap_or_default()
}

impl Drop for DdsDiagnostics {
    fn drop(&mut self) {
        let _ = self.stop.send(());
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// The request and reply topics of service `name` + `suffix` that `node`
/// created, from the client's (`client`) or the server's side. The DDS names
/// come from ros2-client, so a relative `name` gets the node's namespace.
fn service_endpoints(
    node: &NodeName,
    name: &Name,
    suffix: &str,
    client: bool,
    request_qos: &QosProfile,
    reply_qos: &QosProfile,
) -> [(String, Role, QosPolicies); 2] {
    let (request, reply) = if client {
        (Role::Writer, Role::Reader)
    } else {
        (Role::Reader, Role::Writer)
    };
    [
        (
            name.to_dds_name("rq", node, &format!("{suffix}Request")),
            request,
            request_qos.into(),
        ),
        (
            name.to_dds_name("rr", node, &format!("{suffix}Reply")),
            reply,
            reply_qos.into(),
        ),
    ]
}

/// The IPv4 addresses among `locators` (RustDDS does not export its
/// `Locator` type, only its conversion to a socket address).
fn ipv4<L: Copy + Into<SocketAddr>>(locators: &[L]) -> Vec<Ipv4Addr> {
    locators
        .iter()
        .filter_map(|&locator| match locator.into() {
            SocketAddr::V4(address) => Some(*address.ip()),
            SocketAddr::V6(_) => None,
        })
        .collect()
}

/// Every endpoint discovery knows about, including the participant's own.
fn remote_endpoints(participant: &rustdds::DomainParticipant) -> Vec<RemoteEndpoint> {
    let readers = participant
        .discovered_readers()
        .into_iter()
        .map(|r| RemoteEndpoint {
            guid: r.reader_proxy.remote_reader_guid,
            dds_topic: r.subscription_topic_data.topic_name().clone(),
            role: Role::Reader,
            qos: r.subscription_topic_data.qos(),
            addresses: ipv4(&r.reader_proxy.unicast_locator_list),
        });
    let writers = participant
        .discovered_writers()
        .into_iter()
        .map(|w| RemoteEndpoint {
            guid: w.writer_proxy.remote_writer_guid,
            dds_topic: w.publication_topic_data.topic_name().clone(),
            role: Role::Writer,
            qos: w.publication_topic_data.qos(),
            addresses: ipv4(&w.writer_proxy.unicast_locator_list),
        });
    readers.chain(writers).collect()
}

/// The ROS name of a DDS topic (`rt/chatter` -> `/chatter`).
fn ros_name(dds_topic: &str) -> String {
    match dds_topic.get(..3) {
        Some("rt/" | "rq/" | "rr/") => dds_topic[2..].to_owned(),
        _ => dds_topic.to_owned(),
    }
}

/// Warn, once per process, when `ROS_DISTRO` needs a different Gid width than
/// the build. (ros2-client itself logs the other distro differences.)
pub(crate) fn report_ros_distro() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let env = std::env::var("ROS_DISTRO").ok();
        if let Some(message) = gid_mismatch(
            ros2_client::COMPILED_ROS_DISTRO,
            ros2_client::gid::GID_LENGTH,
            env.as_deref(),
        ) {
            eprintln!("warning: {message}");
        }
    });
}

/// The `rmw_dds_common` Gid width of a `ROS_DISTRO`: 24 bytes up to Humble,
/// 16 from Iron on. `None` for a distro this check does not know.
fn gid_width(distro: &str) -> Option<usize> {
    match distro {
        "galactic" | "humble" => Some(24),
        "iron" | "jazzy" | "kilted" | "lyrical" | "rolling" => Some(16),
        _ => None,
    }
}

/// What a build for `built` (with `built_gid`-byte Gids) running next to
/// `ROS_DISTRO=env` should tell the user. A different Gid width means ROS 2
/// nodes cannot decode dora's `ros_discovery_info`, so graph tools miss dora's
/// nodes; topics, services and actions match on DDS discovery alone and keep
/// working (measured with the DDS interop matrix).
fn gid_mismatch(built: RosDistro, built_gid: usize, env: Option<&str>) -> Option<String> {
    let env = env?;
    let env_gid = gid_width(env).filter(|&width| width != built_gid)?;
    // The distro features form a chain where the newest one wins, and
    // `ros2-humble` (24-byte Gid) is the default. So a newer distro needs a
    // feature added, and an older one needs the newer feature removed.
    let fix = if env_gid > built_gid {
        format!("Rebuild the bridge without `--features ros2-{built}`")
    } else {
        let feature = match env {
            "iron" | "jazzy" | "kilted" => env,
            // No feature of its own; the newest one has the same Gid width.
            _ => "kilted",
        };
        format!("Rebuild the bridge with `--features ros2-{feature}`")
    };
    Some(format!(
        "ROS_DISTRO='{env}' but the ROS 2 bridge was built for '{built}': they use different \
         graph identifiers ({env_gid}-byte vs {built_gid}-byte Gid), so ROS 2 graph tools like \
         `ros2 node list` will not show dora's nodes; topics, services and actions still work. \
         {fix} to fix this"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{rustdds::QosPolicyBuilder, transport::interfaces::LocalAddr};

    #[test]
    fn a_different_gid_width_is_reported() {
        // Humble/Galactic use the 24-byte Gid, Iron and later the 16-byte one.
        assert!(gid_mismatch(RosDistro::Humble, 24, Some("jazzy")).is_some());
        assert!(gid_mismatch(RosDistro::Humble, 24, Some("rolling")).is_some());
        assert!(gid_mismatch(RosDistro::Kilted, 16, Some("humble")).is_some());
        assert!(gid_mismatch(RosDistro::Galactic, 24, Some("iron")).is_some());
    }

    #[test]
    fn the_same_gid_width_unset_or_unknown_is_not_reported() {
        assert_eq!(gid_mismatch(RosDistro::Humble, 24, Some("humble")), None);
        assert_eq!(gid_mismatch(RosDistro::Jazzy, 16, Some("kilted")), None);
        assert_eq!(gid_mismatch(RosDistro::Humble, 24, Some("galactic")), None);
        assert_eq!(gid_mismatch(RosDistro::Humble, 24, None), None);
        assert_eq!(gid_mismatch(RosDistro::Humble, 24, Some("foxy")), None);
    }

    #[test]
    fn the_mismatch_message_names_the_effect_and_the_fix() {
        let message = gid_mismatch(RosDistro::Humble, 24, Some("jazzy")).expect("mismatch");
        // Measured with the DDS interop matrix: only graph introspection breaks.
        assert!(message.contains("`ros2 node list`"), "{message}");
        assert!(
            message.contains("topics, services and actions still work"),
            "{message}"
        );
        assert!(message.contains("16-byte vs 24-byte"), "{message}");
        assert!(message.contains("--features ros2-jazzy"), "{message}");
    }

    #[test]
    fn a_newer_distro_without_a_feature_gets_the_newest_feature() {
        // The bridge has no `ros2-rolling`, `ros2-lyrical` or `ros2-galactic`.
        for env in ["rolling", "lyrical"] {
            let message = gid_mismatch(RosDistro::Humble, 24, Some(env)).expect("mismatch");
            assert!(message.contains("--features ros2-kilted"), "{message}");
            assert!(!message.contains(&format!("ros2-{env}")), "{message}");
        }
    }

    #[test]
    fn an_older_distro_is_fixed_by_dropping_the_newer_feature() {
        // `ros2-humble` is on by default and the newest feature wins, so adding
        // it to a Kilted build changes nothing.
        for env in ["humble", "galactic"] {
            let message = gid_mismatch(RosDistro::Kilted, 16, Some(env)).expect("mismatch");
            assert!(
                message.contains("without `--features ros2-kilted`"),
                "{message}"
            );
            assert!(
                !message.contains(&format!("--features ros2-{env}")),
                "{message}"
            );
        }
    }

    fn qos(f: impl FnOnce(QosPolicyBuilder) -> QosPolicyBuilder) -> QosPolicies {
        f(QosPolicyBuilder::new()).build()
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

    #[test]
    fn compatible_qos_has_no_mismatch() {
        assert_eq!(qos_mismatch(&reliable(), &reliable()), None);
        // A stronger offer than requested is fine.
        assert_eq!(qos_mismatch(&reliable(), &qos(|b| b)), None);
    }

    #[test]
    fn best_effort_offer_against_reliable_request_names_reliability() {
        let message = qos_mismatch(&best_effort(), &reliable()).expect("incompatible");
        assert_eq!(
            message,
            "reliability: the reader requests RELIABLE, the writer offers BEST_EFFORT"
        );
    }

    #[test]
    fn volatile_offer_against_transient_local_request_names_durability() {
        let offered = qos(|b| b.durability(policy::Durability::Volatile));
        let requested = qos(|b| b.durability(policy::Durability::TransientLocal));
        let message = qos_mismatch(&offered, &requested).expect("incompatible");
        assert_eq!(
            message,
            "durability: the reader requests TRANSIENT_LOCAL, the writer offers VOLATILE"
        );
    }

    #[test]
    fn a_longer_offered_deadline_names_deadline() {
        let deadline = |secs| {
            move |b: QosPolicyBuilder| {
                b.deadline(policy::Deadline(rustdds::Duration::from_secs(secs)))
            }
        };
        let message = qos_mismatch(&qos(deadline(2)), &qos(deadline(1))).expect("incompatible");
        assert!(
            message.starts_with("deadline: the reader requests Some("),
            "{message}"
        );
    }

    fn guid_from_vendor(vendor: [u8; 2]) -> GUID {
        let mut bytes = [0x2a_u8; 16];
        bytes[..2].copy_from_slice(&vendor);
        GUID::from_bytes(bytes)
    }

    const FAST: [u8; 2] = [0x01, 0x0f];
    const CYCLONE: [u8; 2] = [0x01, 0x10];

    #[test]
    fn the_guid_prefix_names_the_peer_rmw() {
        assert_eq!(peer_rmw(&guid_from_vendor(FAST)), Some(PeerRmw::FastDds));
        assert_eq!(
            peer_rmw(&guid_from_vendor(CYCLONE)),
            Some(PeerRmw::CycloneDds)
        );
        assert_eq!(peer_rmw(&guid_from_vendor([0x01, 0x12])), None);
    }

    #[test]
    fn a_mapping_that_does_not_fit_the_peer_names_the_rmw_to_set() {
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
            addresses: vec![],
        }
    }

    /// A reliable remote endpoint advertising `addresses`.
    fn remote_at(topic: &str, role: Role, addresses: &[&str]) -> RemoteEndpoint {
        RemoteEndpoint {
            addresses: addresses.iter().map(|a| a.parse().unwrap()).collect(),
            ..remote(FAST, topic, role, reliable())
        }
    }

    /// This host: eno1 192.168.1.231 and docker0 172.17.0.1.
    fn unrestricted() -> Unrestricted {
        let addr = |name: &str, ip: &str| LocalAddr {
            name: name.into(),
            ip: ip.parse().unwrap(),
            loopback: false,
            multicast: true,
            up: true,
        };
        Unrestricted::new(&[addr("eno1", "192.168.1.231"), addr("docker0", "172.17.0.1")]).unwrap()
    }

    #[test]
    fn several_addresses_are_reported_once_a_peer_on_another_host_talks_to_dora() {
        let locals = [local("rt/chatter", Role::Reader, reliable())];
        let remotes = [
            remote_at("rt/chatter", Role::Writer, &["192.168.1.50"]),
            remote_at("rt/chatter", Role::Writer, &["192.168.1.51"]),
        ];
        let findings = scan(
            &locals,
            &remotes,
            ServiceMapping::Enhanced,
            Some(&unrestricted()),
        );
        let keys: HashSet<_> = findings.iter().map(|f| &f.key).collect();
        assert_eq!(keys.len(), 1, "{findings:?}");
        assert!(
            findings[0].message.contains("docker0 172.17.0.1"),
            "{findings:?}"
        );
    }

    #[test]
    fn several_addresses_are_not_reported_without_a_peer_on_another_host() {
        let locals = [local("rt/chatter", Role::Reader, reliable())];
        let remotes = [
            // Same host: this host's address, or loopback.
            remote_at("rt/chatter", Role::Writer, &["192.168.1.231"]),
            remote_at("rt/chatter", Role::Writer, &["127.0.0.1"]),
            // Another host, but not talking to dora.
            remote_at("rt/other", Role::Writer, &["192.168.1.50"]),
            remote_at("rt/chatter", Role::Reader, &["192.168.1.50"]),
        ];
        let findings = scan(
            &locals,
            &remotes,
            ServiceMapping::Enhanced,
            Some(&unrestricted()),
        );
        assert_eq!(findings, vec![]);
        // One address, or DORA_ROS2_DDS_INTERFACES set: nothing to report.
        let elsewhere = [remote_at("rt/chatter", Role::Writer, &["192.168.1.50"])];
        assert_eq!(
            scan(&locals, &elsewhere, ServiceMapping::Enhanced, None),
            vec![]
        );
    }

    #[test]
    fn scan_reports_an_incompatible_remote_on_the_same_topic() {
        let locals = [local("rt/chatter", Role::Reader, reliable())];
        let remotes = [remote(FAST, "rt/chatter", Role::Writer, best_effort())];
        let findings = scan(&locals, &remotes, ServiceMapping::Enhanced, None);
        assert_eq!(findings.len(), 1, "{findings:?}");
        let message = &findings[0].message;
        assert!(
            message.starts_with("QoS mismatch on /chatter: dora subscription"),
            "{message}"
        );
        // The GUID reads like `ros2 topic info -v` prints it.
        assert!(message.contains("remote publisher 01.0f.2a."), "{message}");
        assert!(message.contains("reliability"), "{message}");
    }

    #[test]
    fn scan_ignores_other_topics_same_roles_and_compatible_qos() {
        let locals = [local("rt/chatter", Role::Reader, reliable())];
        let remotes = [
            remote(FAST, "rt/other", Role::Writer, best_effort()),
            remote(FAST, "rt/chatter", Role::Reader, best_effort()),
            remote(FAST, "rt/chatter", Role::Writer, reliable()),
        ];
        assert_eq!(
            scan(&locals, &remotes, ServiceMapping::Enhanced, None),
            vec![]
        );
    }

    #[test]
    fn scan_keys_a_mapping_mismatch_by_rmw() {
        let locals = [local("rq/add_two_intsRequest", Role::Writer, reliable())];
        let remotes = [
            remote(CYCLONE, "rq/add_two_intsRequest", Role::Reader, reliable()),
            remote(CYCLONE, "rq/add_two_intsRequest", Role::Reader, reliable()),
            remote(CYCLONE, "rq/unrelatedRequest", Role::Reader, reliable()),
        ];
        let findings = scan(&locals, &remotes, ServiceMapping::Enhanced, None);
        let keys: HashSet<_> = findings.iter().map(|f| &f.key).collect();
        assert_eq!(keys.len(), 1, "{findings:?}");
        assert!(
            findings[0].message.contains("rmw_cyclonedds_cpp"),
            "{findings:?}"
        );
        // Same RMW as the mapping: nothing to report.
        assert_eq!(
            scan(&locals, &remotes, ServiceMapping::Cyclone, None),
            vec![]
        );
    }

    #[test]
    fn scan_ignores_the_rmw_of_remotes_in_the_same_role() {
        // Another client's request writer never talks to dora's client.
        let locals = [local("rq/add_two_intsRequest", Role::Writer, reliable())];
        let remotes = [remote(
            FAST,
            "rq/add_two_intsRequest",
            Role::Writer,
            reliable(),
        )];
        assert_eq!(
            scan(&locals, &remotes, ServiceMapping::Cyclone, None),
            vec![]
        );
    }

    #[test]
    fn scan_reports_each_distinct_mismatch_with_the_same_remote() {
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
        let findings = scan(&locals, &remotes, ServiceMapping::Enhanced, None);
        assert_eq!(findings.len(), 2, "{findings:?}");
        assert_ne!(findings[0].key, findings[1].key);
    }

    fn quiet_watcher(remotes: Vec<RemoteEndpoint>) -> (DdsDiagnostics, mpsc::Receiver<String>) {
        let (tx, rx) = mpsc::channel();
        let watcher = DdsDiagnostics::start_with(
            move || remotes.clone(),
            ServiceMapping::Enhanced,
            None,
            Duration::from_millis(10),
            move |message| {
                let _ = tx.send(message.to_owned());
            },
        );
        (watcher, rx)
    }

    fn watched(watcher: &DdsDiagnostics) -> Vec<(String, Role)> {
        watcher
            .local_endpoints()
            .into_iter()
            .map(|l| (l.dds_topic, l.role))
            .collect()
    }

    fn node() -> NodeName {
        NodeName::new("/robot", "n").unwrap()
    }

    #[test]
    fn the_watcher_reports_a_finding_once() {
        let (watcher, rx) = quiet_watcher(vec![remote(
            FAST,
            "rt/chatter",
            Role::Writer,
            best_effort(),
        )]);
        let _watch = watcher.watch(vec![("rt/chatter".into(), Role::Reader, reliable())]);
        let first = rx.recv_timeout(Duration::from_secs(5)).unwrap();
        assert!(first.contains("reliability"), "{first}");
        assert!(rx.recv_timeout(Duration::from_millis(100)).is_err());
    }

    #[test]
    fn dropping_a_watch_unregisters_its_endpoints() {
        let (watcher, _rx) = quiet_watcher(vec![]);
        let kept = watcher.watch(vec![("rt/kept".into(), Role::Reader, reliable())]);
        let dropped = watcher.watch(vec![
            ("rt/chatter".into(), Role::Reader, reliable()),
            ("rt/other".into(), Role::Writer, reliable()),
        ]);
        assert_eq!(watched(&watcher).len(), 3);
        drop(dropped);
        assert_eq!(watched(&watcher), vec![("rt/kept".into(), Role::Reader)]);
        drop(kept);
        assert_eq!(watched(&watcher), vec![]);
    }

    #[test]
    fn an_endpoint_watched_twice_stays_until_both_watches_drop() {
        let (watcher, _rx) = quiet_watcher(vec![]);
        let first = watcher.watch(vec![("rt/chatter".into(), Role::Reader, reliable())]);
        let second = watcher.watch(vec![("rt/chatter".into(), Role::Reader, reliable())]);
        drop(first);
        assert_eq!(watched(&watcher), vec![("rt/chatter".into(), Role::Reader)]);
        drop(second);
        assert_eq!(watched(&watcher), vec![]);
    }

    #[test]
    fn a_watch_outliving_the_watcher_drops_quietly() {
        let (watcher, _rx) = quiet_watcher(vec![]);
        let watch = watcher.watch(vec![("rt/chatter".into(), Role::Reader, reliable())]);
        drop(watcher);
        drop(watch);
    }

    #[test]
    fn watched_qos_starts_from_ros2_client_defaults() {
        let (watcher, _rx) = quiet_watcher(vec![]);
        let _watch = watcher.watch(vec![
            ("rt/x".into(), Role::Writer, QosPolicies::qos_none()),
            ("rt/x".into(), Role::Reader, QosPolicies::qos_none()),
        ]);
        let locals = watcher.local_endpoints();
        let writer = QosPolicies::from(ros2_client::DEFAULT_PUBLISHER_QOS);
        let reader = QosPolicies::from(ros2_client::DEFAULT_SUBSCRIPTION_QOS);
        assert_eq!(locals[0].qos, writer);
        assert_eq!(locals[1].qos, reader);
    }

    #[test]
    fn service_topics_match_ros2_client_naming() {
        // A relative name goes under the node's namespace, like ros2-client
        // names it (`parse_ros2_name("ns/add_two_ints")` in the Python bridge).
        for name in ["/add_two_ints", "ns/add_two_ints"] {
            let name = Name::parse(name).unwrap();
            let request = name.to_dds_name("rq", &node(), "Request");
            let reply = name.to_dds_name("rr", &node(), "Reply");
            let (watcher, _rx) = quiet_watcher(vec![]);
            let qos = QosProfile::publisher_default();
            let _watch = watcher.watch_service(&node(), &name, true, &qos, &qos);
            assert_eq!(
                watched(&watcher),
                vec![(request, Role::Writer), (reply, Role::Reader)]
            );
        }
        assert_eq!(
            Name::parse("ns/add_two_ints")
                .unwrap()
                .to_dds_name("rq", &node(), "Request"),
            "rq/robot/ns/add_two_intsRequest"
        );
    }

    #[test]
    fn an_action_watches_three_services_and_two_topics_with_their_own_qos() {
        let (watcher, _rx) = quiet_watcher(vec![]);
        let qos = QosProfile::publisher_default();
        let status = qos
            .clone()
            .durability(ros2_client::qos::Durability::TransientLocal);
        let policies = ActionServerQosPolicies {
            goal_service: qos.clone(),
            result_service: qos.clone(),
            cancel_service: qos.clone(),
            feedback_publisher: qos.clone(),
            status_publisher: status,
        };
        // Relative, so it lands under the node's namespace.
        let name = Name::parse("fibonacci").unwrap();
        let _watch = watcher.watch_action_server(&node(), &name, &policies);
        let endpoints = watched(&watcher);
        assert_eq!(endpoints.len(), 8, "{endpoints:?}");
        for (topic, role) in [
            ("rq/robot/fibonacci/_action/send_goalRequest", Role::Reader),
            ("rr/robot/fibonacci/_action/get_resultReply", Role::Writer),
            (
                "rq/robot/fibonacci/_action/cancel_goalRequest",
                Role::Reader,
            ),
            ("rt/robot/fibonacci/_action/feedback", Role::Writer),
            ("rt/robot/fibonacci/_action/status", Role::Writer),
        ] {
            assert!(
                endpoints.contains(&(topic.into(), role)),
                "{topic} {role:?} in {endpoints:?}"
            );
        }
        let locals = watcher.local_endpoints();
        let durability = |suffix: &str| {
            let endpoint = locals.iter().find(|l| l.dds_topic.ends_with(suffix));
            endpoint.unwrap().qos.durability()
        };
        assert_eq!(
            durability("/status"),
            Some(policy::Durability::TransientLocal)
        );
        assert_eq!(durability("/feedback"), Some(policy::Durability::Volatile));
    }

    #[test]
    fn dropping_the_watcher_stops_its_thread_promptly() {
        let (tx, rx) = mpsc::channel::<()>();
        let watcher = DdsDiagnostics::start_with(
            move || {
                let _keep = &tx; // dropped with the thread's closure
                vec![]
            },
            ServiceMapping::Enhanced,
            None,
            Duration::from_secs(60),
            |_| {},
        );
        let started = std::time::Instant::now();
        drop(watcher);
        // The thread (and everything it owns, like the participant) is gone.
        assert_eq!(
            rx.recv_timeout(Duration::from_secs(1)),
            Err(mpsc::RecvTimeoutError::Disconnected)
        );
        assert!(started.elapsed() < Duration::from_secs(1));
    }

    /// Against real RustDDS discovery: a remote participant's best-effort
    /// writer is reported against dora's reliable reader, and the participant's
    /// own best-effort writer on the same topic is not; nor are several local
    /// addresses, since the remote participant runs on this host. Covers what the fakes
    /// above cannot: `remote_endpoints`, the own-prefix filter and the QoS that
    /// discovery reports.
    #[test]
    fn a_real_participant_reports_an_incompatible_remote_writer() {
        use crate::ros2_client::{Context, ContextOptions, MessageTypeName, NodeOptions};
        use rustdds::RTPSEntity;

        /// A domain no other test in this crate uses.
        const DOMAIN_ID: u16 = 88;
        let topic_name = Name::parse("/dora_diagnostics_chatter").unwrap();
        let best_effort = QosProfile::publisher_default().reliability_best_effort();
        let prefix_text = |context: &Context| {
            let bytes = context.domain_participant().guid().to_bytes();
            let prefix: Vec<String> = bytes[..12].iter().map(|b| format!("{b:02x}")).collect();
            prefix.join(".")
        };
        // A node with a best-effort writer on the topic.
        let writer_node = |context: &Context, name: &str| {
            let mut node = context
                .new_node(NodeName::new("/", name).unwrap(), NodeOptions::new())
                .unwrap();
            let topic = node
                .create_topic(
                    &topic_name,
                    MessageTypeName::new("std_msgs", "String"),
                    &best_effort,
                )
                .unwrap();
            let publisher = node
                .create_publisher::<String>(&topic, Some(best_effort.clone()))
                .unwrap();
            (node, publisher)
        };

        // A same-host peer is never reported as being elsewhere. The extra
        // address makes this hold on single-address hosts too, where the
        // check would otherwise not run.
        let mut local = crate::transport::interfaces::local_addrs();
        local.push(LocalAddr {
            name: "test0".into(),
            ip: Ipv4Addr::new(198, 51, 100, 1),
            loopback: false,
            multicast: true,
            up: true,
        });
        let unrestricted = Unrestricted::new(&local).expect("two or more addresses");

        let options = || ContextOptions::new().domain_id(DOMAIN_ID);
        let dora = Context::with_options(options()).unwrap();
        let remote = Context::with_options(options()).unwrap();
        let (tx, rx) = mpsc::channel();
        let watcher = DdsDiagnostics::start_reporting(
            dora.domain_participant(),
            ServiceMapping::Enhanced,
            Some(unrestricted),
            Duration::from_millis(100),
            move |message| {
                let _ = tx.send(message.to_owned());
            },
        );
        let _watch = watcher.watch(vec![(
            topic_name.to_dds_name("rt", &NodeName::new("/", "dora").unwrap(), ""),
            Role::Reader,
            reliable(),
        )]);
        let _own = writer_node(&dora, "dora_writer");
        let _remote = writer_node(&remote, "remote_writer");

        let first = rx
            .recv_timeout(Duration::from_secs(20))
            .expect("the remote writer is reported");
        assert!(
            first.starts_with("QoS mismatch on /dora_diagnostics_chatter"),
            "{first}"
        );
        assert!(first.contains("reliability"), "{first}");
        assert!(first.contains(&prefix_text(&remote)), "{first}");
        // The own writer has had as long as the remote one to show up.
        while let Ok(more) = rx.recv_timeout(Duration::from_secs(1)) {
            assert!(!more.contains(&prefix_text(&dora)), "own writer: {more}");
            assert!(!more.contains("network addresses"), "same host: {more}");
        }
    }
}
