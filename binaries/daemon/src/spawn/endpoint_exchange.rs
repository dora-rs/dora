//! Exchanging node zenoh endpoints between the daemons of one dataflow.
//!
//! A consumer can only dial a producer on another machine if it knows that
//! producer's endpoint *before* it opens its zenoh session: zenoh reads
//! `connect/endpoints` once at startup and never re-reads it, so an endpoint
//! learned later cannot be used at all. The daemon that owns a node is also the
//! only one that can reserve a port for it. So the endpoints have to travel
//! daemon-to-daemon, before either side spawns anything.
//!
//! Each daemon declares a queryable carrying its own local nodes' routable
//! endpoints, then queries for its peers'. A query rather than a publication on
//! purpose: it is pull-based, so a daemon that starts late still gets a complete
//! answer by asking again, where a publication that landed before the subscriber
//! existed is simply gone (the memory-pool path documents exactly that race).
//!
//! Whatever is still missing when the deadline expires is not an error: those
//! edges keep the daemon-forwarded path they use today, which is slower but
//! lossless. That makes this an optimization that degrades, never a barrier that
//! can fail a dataflow.
//!
//! The daemon path has one precondition the exchange can check for free: the
//! two daemons' zenoh sessions must be linked at all. Every daemon in the
//! dataflow declares its queryable, even with nothing to announce, so a reply
//! is proof of the link and a daemon that never replies is one this daemon
//! cannot reach. Nothing else in the daemon detects that state — the producer's
//! sends succeed, the consumer just never hears anything, and the dataflow never
//! finishes — so a daemon still silent once every daemon has spawned is warned
//! about and probed for a while (see [`ExchangeHandle::check_link`] and
//! [`LINK_PROBE_DEADLINE`]). A reply is attributed to a daemon by the key
//! it answers on, which names the daemon; see [`Placement`] for what this
//! daemon can and cannot expect from the descriptor alone.

use std::{
    collections::{BTreeMap, BTreeSet},
    fmt,
    time::Duration,
};

use dora_message::{
    common::{DaemonId, LogLevel},
    descriptor::ResolvedNode,
    id::NodeId,
};
use tokio_util::task::AbortOnDropHandle;
use uuid::Uuid;
use zenoh::Wait;

use crate::log::DataflowLogger;

/// How long to keep asking peers for endpoints before spawning anyway.
///
/// Paid only by dataflows that actually span machines, and only once per spawn.
/// Short on purpose: this runs on the daemon's event loop, and the fallback —
/// the daemon-forwarded path — is correct, just slower.
const DEFAULT_TIMEOUT: Duration = Duration::from_millis(1500);

/// Env var overriding [`DEFAULT_TIMEOUT`], in milliseconds.
///
/// A slow or congested link may need longer; `0` skips collecting peers'
/// endpoints and announces none of this daemon's, so every cross-machine edge
/// touching this daemon keeps the daemon path. The daemon still answers
/// queries (with no endpoints), since a peer reads the answer as proof that
/// the zenoh link exists.
const TIMEOUT_ENV: &str = "DORA_ZENOH_ENDPOINT_EXCHANGE_TIMEOUT_MS";

/// How long to wait for replies to a single query before asking again.
const QUERY_ROUND: Duration = Duration::from_millis(100);

/// How long a daemon that did not answer within the exchange budget keeps being
/// probed, counted from when every daemon has spawned, before the missing link
/// is reported as an error.
///
/// Long enough for multicast scouting to catch up (a second or two,
/// ordinarily), short enough that an operator watching a
/// dataflow that "does nothing" gets the diagnosis before giving up.
const LINK_PROBE_DEADLINE: Duration = Duration::from_secs(30);

/// Pause between link probes.
const LINK_PROBE_INTERVAL: Duration = Duration::from_secs(2);

/// How long one link probe collects answers, given the exchange `budget`. At
/// least [`DEFAULT_TIMEOUT`]: the probe runs after spawn, off the event loop,
/// so lowering the exchange budget to spawn faster must not make probe rounds
/// shorter than the round-trip time and report a working link as missing.
fn link_probe_round(budget: Duration) -> Duration {
    budget.max(DEFAULT_TIMEOUT)
}

/// Zenoh key a daemon answers its local nodes' endpoints on.
///
/// The daemon id is sanitized into one key chunk: a machine id is operator
/// input and may contain characters (`/`, `*`) that would silently reshape the
/// key expression. The id's uuid suffix survives sanitization untouched, so
/// distinct daemons keep distinct keys even if their machine ids collapse to
/// the same sanitized form.
fn endpoints_key(dataflow_id: Uuid, daemon_id: &DaemonId) -> String {
    let daemon = sanitize_chunk(&daemon_id.to_string());
    format!("dora/default/{dataflow_id}/node-endpoints/{daemon}")
}

/// Map one key-expression chunk onto the characters zenoh accepts verbatim.
fn sanitize_chunk(raw: &str) -> String {
    raw.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' || c == '.' || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// Selector matching every daemon's endpoints key for this dataflow.
fn endpoints_selector(dataflow_id: Uuid) -> String {
    format!("dora/default/{dataflow_id}/node-endpoints/*")
}

fn timeout() -> Duration {
    match std::env::var(TIMEOUT_ENV).ok().and_then(|v| v.parse().ok()) {
        Some(ms) => Duration::from_millis(ms),
        None => DEFAULT_TIMEOUT,
    }
}

/// A daemon's answer: the routable endpoint of each of its local nodes that a
/// remote consumer may need to dial.
type Endpoints = BTreeMap<NodeId, String>;

/// Where a remote node runs, as far as this daemon can tell from the
/// descriptor — which decides whether a reply can be attributed to its daemon.
///
/// The coordinator places nodes by `deploy.machine`, then by `deploy.labels`,
/// then on the unnamed daemon. A daemon answers on a key that carries its own
/// id (`<machine>-<uuid>`, or a bare uuid for an unnamed one), so the first
/// and last placements are recognizable in a reply; a label-routed node's
/// daemon is not, and such a node can only be waited for until its endpoint
/// turns up.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum Placement {
    /// `deploy.machine` names the daemon.
    Machine(String),
    /// Neither machine nor labels: the unnamed daemon.
    Unnamed,
    /// Routed by `deploy.labels` to a daemon this daemon cannot name.
    Unknown,
}

impl Placement {
    pub fn of(node: &ResolvedNode) -> Self {
        match &node.deploy {
            Some(deploy) if deploy.machine.is_some() => {
                Placement::Machine(deploy.machine.clone().unwrap_or_default())
            }
            Some(deploy) if !deploy.labels.is_empty() => Placement::Unknown,
            _ => Placement::Unnamed,
        }
    }

    /// Whether a reply on the key chunk `chunk` came from this placement's
    /// daemon.
    ///
    /// The chunk is the daemon id sanitized as a whole; the uuid survives
    /// sanitizing untouched, so it parses back into a daemon id whose machine
    /// part is the sanitized machine id. Two machine ids that sanitize alike
    /// (`a/b` and `a_b`) are told apart by nothing here — a name that needs
    /// sanitizing is already one the key expression could not carry.
    fn answered_by(&self, chunk: &str) -> bool {
        let Some(id) = DaemonId::from_display_str(chunk) else {
            return false;
        };
        match self {
            Placement::Machine(machine) => id.matches_machine_id(&sanitize_chunk(machine)),
            Placement::Unnamed => id.machine_id().is_none(),
            Placement::Unknown => false,
        }
    }
}

impl fmt::Display for Placement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Placement::Machine(machine) => write!(f, "machine `{machine}`"),
            Placement::Unnamed => f.write_str("the unnamed daemon"),
            Placement::Unknown => f.write_str("a daemon chosen by labels"),
        }
    }
}

/// The remote nodes a daemon needs endpoints for, each with where it runs.
pub type Wanted = BTreeMap<NodeId, Placement>;

/// The placements in `expected` that no reply in `answered` came from.
fn unanswered(expected: &BTreeSet<Placement>, answered: &BTreeSet<String>) -> BTreeSet<Placement> {
    expected
        .iter()
        .filter(|placement| !answered.iter().any(|chunk| placement.answered_by(chunk)))
        .cloned()
        .collect()
}

fn list(placements: &BTreeSet<Placement>) -> String {
    placements
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(", ")
}

/// What the exchange leaves running for one dataflow.
///
/// Held for the dataflow's lifetime rather than just the exchange: a daemon
/// that joins late (a restarted node, a second `dora start` against the same
/// graph) asks the same question, and an undeclared queryable answers nothing.
/// Dropped in `finish_dataflow`; an abandoned handle would keep the queryable,
/// its session clone and its payload alive for the daemon's whole lifetime,
/// and the link probe logging about a dataflow that is already gone.
pub struct ExchangeHandle {
    /// Declares the queryable, and keeps it declared until aborted.
    _queryable: AbortOnDropHandle<()>,
    /// Daemons that did not answer during the exchange, waiting for
    /// [`ExchangeHandle::check_link`].
    link_check: Option<LinkCheck>,
    _link_probe: Option<AbortOnDropHandle<()>>,
}

/// What [`probe_link`] needs, parked until every daemon has spawned.
struct LinkCheck {
    session: zenoh::Session,
    dataflow_id: Uuid,
    missing: BTreeSet<Placement>,
    reporter: Reporter,
}

/// What [`exchange`] found out, separated from its logging so it can be
/// tested.
#[derive(Debug, Default, PartialEq, Eq)]
struct Outcome {
    /// Endpoints of the wanted remote nodes that a peer announced.
    found: Endpoints,
    /// Daemons that never answered: no zenoh link to them (yet).
    unanswered: BTreeSet<Placement>,
}

/// Where the exchange reports: the daemon log, and the dataflow's own log
/// when there is one, since that is what `dora start` and `dora logs` show
/// while the daemon's tracing output goes to a file nobody is watching.
struct Reporter(Option<DataflowLogger<'static>>);

impl Reporter {
    async fn report(&mut self, level: LogLevel, message: String) {
        match level {
            LogLevel::Error => tracing::error!("{message}"),
            LogLevel::Warn => tracing::warn!("{message}"),
            _ => tracing::info!("{message}"),
        }
        if let Some(logger) = &mut self.0 {
            logger
                .log(level, None, Some("daemon".into()), message)
                .await;
        }
    }
}

/// Publish this daemon's node endpoints and collect the peers' — see the module
/// docs.
///
/// `local` maps each local node to the endpoint remote consumers should dial;
/// `wanted` names the remote nodes this daemon's own consumers need, and
/// `peers` where the remote nodes this daemon consumes from run, so the daemons
/// expected to answer are known. Returns the endpoints found (a subset of
/// `wanted`) plus the handle to keep alive.
///
/// Never fails the spawn: a zenoh error, an unanswered query or an expired
/// deadline all resolve to "fewer endpoints than asked for", which leaves those
/// edges on the daemon-forwarded path. A daemon that did not answer at all is
/// kept for [`ExchangeHandle::check_link`], since the daemon path needs the
/// link too.
pub async fn exchange(
    session: zenoh::Session,
    dataflow_id: Uuid,
    daemon_id: DaemonId,
    local: Endpoints,
    wanted: Wanted,
    peers: BTreeSet<Placement>,
    logger: Option<DataflowLogger<'static>>,
) -> (Endpoints, Option<ExchangeHandle>) {
    exchange_within(
        timeout(),
        session,
        dataflow_id,
        daemon_id,
        local,
        wanted,
        peers,
        logger,
    )
    .await
}

/// [`exchange`] with an explicit budget instead of the env var.
#[allow(clippy::too_many_arguments)]
async fn exchange_within(
    budget: Duration,
    session: zenoh::Session,
    dataflow_id: Uuid,
    daemon_id: DaemonId,
    local: Endpoints,
    wanted: Wanted,
    peers: BTreeSet<Placement>,
    logger: Option<DataflowLogger<'static>>,
) -> (Endpoints, Option<ExchangeHandle>) {
    if budget.is_zero() {
        // The documented off switch: every cross-machine edge keeps the daemon
        // path, and nothing is asked. Still answer — with no endpoints, so no
        // peer dials a node here directly — because a peer with the exchange
        // on reads silence as a missing link and reports that the dataflow
        // can never finish (dora-rs/dora#3603).
        let (queryable, _declared) =
            spawn_declare(session, dataflow_id, daemon_id, Endpoints::new());
        return (
            BTreeMap::new(),
            Some(ExchangeHandle {
                _queryable: queryable,
                link_check: None,
                _link_probe: None,
            }),
        );
    }
    let started = tokio::time::Instant::now();

    // `declare_queryable` is itself a zenoh operation that can block on a
    // degraded inter-daemon link, so it gets a deadline too — bounding only the
    // query would leave the caller waiting with no limit at all, on the path
    // that must finish before any node spawns. Past the deadline the declare
    // keeps going in the background: until it lands, peers that probe the link
    // would see this daemon as unreachable (dora-rs/dora#3603).
    let (queryable, declared) = spawn_declare(session.clone(), dataflow_id, daemon_id, local);
    if tokio::time::timeout(budget, declared).await.is_err() {
        tracing::warn!(
            "declaring the zenoh node-endpoint queryable did not finish within \
             {budget:?}; consumers on other daemons will receive this daemon's \
             outputs over the daemon path. Still declaring it in the background"
        );
    }

    let remaining = budget.saturating_sub(started.elapsed());
    let expected: BTreeSet<Placement> = peers
        .into_iter()
        .filter(|placement| *placement != Placement::Unknown)
        .collect();
    if (wanted.is_empty() && expected.is_empty()) || remaining.is_zero() {
        // Nothing to collect — but stay answerable, since peers that consume
        // from this daemon's nodes still need what was just declared.
        return (
            BTreeMap::new(),
            Some(ExchangeHandle {
                _queryable: queryable,
                link_check: None,
                _link_probe: None,
            }),
        );
    }

    let Outcome { found, unanswered } =
        run(&session, dataflow_id, &wanted, &expected, remaining).await;

    if !found.is_empty() {
        // The observable that says the mesh formed: every endpoint here is a
        // cross-machine edge that can now skip both daemons.
        tracing::debug!(
            "resolved {}/{} remote node zenoh endpoints: {:?}",
            found.len(),
            wanted.len(),
            found,
        );
    }
    // Nodes without an endpoint fall in two groups: those whose daemon
    // answered, which therefore have no listener a remote consumer could dial
    // (a daemon bound to loopback reserves none — the normal shape of two
    // daemons on one host), and those whose daemon cannot be named, whose
    // reply cannot be told apart from silence.
    let (explained, unresolved): (Vec<_>, Vec<_>) = wanted
        .iter()
        .filter(|(id, _)| !found.contains_key(*id))
        .partition(|(_, placement)| **placement != Placement::Unknown);
    let explained: Vec<&NodeId> = explained.into_iter().map(|(id, _)| id).collect();
    let unresolved: Vec<&NodeId> = unresolved.into_iter().map(|(id, _)| id).collect();
    if !explained.is_empty() && unanswered.is_empty() {
        tracing::info!(
            "remote node(s) {explained:?} have no routable zenoh endpoint; their \
             outputs reach this daemon's nodes over the daemon path instead of \
             directly"
        );
    }
    if !unresolved.is_empty() {
        tracing::warn!(
            "no zenoh endpoint for remote node(s) {unresolved:?} after {budget:?}; \
             their outputs will reach this daemon's nodes over the daemon path \
             instead of directly (set {TIMEOUT_ENV} to allow longer). Their daemon \
             is chosen by labels, so whether it is reachable at all cannot be told \
             from here"
        );
    }
    // Silence here is not yet a diagnosis: the coordinator spawns daemons one
    // at a time in daemon-id order, not producer before consumer, so a
    // producer's daemon may simply not have reached this dataflow yet. The
    // check is held until every daemon has spawned (`check_link`).
    let link_check = if unanswered.is_empty() {
        None
    } else {
        tracing::debug!(
            "{} did not answer the zenoh node-endpoint query within {budget:?}; \
             checking the link once every daemon of the dataflow has spawned",
            list(&unanswered)
        );
        Some(LinkCheck {
            session: session.clone(),
            dataflow_id,
            missing: unanswered,
            reporter: Reporter(logger),
        })
    };
    (
        found,
        Some(ExchangeHandle {
            _queryable: queryable,
            link_check,
            _link_probe: None,
        }),
    )
}

impl ExchangeHandle {
    /// Start probing the daemons that did not answer during the exchange.
    ///
    /// Called on `AllNodesReady`: that arrives over the coordinator's
    /// WebSocket, so it does not depend on the zenoh link, and by then every
    /// daemon of the dataflow has spawned and declared its queryable — silence
    /// can only mean there is no link. Idempotent, since the coordinator
    /// replays `AllNodesReady` to a daemon that reconnects.
    pub fn check_link(&mut self) {
        if let Some(check) = self.link_check.take() {
            self._link_probe = Some(AbortOnDropHandle::new(tokio::spawn(probe_link(
                check.session,
                check.dataflow_id,
                check.missing,
                check.reporter,
            ))));
        }
    }

    /// Release the handle of a dataflow whose nodes on this daemon have all
    /// finished, but keep answering for as long as a peer may still probe.
    ///
    /// A producer can finish within moments of `AllNodesReady`, before a
    /// consumer's daemon has sent its first probe; dropping the queryable at
    /// once would make a linked daemon look unreachable, and the consumer
    /// would log a missing link for data that did arrive. This daemon's own
    /// probe stops here, since the dataflow is done on this side.
    pub fn linger(self) {
        let queryable = self._queryable;
        tokio::spawn(async move {
            tokio::time::sleep(LINK_PROBE_DEADLINE + timeout()).await;
            drop(queryable);
        });
    }
}

/// Collect for up to `deadline`, then sort out what came back.
async fn run(
    session: &zenoh::Session,
    dataflow_id: Uuid,
    wanted: &Wanted,
    expected: &BTreeSet<Placement>,
    deadline: Duration,
) -> Outcome {
    // Bounded twice over: `collect` stops asking at its deadline, and the
    // timeout covers a single `get` that never resolves. The outer bound is
    // given one round of slack so it cannot beat `collect`'s own deadline by a
    // timer tick and discard the answers that did arrive — which would turn
    // every daemon that answered into an "unanswered" one. The queryable
    // survives either way, so peers keep getting answers even when this daemon
    // gave up asking.
    let (found, answered) = tokio::time::timeout(
        deadline + QUERY_ROUND,
        collect(session, dataflow_id, wanted, expected, deadline),
    )
    .await
    .unwrap_or_default();
    Outcome {
        found,
        unanswered: unanswered(expected, &answered),
    }
}

/// Ask the daemons that did not answer during the exchange again, now that all
/// of them have spawned, and say how it ended: answered at once (they were
/// merely spawned after this daemon — nothing to report), linked late (the
/// usual case with multicast scouting, which takes a moment), or never.
async fn probe_link(
    session: zenoh::Session,
    dataflow_id: Uuid,
    mut missing: BTreeSet<Placement>,
    mut reporter: Reporter,
) {
    let started = tokio::time::Instant::now();
    let no_nodes = Wanted::new();
    let mut warned = false;
    loop {
        missing = run(
            &session,
            dataflow_id,
            &no_nodes,
            &missing,
            link_probe_round(timeout()),
        )
        .await
        .unanswered;
        if missing.is_empty() {
            if warned {
                reporter
                    .report(
                        LogLevel::Info,
                        format!(
                            "zenoh link to the other daemon(s) of this dataflow established \
                             after {:?}",
                            started.elapsed()
                        ),
                    )
                    .await;
            }
            return;
        }
        if !warned {
            warned = true;
            reporter
                .report(
                    LogLevel::Warn,
                    format!(
                        "{} does not answer the zenoh node-endpoint query although every \
                         daemon of this dataflow has spawned: this daemon has no zenoh link \
                         to it (yet). Without the link nothing its nodes send can reach the \
                         nodes here — not even over the daemon path — and the dataflow \
                         cannot finish. Probing again for up to {LINK_PROBE_DEADLINE:?}",
                        list(&missing)
                    ),
                )
                .await;
        }
        if started.elapsed() >= LINK_PROBE_DEADLINE {
            reporter
                .report(
                    LogLevel::Error,
                    format!(
                        "still no zenoh link to {} after {:?}: this dataflow's inputs from \
                         its nodes will never arrive and it will not finish. Daemons on one \
                         host link through the coordinator as long as both reach it over \
                         loopback; daemons on different hosts need the coordinator on a \
                         routable address (`dora up --interface`), \
                         `--zenoh-peer`/`--zenoh-listen`, or working multicast — see \
                         docs/multi-machine.md. (A daemon from a dora release before this \
                         check existed answers only when it has an endpoint to announce, \
                         so with mixed versions this can also be a false alarm.)",
                        list(&missing),
                        started.elapsed()
                    ),
                )
                .await;
            return;
        }
        tokio::time::sleep(LINK_PROBE_INTERVAL).await;
    }
}

/// Declare the queryable on its own task, which keeps it declared until the
/// returned handle is dropped. The receiver resolves once the declare has
/// finished, successfully or not.
fn spawn_declare(
    session: zenoh::Session,
    dataflow_id: Uuid,
    daemon_id: DaemonId,
    local: Endpoints,
) -> (AbortOnDropHandle<()>, tokio::sync::oneshot::Receiver<()>) {
    spawn_holding(async move {
        // The session is held with the queryable: if this is its last
        // handle, dropping it closes the session and undeclares the
        // queryable with it.
        let queryable = declare(&session, dataflow_id, &daemon_id, local).await?;
        Some((session, queryable))
    })
}

/// Run `declaring` on its own task and hold what it declares until the
/// returned handle is dropped — whether or not anyone still waits for the
/// receiver, which resolves once `declaring` has finished.
fn spawn_holding<T: Send + 'static>(
    declaring: impl Future<Output = Option<T>> + Send + 'static,
) -> (AbortOnDropHandle<()>, tokio::sync::oneshot::Receiver<()>) {
    let (declared_tx, declared) = tokio::sync::oneshot::channel();
    let task = tokio::spawn(async move {
        let queryable = declaring.await;
        let _ = declared_tx.send(());
        if queryable.is_some() {
            // Hold it until the handle is dropped, which aborts this task and
            // undeclares the queryable with it.
            std::future::pending::<()>().await;
        }
    });
    (AbortOnDropHandle::new(task), declared)
}

/// Declare the queryable that answers this daemon's endpoints.
///
/// Declared even when `local` is empty: peers read a reply — any reply — as
/// proof that this daemon is reachable, and a silent daemon is reported as
/// unlinked (see the module docs).
async fn declare(
    session: &zenoh::Session,
    dataflow_id: Uuid,
    daemon_id: &DaemonId,
    local: Endpoints,
) -> Option<zenoh::query::Queryable<()>> {
    let key = endpoints_key(dataflow_id, daemon_id);
    let payload = match serde_json::to_vec(&local) {
        Ok(payload) => payload,
        Err(err) => {
            tracing::warn!("failed to serialize local zenoh node endpoints: {err}");
            return None;
        }
    };
    let reply_key = key.clone();
    let queryable = session
        .declare_queryable(key.clone())
        .complete(true)
        .callback(move |query| {
            if let Err(err) = query.reply(reply_key.clone(), payload.clone()).wait() {
                tracing::warn!("failed to answer a zenoh node-endpoint query: {err}");
            }
        })
        .await;
    match queryable {
        Ok(queryable) => Some(queryable),
        Err(err) => {
            // Peers keep their cross-machine edges on the daemon path, which is
            // where they are today.
            tracing::warn!("failed to declare the zenoh node-endpoint queryable on {key}: {err}");
            None
        }
    }
}

/// Query peers until the answers are settled or the deadline expires.
///
/// Settled means every expected daemon has answered — a daemon answering
/// without a wanted endpoint has none to give, so waiting longer cannot help —
/// and every wanted node whose daemon cannot be named has been found.
///
/// Returns the endpoints found and the daemon-id chunks of the keys that
/// answered, the latter being what tells a linked daemon with nothing to
/// announce from a daemon this session cannot reach.
async fn collect(
    session: &zenoh::Session,
    dataflow_id: Uuid,
    wanted: &Wanted,
    expected: &BTreeSet<Placement>,
    deadline: Duration,
) -> (Endpoints, BTreeSet<String>) {
    let selector = endpoints_selector(dataflow_id);
    let started = tokio::time::Instant::now();
    let mut found: Endpoints = BTreeMap::new();
    let mut answered: BTreeSet<String> = BTreeSet::new();
    while started.elapsed() < deadline {
        // `timeout(QUERY_ROUND)` bounds how long a round *may* take, not how
        // long it does: a peer whose queryable is already declared answers at
        // once, so without pacing the loop would re-issue the query as fast as
        // replies arrive and turn the retry into a query storm on the
        // inter-daemon session. Round start is captured here and slept out at
        // the bottom, so each round costs one `QUERY_ROUND` regardless. The
        // last round is cut to what is left of the deadline, so a slow reply
        // cannot push the whole collection past it.
        let round_started = tokio::time::Instant::now();
        let round = QUERY_ROUND.min(deadline.saturating_sub(started.elapsed()));
        // A peer that has not processed its own spawn yet has no queryable to
        // answer with, so an empty round means "ask again", not "there is
        // nobody". `ConsolidationMode::None` because every daemon answers on
        // its own key and consolidation would be free to keep just one reply
        // per key — a silent way to lose a whole daemon's endpoints.
        let replies = session
            .get(&selector)
            .consolidation(zenoh::query::ConsolidationMode::None)
            .timeout(round)
            .await;
        match replies {
            Ok(replies) => {
                while let Ok(reply) = replies.recv_async().await {
                    let Ok(sample) = reply.result() else { continue };
                    if let Some(chunk) = sample.key_expr().as_str().rsplit('/').next() {
                        answered.insert(chunk.to_owned());
                    }
                    let payload = sample.payload().to_bytes();
                    match serde_json::from_slice::<Endpoints>(&payload) {
                        Ok(endpoints) => found.extend(
                            endpoints
                                .into_iter()
                                .filter(|(node_id, _)| wanted.contains_key(node_id)),
                        ),
                        Err(err) => {
                            tracing::warn!("ignoring malformed zenoh node-endpoint reply: {err}")
                        }
                    }
                }
            }
            Err(err) => {
                tracing::warn!("zenoh node-endpoint query failed: {err}");
                return (found, answered);
            }
        }
        let settled = wanted
            .iter()
            .all(|(node, placement)| *placement != Placement::Unknown || found.contains_key(node));
        if settled && unanswered(expected, &answered).is_empty() {
            break;
        }
        // Pace the next round. Capped at the remaining budget so pacing can
        // never push the exchange past `deadline` — the caller awaits this on
        // the daemon event loop, so overrunning would stall it further.
        let elapsed = started.elapsed();
        let Some(remaining) = deadline.checked_sub(elapsed) else {
            break;
        };
        let till_next_round = QUERY_ROUND.saturating_sub(round_started.elapsed());
        tokio::time::sleep(till_next_round.min(remaining)).await;
    }
    (found, answered)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn uuid() -> Uuid {
        Uuid::from_u128(0x1234_5678_9abc_def0_1234_5678_9abc_def0)
    }

    /// The selector must match the keys daemons answer on — a mismatch would
    /// look exactly like "no peer replied", i.e. a silent fallback to the
    /// daemon path on every cross-machine edge.
    #[test]
    fn the_selector_matches_a_daemon_key() {
        let key = endpoints_key(uuid(), &DaemonId::new(Some("edge-01".into())));
        let selector = endpoints_selector(uuid());
        let prefix = selector.strip_suffix('*').expect("selector ends in `*`");
        assert!(key.starts_with(prefix), "{key} does not match {selector}");
        // One chunk after the prefix: a `/` in the daemon id would push the key
        // a level deeper, where a single-`*` selector no longer matches it.
        assert!(
            !key[prefix.len()..].contains('/'),
            "key has extra chunks: {key}"
        );
    }

    /// Distinct daemons need distinct keys, or their replies collide.
    #[test]
    fn sanitizing_a_machine_id_keeps_daemons_distinct() {
        let hostile = DaemonId::new(Some("a/b*c".into()));
        let plain = DaemonId::new(Some("a_b_c".into()));
        let hostile_key = endpoints_key(uuid(), &hostile);
        assert!(!hostile_key.contains('/') || hostile_key.matches('/').count() == 4);
        assert!(!hostile_key.contains('*'));
        assert_ne!(hostile_key, endpoints_key(uuid(), &plain));
        // Same daemon, same key, every time it is asked.
        assert_eq!(hostile_key, endpoints_key(uuid(), &hostile));
    }

    /// `0` is the documented way to switch the exchange off; anything
    /// unparseable falls back to the default rather than to no waiting at all.
    #[test]
    fn timeout_falls_back_to_the_default() {
        // Reading the real env var here would race other tests in this binary,
        // so exercise the same parse the getter uses.
        let parse = |v: Option<&str>| match v.and_then(|v| v.parse().ok()) {
            Some(ms) => Duration::from_millis(ms),
            None => DEFAULT_TIMEOUT,
        };
        assert_eq!(parse(None), DEFAULT_TIMEOUT);
        assert_eq!(parse(Some("nonsense")), DEFAULT_TIMEOUT);
        assert_eq!(parse(Some("0")), Duration::ZERO);
        assert_eq!(parse(Some("5000")), Duration::from_secs(5));
    }

    fn chunk_of(daemon_id: &DaemonId) -> String {
        sanitize_chunk(&daemon_id.to_string())
    }

    /// A reply key names the answering daemon; a placement recognizes exactly
    /// its own daemon's key.
    #[test]
    fn a_reply_chunk_is_attributed_to_its_placement() {
        let a = chunk_of(&DaemonId::new(Some("A".into())));
        assert!(Placement::Machine("A".into()).answered_by(&a));
        assert!(!Placement::Machine("B".into()).answered_by(&a));
        assert!(!Placement::Unnamed.answered_by(&a));
        assert!(!Placement::Unknown.answered_by(&a));
        // A machine id that is a prefix of another's must not claim its daemon.
        let ab = chunk_of(&DaemonId::new(Some("a-b".into())));
        assert!(!Placement::Machine("a".into()).answered_by(&ab));
        // Sanitized the same way on both sides.
        let hostile = chunk_of(&DaemonId::new(Some("a/b*c".into())));
        assert!(Placement::Machine("a/b*c".into()).answered_by(&hostile));
        // An unnamed daemon's chunk is a bare uuid.
        let unnamed = chunk_of(&DaemonId::new(None));
        assert!(Placement::Unnamed.answered_by(&unnamed));
        assert!(!Placement::Machine("A".into()).answered_by(&unnamed));
        assert!(!Placement::Unknown.answered_by(&unnamed));
    }

    #[test]
    fn unanswered_is_what_no_reply_accounted_for() {
        let expected: BTreeSet<Placement> =
            [Placement::Machine("A".into()), Placement::Unnamed].into();
        let answered: BTreeSet<String> = [chunk_of(&DaemonId::new(Some("A".into())))].into();
        assert_eq!(
            unanswered(&expected, &answered),
            BTreeSet::from([Placement::Unnamed])
        );
        assert!(unanswered(&BTreeSet::new(), &answered).is_empty());
    }

    /// Two sessions on loopback with multicast off, linked only by an explicit
    /// dial — the shape the coordinator now sets up for two daemons on one
    /// host.
    async fn linked_sessions() -> (zenoh::Session, zenoh::Session) {
        let endpoint =
            dora_core::topics::reserve_loopback_zenoh_endpoint().expect("reserve a loopback port");
        let mut listener = zenoh::Config::default();
        listener
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        listener
            .insert_json5("listen/endpoints", &format!("[{endpoint:?}]"))
            .unwrap();
        let a = zenoh::open(listener).await.expect("open listening session");
        let mut dialer = zenoh::Config::default();
        dialer
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        dialer
            .insert_json5("connect/endpoints", &format!("[{endpoint:?}]"))
            .unwrap();
        let b = zenoh::open(dialer).await.expect("open dialing session");
        (a, b)
    }

    fn wanted(node: &str, placement: Placement) -> Wanted {
        [(NodeId::from(node.to_string()), placement)].into()
    }

    fn on_machine(machine: &str) -> BTreeSet<Placement> {
        [Placement::Machine(machine.into())].into()
    }

    /// A daemon with nothing to announce still answers, and that answer is what
    /// tells the asking daemon the link exists: no endpoint for the node, but
    /// nothing unanswered either — and no waiting for the deadline.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_linked_daemon_with_no_endpoints_counts_as_answered() {
        let (a, b) = linked_sessions().await;
        let dataflow = uuid();
        let _queryable = declare(
            &a,
            dataflow,
            &DaemonId::new(Some("A".into())),
            BTreeMap::new(),
        )
        .await
        .expect("declare on the listening session");

        let started = tokio::time::Instant::now();
        let outcome = run(
            &b,
            dataflow,
            &wanted("n", Placement::Machine("A".into())),
            &on_machine("A"),
            Duration::from_secs(5),
        )
        .await;

        assert_eq!(outcome, Outcome::default());
        assert!(started.elapsed() < Duration::from_secs(4), "settled early");
    }

    /// The unnamed daemon (`dora up`) is recognized by its bare-uuid key.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn the_unnamed_daemon_is_recognized() {
        let (a, b) = linked_sessions().await;
        let dataflow = uuid();
        let _queryable = declare(&a, dataflow, &DaemonId::new(None), BTreeMap::new())
            .await
            .expect("declare on the listening session");

        let outcome = run(
            &b,
            dataflow,
            &wanted("n", Placement::Unnamed),
            &[Placement::Unnamed].into(),
            Duration::from_secs(5),
        )
        .await;

        assert_eq!(outcome, Outcome::default());
    }

    /// The failure this module reports: the other daemon never answers. Here
    /// because it declared nothing; in production because the sessions never
    /// linked. Either way the daemon is named as unanswered.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_silent_daemon_is_reported_as_unanswered() {
        let (_a, b) = linked_sessions().await;

        let outcome = run(
            &b,
            uuid(),
            &wanted("n", Placement::Machine("A".into())),
            &on_machine("A"),
            Duration::from_millis(400),
        )
        .await;

        assert_eq!(
            outcome,
            Outcome {
                found: BTreeMap::new(),
                unanswered: on_machine("A"),
            }
        );
    }

    /// The exchange's original purpose still works through the same reply.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn an_announced_endpoint_is_found() {
        let (a, b) = linked_sessions().await;
        let dataflow = uuid();
        let local: Endpoints = [(
            NodeId::from("n".to_string()),
            "tcp/10.0.2.7:7447".to_string(),
        )]
        .into();
        let _queryable = declare(
            &a,
            dataflow,
            &DaemonId::new(Some("A".into())),
            local.clone(),
        )
        .await
        .expect("declare on the listening session");

        let outcome = run(
            &b,
            dataflow,
            &wanted("n", Placement::Machine("A".into())),
            &on_machine("A"),
            Duration::from_secs(5),
        )
        .await;

        assert_eq!(
            outcome,
            Outcome {
                found: local,
                unanswered: BTreeSet::new(),
            }
        );
    }

    /// A daemon with the exchange switched off still answers the link probe
    /// of a peer that has it on — with no endpoints — or that peer would
    /// report a working link as missing (dora-rs/dora#3603).
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_daemon_with_the_exchange_off_still_answers() {
        let (a, b) = linked_sessions().await;
        let dataflow = uuid();
        let (found, handle) = exchange_within(
            Duration::ZERO,
            a,
            dataflow,
            DaemonId::new(Some("A".into())),
            [(NodeId::from("n".to_string()), "tcp/10.0.0.1:1".to_string())].into(),
            Wanted::new(),
            on_machine("B"),
            None,
        )
        .await;
        assert!(found.is_empty());
        let _handle = handle.expect("the off switch still keeps the queryable declared");

        let (found, answered) = collect(
            &b,
            dataflow,
            &wanted("n", Placement::Machine("A".into())),
            &on_machine("A"),
            DEFAULT_TIMEOUT,
        )
        .await;
        assert!(
            unanswered(&on_machine("A"), &answered).is_empty(),
            "daemon A must answer: {answered:?}"
        );
        assert!(found.is_empty(), "the off switch announces no endpoints");
    }

    /// Lowering the exchange budget to spawn faster must not shorten the
    /// link probe's rounds below what a WAN round trip needs; raising it
    /// lengthens them.
    #[test]
    fn a_short_exchange_budget_does_not_shorten_the_link_probe_round() {
        assert_eq!(link_probe_round(Duration::ZERO), DEFAULT_TIMEOUT);
        assert_eq!(link_probe_round(Duration::from_millis(50)), DEFAULT_TIMEOUT);
        let long = DEFAULT_TIMEOUT * 4;
        assert_eq!(link_probe_round(long), long);
    }

    /// A declare that outlives the exchange budget keeps going in the
    /// background: `exchange_within` stops waiting for it, yet the queryable
    /// still gets declared and answers peers for as long as the handle lives,
    /// and stops once it is dropped (dora-rs/dora#3603). The declare is held
    /// back on purpose, since one on loopback finishes before any budget
    /// could run out.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_declare_past_the_budget_still_answers() {
        let (a, b) = linked_sessions().await;
        let dataflow = uuid();
        let (handle, declared) = spawn_holding(async move {
            tokio::time::sleep(Duration::from_millis(300)).await;
            let queryable = declare(
                &a,
                dataflow,
                &DaemonId::new(Some("A".into())),
                Endpoints::new(),
            )
            .await?;
            Some((a, queryable))
        });
        // What `exchange_within` does with its budget.
        assert!(
            tokio::time::timeout(Duration::from_millis(10), declared)
                .await
                .is_err(),
            "the declare must still be running when the budget runs out"
        );

        let (_, answered) = collect(
            &b,
            dataflow,
            &Wanted::new(),
            &on_machine("A"),
            Duration::from_secs(5),
        )
        .await;
        assert!(
            unanswered(&on_machine("A"), &answered).is_empty(),
            "the declare must finish after its waiter gave up: {answered:?}"
        );

        drop(handle);
        let (_, answered) = collect(
            &b,
            dataflow,
            &Wanted::new(),
            &on_machine("A"),
            Duration::from_millis(500),
        )
        .await;
        assert!(
            answered.is_empty(),
            "dropping the handle undeclares the queryable: {answered:?}"
        );
    }

    /// The probe that runs after an unanswered exchange returns as soon as the
    /// daemon does answer — the "linked late" case, which is what multicast
    /// scouting or a slow peer spawn looks like.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn the_link_probe_ends_when_the_daemon_answers_late() {
        let (a, b) = linked_sessions().await;
        let dataflow = uuid();
        let probe = tokio::spawn(probe_link(
            b.clone(),
            dataflow,
            on_machine("A"),
            Reporter(None),
        ));
        tokio::time::sleep(Duration::from_millis(300)).await;
        let _queryable = declare(
            &a,
            dataflow,
            &DaemonId::new(Some("A".into())),
            BTreeMap::new(),
        )
        .await
        .expect("declare on the listening session");

        tokio::time::timeout(LINK_PROBE_INTERVAL * 3, probe)
            .await
            .expect("probe must return once the daemon answers")
            .expect("probe task must not panic");
    }
}
