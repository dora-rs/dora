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

use std::{
    collections::{BTreeMap, BTreeSet},
    time::Duration,
};

use dora_message::{common::DaemonId, id::NodeId};
use uuid::Uuid;
use zenoh::Wait;

/// How long to keep asking peers for endpoints before spawning anyway.
///
/// Paid only by dataflows that actually span machines, and only once per spawn.
/// Short on purpose: this runs on the daemon's event loop, and the fallback —
/// the daemon-forwarded path — is correct, just slower.
const DEFAULT_TIMEOUT: Duration = Duration::from_millis(1500);

/// Env var overriding [`DEFAULT_TIMEOUT`], in milliseconds.
///
/// A slow or congested link may need longer; `0` skips the exchange entirely
/// and keeps every cross-machine edge on the daemon path.
const TIMEOUT_ENV: &str = "DORA_ZENOH_ENDPOINT_EXCHANGE_TIMEOUT_MS";

/// How long to wait for replies to a single query before asking again.
const QUERY_ROUND: Duration = Duration::from_millis(100);

/// Zenoh key a daemon answers its local nodes' endpoints on.
///
/// The daemon id is sanitized into one key chunk: a machine id is operator
/// input and may contain characters (`/`, `*`) that would silently reshape the
/// key expression. The id's uuid suffix survives sanitization untouched, so
/// distinct daemons keep distinct keys even if their machine ids collapse to
/// the same sanitized form.
fn endpoints_key(dataflow_id: Uuid, daemon_id: &DaemonId) -> String {
    let daemon: String = daemon_id
        .to_string()
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' || c == '.' || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect();
    format!("dora/default/{dataflow_id}/node-endpoints/{daemon}")
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

/// Keeps this daemon answering endpoint queries for one dataflow.
///
/// Held for the dataflow's lifetime rather than just the exchange: a daemon
/// that joins late (a restarted node, a second `dora start` against the same
/// graph) asks the same question, and an undeclared queryable answers nothing.
/// Dropped in `finish_dataflow`; an abandoned handle would keep the queryable,
/// its session clone and its payload alive for the daemon's whole lifetime.
pub struct EndpointQueryable {
    _queryable: zenoh::query::Queryable<()>,
}

/// Publish this daemon's node endpoints and collect the peers' — see the module
/// docs.
///
/// `local` maps each local node to the endpoint remote consumers should dial;
/// `wanted` names the remote nodes this daemon's own consumers need. Returns
/// the endpoints found (a subset of `wanted`) plus the queryable to keep alive.
///
/// Never fails the spawn: a zenoh error, an unanswered query or an expired
/// deadline all resolve to "fewer endpoints than asked for", which leaves those
/// edges on the daemon-forwarded path.
pub async fn exchange(
    session: &zenoh::Session,
    dataflow_id: Uuid,
    daemon_id: &DaemonId,
    local: Endpoints,
    wanted: BTreeSet<NodeId>,
) -> (Endpoints, Option<EndpointQueryable>) {
    let queryable = declare(session, dataflow_id, daemon_id, local).await;
    if wanted.is_empty() {
        // Nothing to collect — but stay answerable, since peers that consume
        // from this daemon's nodes still need what we just declared.
        return (BTreeMap::new(), queryable);
    }
    let deadline = timeout();
    if deadline.is_zero() {
        return (BTreeMap::new(), queryable);
    }
    let found = collect(session, dataflow_id, &wanted, deadline).await;
    if found.len() < wanted.len() {
        let missing: Vec<&NodeId> = wanted
            .iter()
            .filter(|id| !found.contains_key(*id))
            .collect();
        tracing::warn!(
            "no zenoh endpoint for remote node(s) {missing:?} after {deadline:?}; \
             their outputs will reach this daemon's nodes over the daemon path \
             instead of directly (set {TIMEOUT_ENV} to allow longer)"
        );
    }
    (found, queryable)
}

/// Declare the queryable that answers this daemon's endpoints.
async fn declare(
    session: &zenoh::Session,
    dataflow_id: Uuid,
    daemon_id: &DaemonId,
    local: Endpoints,
) -> Option<EndpointQueryable> {
    if local.is_empty() {
        // No node here is consumed from another machine, so there is nothing to
        // answer. Peers still query, and get no reply from us — which is the
        // same answer an empty map would give them.
        return None;
    }
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
        Ok(queryable) => Some(EndpointQueryable {
            _queryable: queryable,
        }),
        Err(err) => {
            // Peers keep their cross-machine edges on the daemon path, which is
            // where they are today.
            tracing::warn!("failed to declare the zenoh node-endpoint queryable on {key}: {err}");
            None
        }
    }
}

/// Query peers until every wanted endpoint is known or the deadline expires.
async fn collect(
    session: &zenoh::Session,
    dataflow_id: Uuid,
    wanted: &BTreeSet<NodeId>,
    deadline: Duration,
) -> Endpoints {
    let selector = endpoints_selector(dataflow_id);
    let started = tokio::time::Instant::now();
    let mut found: Endpoints = BTreeMap::new();
    while started.elapsed() < deadline {
        // A peer that has not processed its own spawn yet has no queryable to
        // answer with, so an empty round means "ask again", not "there is
        // nobody". `ConsolidationMode::None` because every daemon answers on
        // its own key and consolidation would be free to keep just one reply
        // per key — a silent way to lose a whole daemon's endpoints.
        let replies = session
            .get(&selector)
            .consolidation(zenoh::query::ConsolidationMode::None)
            .timeout(QUERY_ROUND)
            .await;
        match replies {
            Ok(replies) => {
                while let Ok(reply) = replies.recv_async().await {
                    let Ok(sample) = reply.result() else { continue };
                    let payload = sample.payload().to_bytes();
                    match serde_json::from_slice::<Endpoints>(&payload) {
                        Ok(endpoints) => found.extend(
                            endpoints
                                .into_iter()
                                .filter(|(node_id, _)| wanted.contains(node_id)),
                        ),
                        Err(err) => {
                            tracing::warn!("ignoring malformed zenoh node-endpoint reply: {err}")
                        }
                    }
                }
            }
            Err(err) => {
                tracing::warn!("zenoh node-endpoint query failed: {err}");
                return found;
            }
        }
        if wanted.iter().all(|id| found.contains_key(id)) {
            break;
        }
    }
    found
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
}
