use std::{
    collections::BTreeMap,
    sync::{Arc, Mutex},
};

use futures::channel::oneshot;
use thiserror::Error;

use super::keyexpr::{EntityKind, KeyError, LivelinessKey};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GraphEntity {
    pub key: String,
    pub token: LivelinessKey,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GraphSnapshot {
    pub generation: u64,
    pub entities: Vec<GraphEntity>,
    pub initialized: bool,
    pub closed: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GraphUpdate(bool);
impl GraphUpdate {
    pub fn changed(self) -> bool {
        self.0
    }
}

#[derive(Debug, Error)]
pub enum GraphError {
    #[error("malformed liveliness token: {0}")]
    MalformedToken(#[source] KeyError),
    #[error("ROS graph is closed")]
    Closed,
}

struct Entry {
    token: LivelinessKey,
    /// Monotonic insertion order — used to evict the oldest remote entity.
    seq: u64,
    /// Locally-declared (own) entity; never evicted under capacity pressure.
    local: bool,
}

#[derive(Default)]
struct State {
    generation: u64,
    entities: BTreeMap<String, Entry>,
    next_seq: u64,
    waiters: Vec<oneshot::Sender<Result<u64, GraphError>>>,
    closed: bool,
    initialized: bool,
}

/// Upper bound on liveliness entities retained in the graph cache. The tokens
/// are announced by remote, unauthenticated peers, so the map must be capped to
/// prevent a memory-exhaustion DoS (a peer streaming distinct tokens on
/// `@ros2_lv/{domain}/**`).
const MAX_GRAPH_ENTITIES: usize = 16 * 1024;

#[derive(Clone)]
pub struct GraphCache {
    domain: usize,
    limit: usize,
    state: Arc<Mutex<State>>,
}

impl GraphCache {
    pub fn new(domain: usize) -> Self {
        Self::with_limit(domain, MAX_GRAPH_ENTITIES)
    }

    fn with_limit(domain: usize, limit: usize) -> Self {
        Self {
            domain,
            limit,
            state: Arc::new(Mutex::new(State::default())),
        }
    }

    /// Record a liveliness token observed from the (untrusted) ROS graph.
    pub fn apply_put(&self, key: &str) -> Result<GraphUpdate, GraphError> {
        self.insert(key, false)
    }

    /// Record a liveliness token the local process declared itself. Local
    /// entities are protected from capacity eviction so a remote token flood
    /// cannot evict the node's own discovery state.
    pub fn apply_put_local(&self, key: &str) -> Result<GraphUpdate, GraphError> {
        self.insert(key, true)
    }

    fn insert(&self, key: &str, local: bool) -> Result<GraphUpdate, GraphError> {
        let token = LivelinessKey::parse(key).map_err(GraphError::MalformedToken)?;
        if token.domain != self.domain {
            return Ok(GraphUpdate(false));
        }
        let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
        if state.closed {
            return Err(GraphError::Closed);
        }
        if let Some(entry) = state.entities.get_mut(key) {
            // Already known. Upgrade to local if this is our own declaration —
            // the subscriber may have echoed the token back before we ran.
            if local && !entry.local {
                entry.local = true;
            }
            return Ok(GraphUpdate(false));
        }
        if state.entities.len() >= self.limit {
            // Admit the new token by evicting the oldest *remote* entity; never
            // evict a local (own) entity. Refuse only if everything retained is
            // local (which cannot happen from a remote flood alone).
            let victim = state
                .entities
                .iter()
                .filter(|(_, entry)| !entry.local)
                .min_by_key(|(_, entry)| entry.seq)
                .map(|(evict_key, _)| evict_key.clone());
            match victim {
                Some(victim) => {
                    state.entities.remove(&victim);
                }
                None => {
                    tracing::warn!(
                        "ROS graph cache at capacity ({}) with only local entities; \
                         dropping liveliness token",
                        self.limit
                    );
                    return Ok(GraphUpdate(false));
                }
            }
        }
        let seq = state.next_seq;
        state.next_seq = state.next_seq.wrapping_add(1);
        state
            .entities
            .insert(key.to_owned(), Entry { token, seq, local });
        Self::changed(&mut state);
        Ok(GraphUpdate(true))
    }

    pub fn apply_delete(&self, key: &str) -> Result<GraphUpdate, GraphError> {
        let token = LivelinessKey::parse(key).map_err(GraphError::MalformedToken)?;
        if token.domain != self.domain {
            return Ok(GraphUpdate(false));
        }
        let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
        if state.closed {
            return Err(GraphError::Closed);
        }
        if state.entities.remove(key).is_none() {
            return Ok(GraphUpdate(false));
        }
        Self::changed(&mut state);
        Ok(GraphUpdate(true))
    }

    pub fn snapshot(&self) -> GraphSnapshot {
        let state = self.state.lock().unwrap_or_else(|error| error.into_inner());
        GraphSnapshot {
            generation: state.generation,
            entities: state
                .entities
                .iter()
                .map(|(key, entry)| GraphEntity {
                    key: key.clone(),
                    token: entry.token.clone(),
                })
                .collect(),
            initialized: state.initialized,
            closed: state.closed,
        }
    }

    pub fn matching_services(
        &self,
        name: &str,
        type_name: &str,
        type_hash: &str,
        qos: &str,
    ) -> Vec<GraphEntity> {
        self.snapshot()
            .entities
            .into_iter()
            .filter(|entity| {
                entity.token.kind == EntityKind::Service
                    && entity.token.topic.as_ref().is_some_and(|topic| {
                        topic.name == name
                            && topic.type_name == type_name
                            && topic.type_hash == type_hash
                            && topic.qos == qos
                    })
            })
            .collect()
    }

    pub async fn wait_for_change(&self, generation: u64) -> Result<u64, GraphError> {
        let receiver = {
            let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
            if state.closed {
                return Err(GraphError::Closed);
            }
            if state.generation != generation {
                return Ok(state.generation);
            }
            let (sender, receiver) = oneshot::channel();
            state.waiters.push(sender);
            receiver
        };
        receiver.await.unwrap_or(Err(GraphError::Closed))
    }

    pub fn shutdown(&self) {
        let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
        if state.closed {
            return;
        }
        state.closed = true;
        for waiter in state.waiters.drain(..) {
            let _ = waiter.send(Err(GraphError::Closed));
        }
    }

    pub fn finish_initialization(&self) {
        let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
        if !state.initialized && !state.closed {
            state.initialized = true;
            Self::changed(&mut state);
        }
    }

    fn changed(state: &mut State) {
        state.generation = state.generation.wrapping_add(1);
        let generation = state.generation;
        for waiter in state.waiters.drain(..) {
            let _ = waiter.send(Ok(generation));
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{GraphCache, GraphSnapshot};
    use crate::transport::zenoh::{Context, ContextOptions, keyexpr::LivelinessKey};

    fn node_key(nid: &str) -> String {
        LivelinessKey::node(0, "zid", nid, "eid", "/", "/ns", "node")
            .unwrap()
            .as_str()
            .to_owned()
    }

    #[test]
    fn apply_put_caps_entities_and_evicts_oldest_remote() {
        // A remote peer controls how many liveliness tokens it announces; the
        // cache stays bounded, admitting the newest by evicting the oldest
        // remote entry (so discovery of new entities is never stalled).
        let cache = GraphCache::with_limit(0, 2);
        for i in 0..8 {
            let _ = cache.apply_put(&node_key(&format!("n{i}")));
        }
        let keys: Vec<_> = cache
            .snapshot()
            .entities
            .into_iter()
            .map(|entity| entity.key)
            .collect();
        assert_eq!(keys.len(), 2);
        // The two most-recent tokens are retained; the oldest were evicted.
        assert!(keys.contains(&node_key("n6")) && keys.contains(&node_key("n7")));
    }

    #[test]
    fn apply_put_never_evicts_local_entities() {
        // Local (own) entities must survive a remote token flood.
        let cache = GraphCache::with_limit(0, 2);
        let local = node_key("local");
        cache.apply_put_local(&local).unwrap();
        for i in 0..16 {
            let _ = cache.apply_put(&node_key(&format!("flood{i}")));
        }
        let keys: Vec<_> = cache
            .snapshot()
            .entities
            .into_iter()
            .map(|entity| entity.key)
            .collect();
        assert_eq!(keys.len(), 2);
        assert!(keys.contains(&local), "local entity must not be evicted");
    }

    /// Liveliness-token declaration/undeclaration propagates through the Zenoh
    /// session asynchronously, so the graph cache is updated a beat after
    /// `create_node` / `drop` return. Await the graph reaching the expected
    /// state (event-driven via `wait_for_change`) instead of asserting the
    /// snapshot synchronously, which raced and flaked (~1-in-4).
    async fn wait_until(graph: &GraphCache, predicate: impl Fn(&GraphSnapshot) -> bool) {
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                let snapshot = graph.snapshot();
                if predicate(&snapshot) {
                    break;
                }
                graph
                    .wait_for_change(snapshot.generation)
                    .await
                    .expect("graph closed while waiting for a change");
            }
        })
        .await
        .expect("timed out waiting for the graph to reach the expected state");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn drop_removes_all_tokens() {
        let context = Context::open(ContextOptions {
            domain_id: 7,
            config_uri: None,
        })
        .await
        .unwrap();
        let node = context
            .create_node("zid", "nid", "/", "/", "node")
            .await
            .unwrap();
        wait_until(&context.graph, |snapshot| snapshot.entities.len() == 1).await;
        drop(node);
        wait_until(&context.graph, |snapshot| snapshot.entities.is_empty()).await;
    }
}
