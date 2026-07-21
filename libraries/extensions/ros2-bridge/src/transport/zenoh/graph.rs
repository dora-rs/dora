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

#[derive(Default)]
struct State {
    generation: u64,
    entities: BTreeMap<String, LivelinessKey>,
    waiters: Vec<oneshot::Sender<Result<u64, GraphError>>>,
    closed: bool,
    initialized: bool,
}

#[derive(Clone)]
pub struct GraphCache {
    domain: usize,
    state: Arc<Mutex<State>>,
}

impl GraphCache {
    pub fn new(domain: usize) -> Self {
        Self {
            domain,
            state: Arc::new(Mutex::new(State::default())),
        }
    }

    pub fn apply_put(&self, key: &str) -> Result<GraphUpdate, GraphError> {
        let token = LivelinessKey::parse(key).map_err(GraphError::MalformedToken)?;
        if token.domain != self.domain {
            return Ok(GraphUpdate(false));
        }
        let mut state = self.state.lock().unwrap_or_else(|error| error.into_inner());
        if state.closed {
            return Err(GraphError::Closed);
        }
        if state.entities.contains_key(key) {
            return Ok(GraphUpdate(false));
        }
        state.entities.insert(key.to_owned(), token);
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
                .map(|(key, token)| GraphEntity {
                    key: key.clone(),
                    token: token.clone(),
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
    use crate::transport::zenoh::{Context, ContextOptions};

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
        assert_eq!(context.graph.snapshot().entities.len(), 1);
        drop(node);
        assert!(context.graph.snapshot().entities.is_empty());
    }
}
