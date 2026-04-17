use dashmap::DashMap;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct StateKey {
    namespace: String,
    key: String,
}

impl StateKey {
    fn new(namespace: impl Into<String>, key: impl Into<String>) -> Self {
        Self {
            namespace: namespace.into(),
            key: key.into(),
        }
    }
}

#[derive(Debug, Clone)]
struct StateEntry {
    value: Vec<u8>,
    expires_at: Option<Instant>,
}

#[derive(Debug, Default)]
pub struct SharedStateStore {
    entries: DashMap<StateKey, StateEntry>,
}

impl SharedStateStore {
    pub fn get(&self, namespace: &str, key: &str) -> Option<Vec<u8>> {
        let state_key = StateKey::new(namespace, key);
        let entry = self.entries.get(&state_key)?;
        if is_expired(entry.expires_at) {
            drop(entry);
            self.entries.remove(&state_key);
            return None;
        }
        Some(entry.value.clone())
    }

    pub fn set(
        &self,
        namespace: String,
        key: String,
        value: Vec<u8>,
        ttl_ms: Option<u64>,
    ) -> Result<(), String> {
        validate_part("namespace", &namespace)?;
        validate_part("key", &key)?;
        let expires_at = expiration_from_ttl_ms(ttl_ms)?;

        self.entries.insert(
            StateKey::new(namespace, key),
            StateEntry { value, expires_at },
        );
        Ok(())
    }

    pub fn delete(&self, namespace: &str, key: &str) -> bool {
        let state_key = StateKey::new(namespace, key);
        self.entries.remove(&state_key).is_some()
    }
}

fn validate_part(name: &str, value: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{name} must not be empty"));
    }
    Ok(())
}

fn expiration_from_ttl_ms(ttl_ms: Option<u64>) -> Result<Option<Instant>, String> {
    let Some(ttl_ms) = ttl_ms else {
        return Ok(None);
    };
    if ttl_ms == 0 {
        return Err("ttl_ms must be greater than 0".to_owned());
    }
    let duration = Duration::from_millis(ttl_ms);
    let expires_at = Instant::now()
        .checked_add(duration)
        .ok_or_else(|| format!("ttl_ms is too large: {ttl_ms}"))?;
    Ok(Some(expires_at))
}

fn is_expired(expires_at: Option<Instant>) -> bool {
    expires_at.is_some_and(|deadline| Instant::now() >= deadline)
}
