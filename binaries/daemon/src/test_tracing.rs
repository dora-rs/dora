//! Tracing helpers for tests that assert on the events the daemon logs.
//!
//! A test captures events by running its code under a scoped subscriber
//! (`tracing::subscriber::with_default`). On its own that races with every
//! other test in the binary: while no global subscriber is set and only one
//! dispatcher is alive, `tracing-core` registers a callsite by asking just the
//! registering thread's default dispatcher. A test thread with no scoped
//! subscriber answers "never", and that interest is cached for the whole
//! process, so a capture running concurrently on another thread then sees
//! none of that callsite's events (dora-rs/dora#3610, dora-rs/dora#3711).
//!
//! [`install_global_subscriber`] sets a global subscriber that is interested
//! in everything and does nothing. With it in place a callsite registered on a
//! thread without a scoped subscriber resolves to "always", and while a
//! capture is alive every live dispatcher is consulted. The capture
//! subscribers below call it from their constructors, so it is in place before
//! any capture starts; a new capture belongs here and should do the same.

use std::sync::{Arc, Mutex, Once, PoisonError};

/// Installs the do-nothing global subscriber, once per test process.
fn install_global_subscriber() {
    static INSTALL: Once = Once::new();
    INSTALL.call_once(|| {
        // Something else may have set a global already; that one serves too.
        let _ = tracing::subscriber::set_global_default(Discard);
    });
}

/// Accepts every event and drops it.
struct Discard;

impl tracing::Subscriber for Discard {
    fn enabled(&self, _metadata: &tracing::Metadata<'_>) -> bool {
        true
    }
    fn new_span(&self, _span: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _span: &tracing::span::Id, _values: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _span: &tracing::span::Id, _follows: &tracing::span::Id) {}
    fn event(&self, _event: &tracing::Event<'_>) {}
    fn enter(&self, _span: &tracing::span::Id) {}
    fn exit(&self, _span: &tracing::span::Id) {}
}

/// Minimal `tracing::Subscriber` that records the level of every event it
/// receives, so a test can assert whether (and how often) a level fires.
#[derive(Clone)]
pub(crate) struct LevelCapture {
    pub(crate) levels: Arc<Mutex<Vec<tracing::Level>>>,
}

impl LevelCapture {
    pub(crate) fn new() -> Self {
        install_global_subscriber();
        Self {
            levels: Default::default(),
        }
    }
}

impl tracing::Subscriber for LevelCapture {
    fn enabled(&self, _metadata: &tracing::Metadata<'_>) -> bool {
        true
    }
    fn new_span(&self, _span: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _span: &tracing::span::Id, _values: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _span: &tracing::span::Id, _follows: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        self.levels
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(*event.metadata().level());
    }
    fn enter(&self, _span: &tracing::span::Id) {}
    fn exit(&self, _span: &tracing::span::Id) {}
}

/// Records the text of every tracing event's fields.
#[derive(Clone)]
pub(crate) struct TextCapture(pub(crate) Arc<Mutex<Vec<String>>>);

impl TextCapture {
    pub(crate) fn new() -> Self {
        install_global_subscriber();
        Self(Default::default())
    }
}

impl tracing::Subscriber for TextCapture {
    fn enabled(&self, _metadata: &tracing::Metadata<'_>) -> bool {
        true
    }
    fn new_span(&self, _span: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _span: &tracing::span::Id, _values: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _span: &tracing::span::Id, _follows: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        struct Text<'a>(&'a mut String);
        impl tracing::field::Visit for Text<'_> {
            fn record_debug(
                &mut self,
                _field: &tracing::field::Field,
                value: &dyn std::fmt::Debug,
            ) {
                use std::fmt::Write;
                let _ = write!(self.0, "{value:?}");
            }
        }
        let mut text = String::new();
        event.record(&mut Text(&mut text));
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(text);
    }
    fn enter(&self, _span: &tracing::span::Id) {}
    fn exit(&self, _span: &tracing::span::Id) {}
}
