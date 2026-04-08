use std::{
    collections::VecDeque,
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, Ordering},
    },
    time::{Instant, SystemTime, UNIX_EPOCH},
};

use tracing::{
    Id, Subscriber,
    field::{Field, Visit},
    span::Attributes,
};
use tracing_subscriber::{Layer, layer::Context, registry::LookupSpan};

/// A completed span captured for in-memory inspection.
#[derive(Debug, Clone)]
pub struct SpanRecord {
    pub trace_id: String,
    pub span_id: u64,
    pub parent_span_id: Option<u64>,
    pub name: String,
    pub target: String,
    pub level: String,
    pub start_time: u64,
    pub duration_us: u64,
    pub fields: Vec<(String, String)>,
}

const MAX_SPANS: usize = 4096;
const MAX_FIELDS_PER_SPAN: usize = 64;
const MAX_FIELD_VALUE_BYTES: usize = 4096;

/// Bounded ring-buffer of completed spans.
pub struct SpanStore {
    spans: VecDeque<SpanRecord>,
}

impl Default for SpanStore {
    fn default() -> Self {
        Self {
            spans: VecDeque::with_capacity(MAX_SPANS),
        }
    }
}

impl SpanStore {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, record: SpanRecord) {
        if self.spans.len() >= MAX_SPANS {
            self.spans.pop_front();
        }
        self.spans.push_back(record);
    }

    pub fn spans(&self) -> &VecDeque<SpanRecord> {
        &self.spans
    }
}

pub type SharedSpanStore = Arc<Mutex<SpanStore>>;

pub fn new_shared_store() -> SharedSpanStore {
    Arc::new(Mutex::new(SpanStore::new()))
}

/// Extension data stored on each span while it's open.
struct SpanData {
    trace_id: String,
    span_id: u64,
    parent_span_id: Option<u64>,
    start_instant: Instant,
    start_time: u64,
    fields: Vec<(String, String)>,
}

struct FieldVisitor(Vec<(String, String)>);

impl FieldVisitor {
    fn push_field(&mut self, name: &str, mut value: String) {
        if self.0.len() >= MAX_FIELDS_PER_SPAN {
            return;
        }
        if value.len() > MAX_FIELD_VALUE_BYTES {
            value.truncate(MAX_FIELD_VALUE_BYTES);
            value.push_str("...");
        }
        self.0.push((name.to_string(), value));
    }
}

impl Visit for FieldVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        self.push_field(field.name(), format!("{value:?}"));
    }

    fn record_str(&mut self, field: &Field, value: &str) {
        self.push_field(field.name(), value.to_string());
    }

    fn record_i64(&mut self, field: &Field, value: i64) {
        self.push_field(field.name(), value.to_string());
    }

    fn record_u64(&mut self, field: &Field, value: u64) {
        self.push_field(field.name(), value.to_string());
    }

    fn record_bool(&mut self, field: &Field, value: bool) {
        self.push_field(field.name(), value.to_string());
    }
}

/// A `tracing` layer that captures completed spans into a [`SharedSpanStore`].
pub struct SpanCaptureLayer {
    store: SharedSpanStore,
    next_span_id: AtomicU64,
}

impl SpanCaptureLayer {
    pub fn new(store: SharedSpanStore) -> Self {
        Self {
            store,
            next_span_id: AtomicU64::new(1),
        }
    }
}

fn unix_millis_now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

impl<S> Layer<S> for SpanCaptureLayer
where
    S: Subscriber + for<'a> LookupSpan<'a>,
{
    fn on_new_span(&self, attrs: &Attributes<'_>, id: &Id, ctx: Context<'_, S>) {
        let Some(span) = ctx.span(id) else { return };
        let span_id = self.next_span_id.fetch_add(1, Ordering::Relaxed);

        // Inherit trace_id from parent or generate a new one.
        let (trace_id, parent_span_id) = if let Some(parent) =
            span.parent().and_then(|parent_ref| {
                let ext = parent_ref.extensions();
                ext.get::<SpanData>()
                    .map(|d| (d.trace_id.clone(), d.span_id))
            }) {
            (parent.0, Some(parent.1))
        } else {
            (uuid::Uuid::new_v4().to_string(), None)
        };

        let mut visitor = FieldVisitor(Vec::new());
        attrs.record(&mut visitor);

        let data = SpanData {
            trace_id,
            span_id,
            parent_span_id,
            start_instant: Instant::now(),
            start_time: unix_millis_now(),
            fields: visitor.0,
        };

        span.extensions_mut().insert(data);
    }

    fn on_record(&self, id: &Id, values: &tracing::span::Record<'_>, ctx: Context<'_, S>) {
        if let Some(span) = ctx.span(id) {
            let mut ext = span.extensions_mut();
            if let Some(data) = ext.get_mut::<SpanData>()
                && data.fields.len() < MAX_FIELDS_PER_SPAN
            {
                let mut visitor = FieldVisitor(Vec::new());
                values.record(&mut visitor);
                let remaining = MAX_FIELDS_PER_SPAN.saturating_sub(data.fields.len());
                data.fields.extend(visitor.0.into_iter().take(remaining));
            }
        }
    }

    fn on_close(&self, id: Id, ctx: Context<'_, S>) {
        let Some(span) = ctx.span(&id) else { return };
        let ext = span.extensions();
        let Some(data) = ext.get::<SpanData>() else {
            return;
        };

        let record = SpanRecord {
            trace_id: data.trace_id.clone(),
            span_id: data.span_id,
            parent_span_id: data.parent_span_id,
            name: span.name().to_string(),
            target: span.metadata().target().to_string(),
            level: span.metadata().level().to_string(),
            start_time: data.start_time,
            duration_us: data.start_instant.elapsed().as_micros() as u64,
            fields: data.fields.clone(),
        };

        drop(ext);

        if let Ok(mut store) = self.store.lock() {
            store.push(record);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tracing_subscriber::{Registry, layer::SubscriberExt};

    #[test]
    fn span_store_evicts_oldest() {
        let mut store = SpanStore::new();
        for i in 0..MAX_SPANS + 10 {
            store.push(SpanRecord {
                trace_id: "t".into(),
                span_id: i as u64,
                parent_span_id: None,
                name: "test".into(),
                target: "test".into(),
                level: "INFO".into(),
                start_time: 0,
                duration_us: 0,
                fields: vec![],
            });
        }
        assert_eq!(store.spans().len(), MAX_SPANS);
        assert_eq!(store.spans().front().unwrap().span_id, 10);
    }

    #[test]
    fn capture_layer_records_spans() {
        let store = new_shared_store();
        let layer = SpanCaptureLayer::new(store.clone());
        let subscriber = Registry::default().with(layer);

        tracing::subscriber::with_default(subscriber, || {
            let span = tracing::info_span!("outer", key = "val");
            let _guard = span.enter();
            {
                let inner = tracing::debug_span!("inner");
                let _g2 = inner.enter();
            }
        });

        let store = store.lock().unwrap();
        assert_eq!(store.spans().len(), 2);

        // inner closes first
        let inner = &store.spans()[0];
        assert_eq!(inner.name, "inner");
        assert!(inner.parent_span_id.is_some());

        let outer = &store.spans()[1];
        assert_eq!(outer.name, "outer");
        assert!(outer.parent_span_id.is_none());

        // Same trace
        assert_eq!(inner.trace_id, outer.trace_id);

        // Fields
        assert!(outer.fields.iter().any(|(k, _)| k == "key"));
    }
}
