//! Merge external stream into an [`EventStream`][super::EventStream].
//!
//! Sometimes nodes need to listen to external events, in addition to Dora events.
//! This module provides support for that by providing the [`MergeExternal`] trait.

use futures::{Stream, StreamExt};
use futures_concurrency::stream::Merge;

/// A Dora event or an event from an external source.
#[derive(Debug)]
pub enum MergedEvent<E> {
    /// A Dora event
    Dora(super::Event),
    /// An external event
    ///
    /// Yielded by the stream that was merged into the Dora [`EventStream`][super::EventStream].
    External(E),
}

/// A general enum to represent a value of two possible types.
pub enum Either<A, B> {
    /// Value is of the first type, type `A`.
    First(A),
    /// Value is of the second type, type `B`.
    Second(B),
}

impl<A> Either<A, A> {
    /// Unwraps an `Either` instance where both types are identical.
    pub fn flatten(self) -> A {
        match self {
            Either::First(a) => a,
            Either::Second(a) => a,
        }
    }
}

/// Allows merging an external event stream into an existing event stream.
// TODO: use impl trait return type once stable
pub trait MergeExternal<'a, E> {
    /// The item type yielded from the merged stream.
    type Item;

    /// Merge the given stream into an existing event stream.
    ///
    /// Returns a new event stream that yields items from both streams.
    /// The ordering between the two streams is not guaranteed.
    fn merge_external(
        self,
        external_events: impl Stream<Item = E> + Unpin + 'a,
    ) -> Box<dyn Stream<Item = Self::Item> + Unpin + 'a>;
}

/// Allows merging a sendable external event stream into an existing (sendable) event stream.
///
/// By implementing [`Send`], the streams can be sent to different threads.
pub trait MergeExternalSend<'a, E> {
    /// The item type yielded from the merged stream.
    type Item;

    /// Merge the given stream into an existing event stream.
    ///
    /// Returns a new event stream that yields items from both streams.
    /// The ordering between the two streams is not guaranteed.
    fn merge_external_send(
        self,
        external_events: impl Stream<Item = E> + Unpin + Send + Sync + 'a,
    ) -> Box<dyn Stream<Item = Self::Item> + Unpin + Send + Sync + 'a>;
}

impl<'a, E> MergeExternal<'a, E> for super::EventStream
where
    E: 'static,
{
    type Item = MergedEvent<E>;

    fn merge_external(
        self,
        external_events: impl Stream<Item = E> + Unpin + 'a,
    ) -> Box<dyn Stream<Item = Self::Item> + Unpin + 'a> {
        let dora = self.map(MergedEvent::Dora);
        let external = external_events.map(MergedEvent::External);
        Box::new((dora, external).merge())
    }
}

impl<'a, E> MergeExternalSend<'a, E> for super::EventStream
where
    E: 'static,
{
    type Item = MergedEvent<E>;

    fn merge_external_send(
        self,
        external_events: impl Stream<Item = E> + Unpin + Send + Sync + 'a,
    ) -> Box<dyn Stream<Item = Self::Item> + Unpin + Send + Sync + 'a> {
        let dora = self.map(MergedEvent::Dora);
        let external = external_events.map(MergedEvent::External);
        Box::new((dora, external).merge())
    }
}

impl<'a, E, F, S> MergeExternal<'a, F> for S
where
    S: Stream<Item = MergedEvent<E>> + Unpin + 'a,
    E: 'a,
    F: 'a,
{
    type Item = MergedEvent<Either<E, F>>;

    fn merge_external(
        self,
        external_events: impl Stream<Item = F> + Unpin + 'a,
    ) -> Box<dyn Stream<Item = Self::Item> + Unpin + 'a> {
        let first = self.map(|e| match e {
            MergedEvent::Dora(d) => MergedEvent::Dora(d),
            MergedEvent::External(e) => MergedEvent::External(Either::First(e)),
        });
        let second = external_events.map(|e| MergedEvent::External(Either::Second(e)));
        Box::new((first, second).merge())
    }
}

impl<'a, E, F, S> MergeExternalSend<'a, F> for S
where
    S: Stream<Item = MergedEvent<E>> + Unpin + Send + Sync + 'a,
    E: 'a,
    F: 'a,
{
    type Item = MergedEvent<Either<E, F>>;

    fn merge_external_send(
        self,
        external_events: impl Stream<Item = F> + Unpin + Send + Sync + 'a,
    ) -> Box<dyn Stream<Item = Self::Item> + Unpin + Send + Sync + 'a> {
        let first = self.map(|e| match e {
            MergedEvent::Dora(d) => MergedEvent::Dora(d),
            MergedEvent::External(e) => MergedEvent::External(Either::First(e)),
        });
        let second = external_events.map(|e| MergedEvent::External(Either::Second(e)));
        Box::new((first, second).merge())
    }
}
