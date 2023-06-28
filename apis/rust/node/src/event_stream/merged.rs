use futures::{Stream, StreamExt};
use futures_concurrency::stream::Merge;

pub enum MergedEvent<E> {
    Dora(super::Event),
    External(E),
}

pub enum Either<A, B> {
    First(A),
    Second(B),
}

pub trait MergeExternal<'a, E> {
    type Item;

    fn merge_external(
        self,
        external_events: impl Stream<Item = E> + Unpin + 'a,
    ) -> Box<dyn Stream<Item = Self::Item> + Unpin + 'a>;
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
