use crate::Event;
use dora_core::topics::ZENOH_CONTROL_QUERYABLE;
use eyre::eyre;
use futures::{Stream, StreamExt};
use futures_concurrency::stream::IntoStream;
use zenoh::{prelude::EntityFactory, sync::ZFuture};

pub(crate) async fn control_events() -> eyre::Result<impl Stream<Item = Event>> {
    let zenoh = zenoh::open(zenoh::config::Config::default())
        .wait()
        .map_err(|err| eyre!(err))?
        .into_arc();

    let queryable = zenoh
        .queryable(ZENOH_CONTROL_QUERYABLE)
        .wait()
        .map_err(|err| eyre!(err))?;

    Ok(queryable.into_stream().map(Event::Control))
}
