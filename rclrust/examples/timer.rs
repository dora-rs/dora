use std::{
    sync::atomic::{AtomicU32, Ordering},
    time::Duration,
};

use anyhow::Result;
use rclrust::rclrust_info;

#[tokio::main]
async fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let mut node = ctx.create_node("examples_timer")?;
    let logger = node.logger();
    let count = AtomicU32::default();
    let _timer = node.create_wall_timer(Duration::from_millis(100), move || {
        rclrust_info!(logger, "count: {}", count.load(Ordering::Relaxed));
        count.fetch_add(1, Ordering::Relaxed);
    })?;

    node.wait();
    Ok(())
}
