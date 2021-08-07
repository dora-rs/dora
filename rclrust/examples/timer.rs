use std::cell::Cell;
use std::time::Duration;

use anyhow::Result;
use rclrust::prelude::*;

fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let node = ctx.create_node("examples_timer")?;
    let logger = node.logger();
    let count = Cell::new(0);
    let _timer = node.create_wall_timer(Duration::from_millis(100), move || {
        rclrust_info!(logger, "count: {}", count.get());
        count.set(count.get() + 1);
    })?;

    rclrust::spin(&node)?;

    Ok(())
}
