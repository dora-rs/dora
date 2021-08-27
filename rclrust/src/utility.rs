use anyhow::Result;

use crate::{context::Context, init_options::InitOptions};

/// Initialize rclrust context
///
/// # Examples
///
/// ```
/// let ctx = rclrust::init().unwrap();
/// assert!(ctx.is_valid());
/// ```
pub fn init() -> Result<Context> {
    init_with_options(InitOptions::new()?)
}

/// Initialize rclrust context
///
/// # Examples
///
/// ```
/// use rclrust::InitOptions;
///
/// let init_options = InitOptions::new().unwrap();
/// let ctx = rclrust::init_with_options(init_options).unwrap();
/// assert!(ctx.is_valid());
/// ```
pub fn init_with_options(init_options: InitOptions) -> Result<Context> {
    Context::new(std::env::args().collect::<Vec<_>>(), init_options)
}

/// Check rclrust's status.
pub fn ok(ctx: &Context) -> bool {
    ctx.is_valid()
}

/// Shutdown rclrust context
pub fn shutdown(ctx: &Context, reason: &str) -> Result<()> {
    ctx.shutdown(reason)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn rclrust_init() -> Result<()> {
        let ctx = init()?;
        assert!(ok(&ctx));

        Ok(())
    }
}
