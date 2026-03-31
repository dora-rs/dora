//! Backward-compatible shim crate that re-exports [`adora_operator_api`] under
//! the `dora-operator-api` name so that existing dora-hub operators compile
//! without changes.
//!
//! Usage in dora-hub operators:
//! ```rust,ignore
//! use dora_operator_api::{DoraOperator, DoraOutputSender, DoraStatus, Event, register_operator};
//! ```

pub use adora_operator_api::*;

// Trait alias: dora-hub operators `impl DoraOperator for T`.
pub use adora_operator_api::AdoraOperator as DoraOperator;

#[cfg(test)]
mod tests {
    //! Verify that every import pattern used by dora-hub operators compiles.

    // Pattern: `use dora_operator_api::{DoraOperator, DoraOutputSender, DoraStatus, Event, register_operator};`
    #[allow(unused_imports)]
    use crate::{DoraOperator, DoraOutputSender, DoraStatus, Event, register_operator};

    // DoraStatus enum variants used by dora-hub
    #[test]
    fn dora_status_variants() {
        let _continue = DoraStatus::Continue;
        let _stop = DoraStatus::Stop;
        let _stop_all = DoraStatus::StopAll;
    }

    // DoraOperator trait is implementable (the core dora-hub pattern).
    #[derive(Default)]
    struct TestOp;

    impl DoraOperator for TestOp {
        fn on_event(
            &mut self,
            event: &Event,
            _output_sender: &mut DoraOutputSender,
        ) -> Result<DoraStatus, String> {
            match event {
                Event::Input {
                    id,
                    data: _,
                    metadata: _,
                } => {
                    let _ = id;
                    Ok(DoraStatus::Continue)
                }
                Event::InputClosed { id: _ } => Ok(DoraStatus::Continue),
                Event::Stop => Ok(DoraStatus::Stop),
                _ => Ok(DoraStatus::Continue),
            }
        }
    }

    #[test]
    fn impl_dora_operator_compiles() {
        // The struct implements DoraOperator which is AdoraOperator.
        fn _assert_adora_operator<T: crate::AdoraOperator>() {}
        _assert_adora_operator::<TestOp>();
    }

    #[test]
    fn dora_output_sender_is_adora_output_sender() {
        // DoraOutputSender and AdoraOutputSender must be the same type.
        fn _takes_dora(_: &mut DoraOutputSender) {}
        fn _takes_adora(sender: &mut crate::AdoraOutputSender) {
            _takes_dora(sender);
        }
    }
}
