//! Regression fixture for dora-rs/dora: a shared-library operator that emits
//! many large Arrow outputs on every tick, so a burst is in flight at
//! `--stop-after`. The runtime used to unload the operator's `.so` as soon as
//! its `on_event` loop returned, while the main loop (another thread) was still
//! sending those in-flight arrays — whose Arrow FFI `release` callbacks live in
//! the `.so`. Freeing them after `dlclose` jumped into unmapped code (SIGSEGV).
//! Driven by `tests/example-smoke.rs::local_rust_operator_teardown_no_segfault`.
#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{
    DoraOperator, DoraOutputSender, DoraStatus, Event, IntoArrow, register_operator,
};

register_operator!(TeardownOperator);

#[derive(Default)]
struct TeardownOperator;

impl DoraOperator for TeardownOperator {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, String> {
        match event {
            Event::Input { id, .. } => match *id {
                // Source: flood large outputs so many are in flight at shutdown.
                "tick" => {
                    for _ in 0..20 {
                        output_sender.send("data", vec![0u8; 512 * 1024].into_arrow())?;
                    }
                }
                // Sink: just consume.
                "data" => {}
                _ => {}
            },
            Event::Stop => return Ok(DoraStatus::Stop),
            _ => {}
        }
        Ok(DoraStatus::Continue)
    }
}
