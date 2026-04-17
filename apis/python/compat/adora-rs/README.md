# adora-rs (deprecated)

This package is a compatibility shim.

**`adora-rs` was renamed to `dora-rs` for the 1.0 release.** The `adora-rs` PyPI name was used by the fork (`dora-rs/adora`) before the consolidation into `dora-rs/dora` as dora 1.0. Installing this package pulls in `dora-rs>=1.0.0` and emits a `DeprecationWarning` on import; existing `from adora_rs import ...` code continues to work during migration.

**To stop seeing the warning**: `pip uninstall adora-rs && pip install dora-rs`.

Full migration guide: https://github.com/dora-rs/dora/blob/main/docs/migration-from-0.x.md

This shim is maintained for ~6 months after the 1.0 GA release, per the consolidation plan (see decision point D-4). No functional changes will ship here — all features land in `dora-rs`.
