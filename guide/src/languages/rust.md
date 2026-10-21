# Rust API Reference

Dora provides two main Rust crates for building dataflow components:

- **`dora-node-api`** — for standalone node executables
- **`dora-operator-api`** — for in-process operators managed by the Dora runtime

## Full reference

The complete Rust API reference — `DoraNode`, `send_output` and friends, the
`Event` stream, the `DoraArray` / `IntoArrow` Arrow interface, and the
[Arrow version policy][arrow-policy] — is maintained in a single canonical
document:

> **[docs/api-rust.md — Rust API Reference][canonical]**

This page intentionally links out instead of restating the reference. The two
used to be hand-maintained copies of the same text (see
[#3252][issue]); a correction then had to be made in both places and they
drifted apart anyway. Keeping one source of truth avoids that.

[canonical]: https://github.com/dora-rs/dora/blob/main/docs/api-rust.md
[arrow-policy]: https://github.com/dora-rs/dora/blob/main/docs/api-rust.md#arrow-version-policy
[issue]: https://github.com/dora-rs/dora/issues/3252
