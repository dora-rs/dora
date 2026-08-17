# Decoupling `dora-node-api` from the Arrow major version

**Status:** implemented. The user-facing contract now lives in
[`docs/api-rust.md` § Arrow version policy](api-rust.md#arrow-version-policy);
this document is kept as the design rationale.
**Scope:** `dora-node-api`, `dora-arrow-convert`, and every consumer that names an Arrow type.

## The problem

At 1.0, everything reachable from `dora-node-api`'s public API is frozen for
the life of 1.x. Arrow is reachable in three ways:

| Where | What |
|---|---|
| `apis/rust/node/src/lib.rs:89` | `pub use arrow;` — the whole crate |
| `libraries/arrow-convert/src/lib.rs:92` | `pub struct ArrowData(pub arrow::array::ArrayRef)` — public field, plus `Deref<Target = ArrayRef>` |
| `libraries/arrow-convert/src/lib.rs:44` | `trait IntoArrow { type A: arrow::array::Array; }` — associated-type bound |

`ArrowData` is a public field of `Event::Input`
(`apis/rust/node/src/event_stream/event.rs:31`), so it is on the hottest path
of every node.

Because those are in the contract, a user cannot mix dora's Arrow with their
own Arrow of a different major — cargo treats them as unrelated types and
`send_output` will not accept their arrays. Arrow ships a major roughly every
6–8 weeks. Frozen at 1.0, that means dora 1.x is pinned to Arrow 59 for the
life of the major, and anyone wanting current polars / datafusion / parquet
has to choose between them and dora.

**What is *not* a problem:** the wire format. dora serializes with Arrow IPC
(`apis/rust/node/src/node/arrow_utils.rs:44`), and IPC is a stable format
independent of the arrow-rs Rust API. Two dora nodes built against different
Arrow majors already exchange data correctly. The coupling is purely the
in-process Rust type boundary.

## The design

Four parts. Only the first two must land before 1.0; the rest are additive.

### 1. `DoraArray` — a dora-owned type in every signature

Replace the Arrow types in public signatures with a dora-owned newtype, so no
Arrow type appears in the frozen contract. This is what actually buys the
freedom: with it, dora can bump its internal Arrow in a *minor*.

- `ArrowData(pub ArrayRef)` and its `Deref<Target = ArrayRef>` become
  `DoraArray` with a private field.
- `IntoArrow::A: Array` becomes `fn into_arrow(self) -> DoraArray`.
- `Event::Input { data: DoraArray }`.
- `data_type() -> &arrow_schema::DataType`
  (`apis/rust/node/src/node/mod.rs:2690`) is the one signature naming a
  non-umbrella Arrow crate. It should return a dora-owned type — there is
  already a type-URN registry (`std/core/v1/UInt64`) — or move behind a
  feature gate.

`DoraArray` holds dora's *internal* Arrow version, so users on that major keep
today's ergonomics with no conversion.

**The raw accessor must be feature-gated.** An ungated
`DoraArray::as_array() -> &arrow::array::ArrayRef` would put Arrow straight
back into the public API and undo the whole exercise. So the direct accessor
lives behind the feature for dora's *current internal* major:

```rust
#[cfg(feature = "arrow-v59")]
impl DoraArray {
    pub fn as_array(&self) -> &arrow59::array::ArrayRef { &self.0 }
    pub fn into_inner(self) -> arrow59::array::ArrayRef { self.0 }
}
```

With `default = []` that is not in the default surface. When dora later moves
internally to 60, the *direct* accessor re-gates behind `arrow-v60` and
`arrow-v59` keeps a **converting** accessor instead — so old code still
compiles, it just pays the FFI hop.

### 2. `arrow_vN` re-exports, each feature-gated

`pub use arrow;` is dishonest: its meaning changes silently when dora bumps.
`pub use arrow as arrow_v59;` cannot — it either exists and means Arrow 59, or
it is visibly gone. That turns a bump from **mutating** into **additive**:
moving internally to 60 *adds* `arrow_v60` and leaves `arrow_v59` working.

```toml
[dependencies]
arrow = { workspace = true, features = ["ipc"] }   # internal, ungated
arrow58 = { package = "arrow", version = "58", optional = true }

[features]
default = []
arrow-v58 = ["dep:arrow58"]
arrow-v59 = []            # dora's internal version; re-export only
```

`arrow-v59` deliberately pulls **no extra dependency**. It re-exports the
already-present internal `arrow`, so exactly one copy of Arrow 59 links. Only
majors *other* than the internal one get an aliased `optional` dependency.
The feature exists so that naming the internal major is an explicit opt-in
like every other, and so the gating survives unchanged when 59 stops being
internal.

Two rules that matter:

- **dora's internal Arrow stays ungated.** Internal IPC encode/decode must be
  written against exactly one version or every call site needs `cfg`. Only the
  *extra* majors are optional. When dora later moves internally to 60, the
  previously-internal 59 demotes to just another optional compat feature.
- **`default = []`.** If the blessed version sits in `default` and `default`
  later changes, that is a breaking change for anyone relying on it — the
  silent-drift problem reinvented one level up. An empty default never drifts.
  This is viable precisely because of `DoraArray`: a node writing
  `node.send_output(id, meta, 42u32.into_arrow())?` never names an Arrow type,
  so the common path needs no feature at all.

Cargo features are additive, and these are honestly additive — enabling
`arrow-v58` only adds a re-export and some `From` impls. So if one crate in the
graph wants `arrow-v58` and another wants `arrow-v60`, cargo unifies to both
and each gets what it asked for. Two Arrow majors coexist fine: distinct
crates, distinct symbols, pure Rust, no C symbol collisions.

Cargo also *unifies semver-compatible* versions, so a user who declares
`arrow = "59"` themselves gets the same crate instance as
`dora_node_api::arrow_v59`. The types are interchangeable, not merely similar
— which is what makes this safe for people mixing dora with polars or parquet
on the same major.

### 3. `From`/`Into` impls via the C Data Interface

Conversion between majors must be zero-copy or it is a non-starter. The
mechanism is the Arrow C Data Interface, which exists for exactly this and is
**already used in-tree**: `apis/rust/operator/types/src/lib.rs:165` calls
`arrow::ffi::from_ffi`, and `Input`/`Output` carry
`FFI_ArrowArray`/`FFI_ArrowSchema` across a `#[repr(C)]` boundary.

The hop is `arrow58::ffi::to_ffi(&data)` → reinterpret → `arrow::ffi::from_ffi`.

> **The sharp edge.** `arrow58::ffi::FFI_ArrowArray` and
> `arrow::ffi::FFI_ArrowArray` are *distinct Rust types* to rustc even though
> both are `#[repr(C)]` against the same frozen Arrow C ABI. The bridge is
> therefore a transmute / field-wise reinterpret whose soundness rests on the
> spec being stable (it is — that is the point of the spec) and on both crates
> implementing it faithfully. This is unsafe glue and needs careful comments
> and tests. The thing to test first is the `release` callback: the newer arrow
> will invoke the releaser the older one installed. That is exactly what the
> interface is designed for, and exactly where a mistake becomes a
> use-after-free.

Adding a `From` impl later is additive and non-breaking, so extra majors can
land in minors whenever someone wants them.

### 4. Support-window policy

Adding an `arrow_vN` feature is non-breaking. **Removing one is breaking** and
must wait for a major, ideally after a release or two of `#[deprecated]`. At
~8 Arrow majors a year, realistically carry two or three. Write the policy
down; it is the only ongoing cost of this design.

## Why not the alternatives

- **Pin Arrow for all of 1.x.** A year-long 1.x ends ~8 majors behind. Users
  wanting current polars/datafusion/parquet must choose.
- **Carve Arrow out of the semver guarantee in writing.** Cheap and honest,
  and it is what the Arrow ecosystem itself does (`parquet`, `arrow-flight`
  version in lockstep). But dora minors then break builds, and users pin dora
  exactly. Acceptable fallback if the work above cannot land before 1.0.
- **Make the public boundary the C Data Interface itself.** The real
  decoupling, but it taxes ergonomics on the receive path: every node gains a
  fallible conversion on its hottest path and loses the `Deref` that makes
  `ArrowData` pleasant. `DoraArray` gets the same decoupling while keeping the
  matching-version case free.
- **Make `Event` generic over the payload.** Pushes a type parameter into every
  user signature.

## What landed, and where it deviated

All five steps below are implemented. Two things differ from the design as
written:

- **`EncodedSample::data_type()` is gated, not replaced by a type URN.**
  `dora_core::types::TypeRegistry` only covers the standard scalar/struct
  catalog; nested lists, dictionaries, unions and timestamps-with-timezone have
  no URN, so a URN-returning accessor would be lossy for exactly the outputs
  whose type a caller most needs. It is `#[cfg(feature = "arrow-v59")]`, with an
  ungated `type_name() -> String` for logging. `DoraArray` got the same
  treatment.
- **One Arrow-typed seam remains, deliberately.** `DoraArray` lives in
  `dora-arrow-convert`, but the code that must build and unwrap it
  (`dora-node-api`, the C/C++/Python bindings, record/replay) lives in other
  crates, and Rust has no cross-crate `pub(crate)`. Routing that through the
  `arrow-v59` *feature* would not work: `dora-node-api` would have to enable it
  unconditionally, and cargo feature unification would then hand every
  downstream user the ungated accessor — the silent-drift problem the gate
  exists to prevent. So `dora_arrow_convert::internal` is `pub`,
  `#[doc(hidden)]`-in-spirit, **not** re-exported from `dora-node-api`, and
  written down in `docs/api-rust.md` as exempt from the semver guarantee.
  `dora-node-api`'s own frozen surface is Arrow-free; that is the contract that
  matters.

Two additions the design did not call for but the migration needed:

- **`arrow_utils::IpcPayload`**, a dora-owned wrapper for the receive-side byte
  buffer, so `decode_arrow_ipc_zero_copy` / `InputDecoder::{set_schema,
  decode_batch}` do not name `arrow::buffer::Buffer`. It also removed three
  copies of the same hand-written `Buffer::from_custom_allocation` unsafe block.
- **Gated `IntoArrow` impls for Arrow 59 array types** (`ArrayRef`,
  `PrimitiveArray<T>`, `StructArray`, …), so `send_output(id, params,
  my_struct_array)` still works for callers on the internal major. A blanket
  `impl<A: arrow::array::Array> IntoArrow for A` is *not* possible — it overlaps
  with `impl IntoArrow for u8` under coherence's "upstream crates may add a new
  impl" rule — so the concrete types are listed.

## Order of work

1. Introduce `DoraArray`; migrate `ArrowData`, `IntoArrow`, `Event::Input`, and
   every consumer.
2. Replace `pub use arrow;` with gated `arrow_vN` re-exports, `default = []`.
3. Retype the leaked IPC encoder surface (see below).
4. Add `arrow-v58` with the FFI bridge as the worked example of an older major.
5. Write the support-window policy into `docs/api-rust.md`.

Steps 1–3 must precede 1.0. Steps 4–5 are additive, but 4 should land with the
rest so the conversion path has a real test rather than a hypothetical one.

### On the leaked IPC encoder surface (step 3)

`arrow_utils::ipc_encode` is `pub mod` inside a re-exported module, so
`encode_ipc_into`, `ipc_fast_path_len`, `encode_ipc_to_vec`,
`encode_schema_message`, `batch_fast_path_len`, `encode_batch_into`,
`schema_block_and_hash`, `encode_uint8_ipc_header`, `uint8_ipc_len`,
`PreparedUint8Ipc` and `InputDecoder` are frozen public API — and most of the
array-taking ones name `arrow::array::ArrayData`.

An earlier draft of this plan called hiding them "free surface reduction,
independent of the rest." **That was wrong on both counts.** They have real
cross-crate consumers:

- `binaries/record-node/src/main.rs:109,113,119` — `ipc_fast_path_len`,
  `encode_ipc_into`, `encode_ipc_to_vec`
- `binaries/daemon/src/lib.rs:9807` — `schema_block_and_hash`
- `apis/python/node/src/sample_handler.rs:325` — `PreparedUint8Ipc`
- `apis/rust/node/benches/arrow_framing.rs` and `tests/copy_count.rs` — reach
  them through the public path, and `copy_count.rs` also uses
  `encode_uint8_ipc_header` / `uint8_ipc_len`

So `pub(crate)` would break three crates plus a bench and a test. And
`#[doc(hidden)]` only hides from rustdoc — the items stay public and
semver-relevant, so `arrow::array::ArrayData` would **stay in the frozen API**,
which defeats the purpose.

**Do this instead:** keep them public for their real consumers, and retype the
array-taking ones to take `&DoraArray` rather than `&arrow::array::ArrayData`.
Arrow then leaves those signatures without anything being hidden or broken —
in-workspace callers pass a `DoraArray`, which is a newtype and therefore free.
`schema_block_and_hash` already takes bytes and needs no change;
`encode_schema_message(&DataType)` needs the same dora-owned-type treatment as
`data_type()` in part 1.

This is why the step is **ordered after `DoraArray` exists** rather than first:
it is not independent of it.
