# Native `rmw_zenoh` Full-Parity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an explicitly selectable native Zenoh backend to every Dora ROS 2 bridge surface, interoperable with `rmw_zenoh_cpp` graph discovery, topics, services, actions, and supported QoS on Humble and a pinned REP-2016 distribution.

**Architecture:** Introduce backend-neutral ROS entity types and retain the current `ros2-client` implementation behind a DDS adapter. Add a direct Zenoh backend whose protocol codecs are isolated from session/entity logic, whose graph cache drives readiness, and whose action support composes the completed topic and service layers.

**Tech Stack:** Rust 2024, `zenoh`/`zenoh-ext` 1.9, `ros2-client` 0.8.x, RustDDS 0.11.4, Serde, PyO3, CXX, Arrow, Docker Compose, ROS 2 Humble and Kilted.

## Global Constraints

- Preserve DDS as the default for existing YAML, Python, generated Rust, and generated C++ callers.
- Support exactly `humble` and `rep2016` compatibility profiles initially; do not infer a profile from `ROS_DISTRO`.
- Do not infer transport from `RMW_IMPLEMENTATION`.
- Use authoritative installed ROS type descriptions for REP-2016 hashes; fail closed when a hash cannot be established.
- Keep ROS payload serialization in the existing Arrow/CDR layer.
- Keep the ROS Zenoh session separate from Dora's internal data-plane session.
- Do not claim endpoint compatibility until a real pinned `rmw_zenoh_cpp` peer passes.
- Follow RED-GREEN-IMPROVE and keep each task independently reviewable.

---

### Task 1: Add transport configuration with DDS-compatible defaults

**Files:**
- Modify: `libraries/message/src/descriptor.rs`
- Modify: `libraries/core/src/descriptor/mod.rs`
- Modify: `libraries/core/src/descriptor/validate.rs`
- Modify: `libraries/core/tests/dataflow-descriptor-schema.json`

**Interfaces:**
- Produces: `Ros2TransportConfig::{Dds, Zenoh { compatibility, config_uri }}`
- Produces: `RmwZenohCompatibility::{Humble, Rep2016}`
- Preserves: deserializing an existing `Ros2BridgeConfig` selects DDS

- [ ] **Step 1: Write descriptor tests that define the serialized contract**

Add tests beside the existing ROS2 descriptor tests:

```rust
#[test]
fn ros2_transport_defaults_to_dds() {
    let config: Ros2BridgeConfig = serde_yaml::from_str(
        "topic: /chatter\nmessage_type: std_msgs/String\n",
    )
    .unwrap();
    assert!(matches!(config.transport, Ros2TransportConfig::Dds));
}

#[test]
fn parses_humble_zenoh_transport() {
    let config: Ros2BridgeConfig = serde_yaml::from_str(
        "transport:\n  kind: zenoh\n  compatibility: humble\n  config_uri: /tmp/rmw.json5\n\
         topic: /chatter\nmessage_type: std_msgs/String\n",
    )
    .unwrap();
    assert_eq!(
        config.transport,
        Ros2TransportConfig::Zenoh {
            compatibility: RmwZenohCompatibility::Humble,
            config_uri: Some("/tmp/rmw.json5".into()),
        }
    );
}

#[test]
fn rejects_unknown_zenoh_compatibility() {
    let error = serde_yaml::from_str::<Ros2BridgeConfig>(
        "transport:\n  kind: zenoh\n  compatibility: automatic\n\
         topic: /chatter\nmessage_type: std_msgs/String\n",
    )
    .unwrap_err();
    assert!(error.to_string().contains("unknown variant `automatic`"));
}
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
cargo test -p dora-message ros2_transport -- --nocapture
```

Expected: compilation fails because `Ros2TransportConfig`, `RmwZenohCompatibility`, and `transport` do not exist.

- [ ] **Step 3: Add the tagged enums and default field**

Add `PathBuf` to the descriptor imports and define:

```rust
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Ros2TransportConfig {
    #[default]
    Dds,
    Zenoh {
        compatibility: RmwZenohCompatibility,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        config_uri: Option<PathBuf>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RmwZenohCompatibility {
    Humble,
    Rep2016,
}
```

Add `#[serde(default)] pub transport: Ros2TransportConfig` to `Ros2BridgeConfig` and `transport: Default::default()` to its `Default` implementation. Reexport both enums from `libraries/core/src/descriptor/mod.rs`.

- [ ] **Step 4: Add semantic validation for the explicit configuration**

In `validate_ros2_config`, reject an empty `config_uri` path:

```rust
if let Ros2TransportConfig::Zenoh {
    config_uri: Some(uri), ..
} = &config.transport
    && uri.as_os_str().is_empty()
{
    bail!("node `{node_id}`: ros2 Zenoh config_uri must not be empty");
}
```

- [ ] **Step 5: Regenerate the checked-in descriptor schema**

Run:

```bash
cargo run -p dora-core --bin generate_schema
```

Then verify `libraries/core/tests/dataflow-descriptor-schema.json` contains `dds`, `zenoh`, `humble`, and `rep2016` and has no unrelated diff.

- [ ] **Step 6: Run GREEN validation**

```bash
cargo test -p dora-message ros2_transport
cargo test -p dora-core ros2
cargo fmt --all -- --check
```

Expected: all selected tests pass and formatting is clean.

- [ ] **Step 7: Commit the independently reviewable configuration change**

```bash
git add libraries/message/src/descriptor.rs libraries/core/src/descriptor libraries/core/tests/dataflow-descriptor-schema.json
git commit -m "feat(ros2-bridge): add explicit transport configuration"
```

### Task 2: Introduce backend-neutral types and adapt DDS without behavior changes

**Files:**
- Create: `libraries/extensions/ros2-bridge/src/transport/mod.rs`
- Create: `libraries/extensions/ros2-bridge/src/transport/types.rs`
- Create: `libraries/extensions/ros2-bridge/src/transport/dds.rs`
- Modify: `libraries/extensions/ros2-bridge/src/lib.rs`
- Modify: `libraries/extensions/ros2-bridge/Cargo.toml`

**Interfaces:**
- Produces: neutral `Ros2Qos`, `MessageMetadata`, `RequestId`, `TransportError`
- Produces: `transport::Context` and `transport::Node` backend enums
- Consumes: Task 1 transport configuration
- Preserves: existing `ros2_client` and `rustdds` reexports

- [ ] **Step 1: Add compile-time and conversion tests**

In `transport/dds.rs`, start with tests asserting all current QoS fields survive a neutral-to-RustDDS round trip and `Context::new(Dds)` creates the DDS variant. Include reliable/keep-all/transient-local and best-effort/keep-last cases.

```rust
#[test]
fn dds_qos_adapter_preserves_reliable_keep_all() {
    let qos = Ros2Qos {
        reliability: Reliability::Reliable {
            max_blocking_time: Duration::from_millis(250),
        },
        durability: Durability::TransientLocal,
        history: History::KeepAll,
        liveliness: Liveliness::Automatic { lease_duration: None },
    };
    let rustdds = to_rustdds_qos(&qos);
    assert_eq!(from_rustdds_qos(&rustdds), qos);
}
```

- [ ] **Step 2: Verify RED**

```bash
cargo test -p dora-ros2-bridge transport::dds
```

Expected: module and neutral types are missing.

- [ ] **Step 3: Define neutral value types**

Implement the types with exhaustive enums, `thiserror` errors, and no backend types in public fields:

```rust
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MessageMetadata {
    pub sequence_number: i64,
    pub source_timestamp_ns: i64,
    pub publisher_gid: [u8; 16],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RequestId {
    pub sequence_number: i64,
    pub client_gid: [u8; 16],
}
```

Define explicit `Reliability`, `Durability`, `History`, and `Liveliness` enums. Use `std::time::Duration`; represent infinity as `None`.

- [ ] **Step 4: Move existing DDS construction behind adapters**

Wrap `ros2_client::Context` and `Node` in `dds::Context` and `dds::Node`. Move `detect_service_mapping` into the DDS module and reexport it at the old path. Add neutral-to-RustDDS QoS conversion. Do not change endpoint call sites yet.

- [ ] **Step 5: Run DDS regression tests**

```bash
cargo test -p dora-ros2-bridge --lib
cargo check -p dora-ros2-bridge --no-default-features
cargo test -p dora-ros2-bridge-python
```

Expected: all existing tests pass; public imports still compile.

- [ ] **Step 6: Commit the neutral foundation**

```bash
git add libraries/extensions/ros2-bridge
git commit -m "refactor(ros2-bridge): isolate DDS transport types"
```

### Task 3: Resolve ROS DDS type identities and compatibility profiles

**Files:**
- Create: `libraries/extensions/ros2-bridge/src/transport/zenoh/compatibility.rs`
- Create: `libraries/extensions/ros2-bridge/msg-gen/src/type_description.rs`
- Modify: `libraries/extensions/ros2-bridge/msg-gen/src/types/message.rs`
- Modify: `libraries/extensions/ros2-bridge/msg-gen/src/types/service.rs`
- Modify: `libraries/extensions/ros2-bridge/msg-gen/src/types/action.rs`
- Create: `libraries/extensions/ros2-bridge/msg-gen/test_type_descriptions/`

**Interfaces:**
- Produces: `RosTypeIdentity { ros_name, dds_name, hash }`
- Produces: `TypeHash::{HumbleUnsupported, Rep2016([u8; 32])}`
- Produces: `TypeDescriptionResolver::resolve(package, kind, name)`
- Consumes: Task 1 compatibility enum

- [ ] **Step 1: Check in authoritative hash fixtures with provenance**

Add fixture JSON copied from a pinned Kilted installation for `std_msgs/msg/String`, `nav_msgs/msg/Odometry`, `example_interfaces/srv/AddTwoInts`, and `example_interfaces/action/Fibonacci`. Add `README.md` recording the ROS image digest and the command used to extract `type_description_interfaces/msg/TypeDescription` and `RIHS01` hashes.

- [ ] **Step 2: Add failing identity tests**

```rust
#[test]
fn humble_topic_identity_uses_unsupported_hash_literal() {
    let id = resolve_message(Humble, "std_msgs", "String", &fixtures()).unwrap();
    assert_eq!(id.dds_name, "std_msgs::msg::dds_::String_");
    assert_eq!(id.key_hash_component(), "TypeHashNotSupported");
}

#[test]
fn rep2016_identity_matches_installed_fixture() {
    let id = resolve_message(Rep2016, "std_msgs", "String", &fixtures()).unwrap();
    assert_eq!(
        id.key_hash_component(),
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
    );
}

#[test]
fn rep2016_missing_description_fails_closed() {
    let error = resolve_message(Rep2016, "missing_pkg", "Missing", &fixtures()).unwrap_err();
    assert!(matches!(error, TypeIdentityError::DescriptionNotFound { .. }));
}
```

- [ ] **Step 3: Run RED**

```bash
cargo test -p dora-ros2-bridge-msg-gen type_description
cargo test -p dora-ros2-bridge compatibility
```

- [ ] **Step 4: Implement the installed-description loader**

Search each `AMENT_PREFIX_PATH` entry for the distribution's installed type-description artifact. Deserialize the complete description, verify its declared type name, parse the official hash, and retain nested descriptions for diagnostics. Do not derive REP-2016 hashes from `.msg` source.

- [ ] **Step 5: Implement DDS naming and action expansion**

Use explicit constructors:

```rust
fn message_dds_name(package: &str, name: &str) -> String {
    format!("{package}::msg::dds_::{name}_")
}

fn service_dds_name(package: &str, name: &str) -> String {
    format!("{package}::srv::dds_::{name}_")
}
```

Resolve action component service/message types from the generated ROS action naming convention, and test every component against fixture output.

- [ ] **Step 6: Run GREEN and malformed-fixture tests**

```bash
cargo test -p dora-ros2-bridge-msg-gen type_description
cargo test -p dora-ros2-bridge compatibility
```

Expected: known hashes and names match; truncated, mismatched, and missing fixtures return typed errors.

- [ ] **Step 7: Commit type identity support**

```bash
git add libraries/extensions/ros2-bridge/msg-gen libraries/extensions/ros2-bridge/src/transport/zenoh/compatibility.rs
git commit -m "feat(ros2-bridge): resolve rmw zenoh type identities"
```

### Task 4: Implement protocol key, attachment, and QoS codecs

**Files:**
- Create: `libraries/extensions/ros2-bridge/src/transport/zenoh/mod.rs`
- Create: `libraries/extensions/ros2-bridge/src/transport/zenoh/keyexpr.rs`
- Create: `libraries/extensions/ros2-bridge/src/transport/zenoh/attachment.rs`
- Create: `libraries/extensions/ros2-bridge/src/transport/zenoh/qos.rs`
- Create: `libraries/extensions/ros2-bridge/tests/fixtures/rmw_zenoh/`
- Modify: `libraries/extensions/ros2-bridge/Cargo.toml`
- Modify: root `Cargo.toml`

**Interfaces:**
- Produces: `DataKey::new(domain, fully_qualified_name, identity)`
- Produces: `LivelinessKey::{node, endpoint, parse}`
- Produces: `Attachment::{encode, decode}`
- Produces: `ZenohQosMapping::from_ros_qos`

- [ ] **Step 1: Add upstream-generated golden fixtures**

Check in exact data keys, all five liveliness entity keys, QoS components, and attachment bytes produced by pinned Humble and Kilted `rmw_zenoh_cpp`. Record upstream commit SHAs in the fixture README.

- [ ] **Step 2: Add table-driven failing tests**

Cover root and nested namespaces, domain 0 and 42, hidden names, all entity kinds, default and non-default QoS, sequence boundaries, timestamps before and after epoch, invalid GID length, truncated bytes, extra bytes, malformed percent escaping, and cross-domain tokens.

```rust
#[test]
fn attachment_matches_humble_golden_bytes() {
    let attachment = Attachment {
        sequence_number: 7,
        source_timestamp_ns: 1_725_000_000_000_000_000,
        gid: [0x2a; 16],
    };
    assert_eq!(attachment.encode().unwrap(), fixture_bytes("attachment-seq7.bin"));
    assert_eq!(Attachment::decode(&fixture_bytes("attachment-seq7.bin")).unwrap(), attachment);
}
```

- [ ] **Step 3: Run RED**

```bash
cargo test -p dora-ros2-bridge --test rmw_zenoh_protocol
```

- [ ] **Step 4: Add Zenoh dependencies behind a crate feature**

Add `zenoh` and `zenoh-ext` workspace dependencies to `dora-ros2-bridge` under a `rmw-zenoh` feature. Ensure the standalone and Python crates can enable it explicitly. Do not create a second Zenoh version.

- [ ] **Step 5: Implement strict codecs**

Use `zenoh::bytes::ZBytes` serializers for attachments so the representation matches upstream. Use parsed component structs for liveliness rather than concatenating ad hoc strings. Validate all ROS-derived components before constructing `OwnedKeyExpr`.

- [ ] **Step 6: Run GREEN, dependency, and feature checks**

```bash
cargo test -p dora-ros2-bridge --features rmw-zenoh --test rmw_zenoh_protocol
cargo tree -d | rg '^zenoh v'
cargo check -p dora-ros2-bridge --no-default-features
```

Expected: protocol tests pass; only the workspace Zenoh release is present; DDS-only builds remain valid.

- [ ] **Step 7: Commit protocol codecs**

```bash
git add Cargo.toml Cargo.lock libraries/extensions/ros2-bridge
git commit -m "feat(ros2-bridge): add rmw zenoh protocol codecs"
```

### Task 5: Implement Zenoh context lifecycle and ROS graph cache

**Files:**
- Create: `libraries/extensions/ros2-bridge/src/transport/zenoh/graph.rs`
- Modify: `libraries/extensions/ros2-bridge/src/transport/zenoh/mod.rs`
- Create: `libraries/extensions/ros2-bridge/tests/rmw_zenoh_graph.rs`

**Interfaces:**
- Produces: `zenoh::Context::open(options)` and `Context::create_node`
- Produces: `GraphCache::{apply_put, apply_delete, matching_services, snapshot}`
- Produces: RAII `NodeToken` and `EndpointToken`
- Consumes: Task 4 liveliness codec

- [ ] **Step 1: Write graph state-machine tests**

Use a fake token source and deterministic clock. Verify initial query ordering, put/delete idempotence, duplicate entity IDs, domain isolation, malformed token rejection, endpoint-before-node arrival, node removal, waiter wakeup, and shutdown wakeup.

```rust
#[test]
fn graph_delete_is_idempotent_and_wakes_waiters_once() {
    let graph = GraphCache::new(7);
    let endpoint = fixture_endpoint(EntityKind::ServiceServer);
    assert!(graph.apply_put(endpoint.clone()).unwrap().changed());
    assert!(graph.apply_delete(&endpoint.key).unwrap().changed());
    assert!(!graph.apply_delete(&endpoint.key).unwrap().changed());
    assert!(graph.snapshot().entities.is_empty());
}
```

- [ ] **Step 2: Run RED**

```bash
cargo test -p dora-ros2-bridge --features rmw-zenoh --test rmw_zenoh_graph
```

- [ ] **Step 3: Implement configuration resolution and session open**

Implement `explicit config_uri > ZENOH_SESSION_CONFIG_URI > embedded pinned default`. Parse before opening. Store the selected source in diagnostics. Open exactly one session per context and perform the initial liveliness get before marking graph initialization complete.

- [ ] **Step 4: Implement RAII entity tokens**

Allocate stable context-local numeric IDs and 16-byte GIDs. Declare node tokens on node creation. Endpoint constructors declare their token only after the corresponding data entity succeeds. Drop endpoint tokens before data entities and node tokens before the session.

- [ ] **Step 5: Implement graph cache and readiness watches**

Use a lock-protected entity map plus a monotonic generation counter and async notification. Parsing occurs before taking the write lock. Service matching compares domain, fully qualified name, DDS identity, hash, entity kind, and QoS compatibility.

- [ ] **Step 6: Run GREEN and leak checks**

```bash
cargo test -p dora-ros2-bridge --features rmw-zenoh --test rmw_zenoh_graph
cargo test -p dora-ros2-bridge --features rmw-zenoh graph::tests::drop_removes_all_tokens -- --nocapture
```

Expected: state-machine tests pass and the test router reports no remaining Dora tokens after context drop.

- [ ] **Step 7: Commit graph support**

```bash
git add libraries/extensions/ros2-bridge/src/transport/zenoh libraries/extensions/ros2-bridge/tests
git commit -m "feat(ros2-bridge): add rmw zenoh graph discovery"
```

### Task 6: Add Zenoh topic publication, subscription, and QoS behavior

**Files:**
- Create: `libraries/extensions/ros2-bridge/src/transport/zenoh/pubsub.rs`
- Modify: `libraries/extensions/ros2-bridge/src/transport/mod.rs`
- Modify: `libraries/extensions/ros2-bridge/arrow/src/lib.rs`
- Create: `libraries/extensions/ros2-bridge/tests/rmw_zenoh_pubsub.rs`

**Interfaces:**
- Produces: transport-neutral `Publisher<TypedValue>` and `Subscription<ArrayData>` Zenoh variants
- Produces: raw CDR encode/decode entry points in the Arrow bridge
- Consumes: Tasks 3-5 identity, protocol, graph, and QoS

- [ ] **Step 1: Add raw CDR boundary tests**

Expose narrowly scoped functions that serialize `TypedValue` to bytes and deserialize bytes with `StructDeserializer`. Assert the bytes match the current RustDDS serializer for representative primitive, nested, sequence, bounded string, and `nav_msgs/Odometry` values.

- [ ] **Step 2: Add failing two-context topic tests**

Start an in-process test router/session configuration. Verify both directions, ordering and metadata, namespace/domain isolation, malformed attachment rejection, bounded malformed CDR rejection, publisher drop, subscriber shutdown, and no unbounded queue.

- [ ] **Step 3: Run RED**

```bash
cargo test -p dora-ros2-bridge-arrow raw_cdr
cargo test -p dora-ros2-bridge --features rmw-zenoh --test rmw_zenoh_pubsub
```

- [ ] **Step 4: Implement publisher behavior**

Declare standard or advanced publishers according to Task 4's mapping. Maintain an `AtomicI64` sequence starting at 1, generate Unix nanoseconds with checked conversion, attach the endpoint GID, and return a typed congestion/session error.

- [ ] **Step 5: Implement subscriber behavior**

Declare standard or advanced subscribers, parse attachments before payloads, enforce a configured maximum payload size, deserialize CDR on a bounded worker path, and send results through a bounded channel. Record malformed and dropped sample counters.

- [ ] **Step 6: Add QoS behavior tests**

Verify best-effort, reliable, keep-last depths 1 and 10, keep-all congestion policy, and transient-local late join. Assert unsupported deadline/liveliness event registration returns `TransportError::UnsupportedQosEvent`.

- [ ] **Step 7: Run GREEN**

```bash
cargo test -p dora-ros2-bridge-arrow raw_cdr
cargo test -p dora-ros2-bridge --features rmw-zenoh --test rmw_zenoh_pubsub
cargo clippy -p dora-ros2-bridge --features rmw-zenoh -- -D warnings
```

- [ ] **Step 8: Commit topic transport**

```bash
git add libraries/extensions/ros2-bridge libraries/extensions/ros2-bridge/arrow
git commit -m "feat(ros2-bridge): add rmw zenoh topics"
```

### Task 7: Add Zenoh service clients, servers, and readiness

**Files:**
- Create: `libraries/extensions/ros2-bridge/src/transport/zenoh/service.rs`
- Modify: `libraries/extensions/ros2-bridge/src/transport/mod.rs`
- Create: `libraries/extensions/ros2-bridge/tests/rmw_zenoh_service.rs`

**Interfaces:**
- Produces: neutral service client/server Zenoh variants
- Produces: `wait_for_service(deadline)` backed by `GraphCache`
- Consumes: Task 4 attachments and Task 5 graph matching

- [ ] **Step 1: Write failing service state tests**

Cover request/response CDR, request ID extraction, out-of-order responses, duplicate sequence numbers from different GIDs, query timeout, server disappearance, multiple complete servers, pending limit 64, 30-second expiry under a fake clock, malformed attachments, and shutdown.

- [ ] **Step 2: Run RED**

```bash
cargo test -p dora-ros2-bridge --features rmw-zenoh --test rmw_zenoh_service
```

- [ ] **Step 3: Implement the service server**

Declare a complete queryable on the service data key. Convert each query attachment into `RequestId`, retain the reply handle in a bounded pending map, expose request CDR to the bridge, and reply with the same sequence/client GID plus a fresh source timestamp. Reject a response whose request is absent or expired.

- [ ] **Step 4: Implement the service client**

Declare a querier with `ALL_COMPLETE` semantics and no consolidation. Generate a unique sequence, attach the client GID, correlate replies by both fields, bound outstanding calls, and surface remote errors distinctly from timeouts.

- [ ] **Step 5: Implement graph-backed readiness**

Wait on graph generation changes until an exactly matching `SS` entity appears or the caller's deadline expires. Return immediately if already present and return `TransportClosed` if context shutdown wins.

- [ ] **Step 6: Run GREEN and stress tests**

```bash
cargo test -p dora-ros2-bridge --features rmw-zenoh --test rmw_zenoh_service
cargo test -p dora-ros2-bridge --features rmw-zenoh service_concurrent -- --nocapture
```

- [ ] **Step 7: Commit service transport**

```bash
git add libraries/extensions/ros2-bridge/src/transport libraries/extensions/ros2-bridge/tests
git commit -m "feat(ros2-bridge): add rmw zenoh services"
```

### Task 8: Refactor actions to compose transport-neutral topics and services

**Files:**
- Create: `libraries/extensions/ros2-bridge/src/transport/action.rs`
- Modify: `libraries/extensions/ros2-bridge/python/src/lib.rs`
- Modify: `binaries/ros2-bridge-node/src/main.rs`
- Modify: `libraries/extensions/ros2-bridge/msg-gen/src/types/action.rs`
- Create: `libraries/extensions/ros2-bridge/tests/rmw_zenoh_action.rs`

**Interfaces:**
- Produces: transport-neutral action client/server used by all surfaces
- Consumes: Tasks 6 and 7 topic/service entities
- Preserves: max 8 concurrent goals and existing Dora metadata contracts

- [ ] **Step 1: Write endpoint-expansion and state-machine tests**

Assert exact names and types for send-goal, get-result, cancel-goal, feedback, and status. Cover accepted/rejected goals, feedback ordering, result before/after request, cancellation, abort, concurrent goals, unknown goal IDs, server loss, and long-lived get-result.

- [ ] **Step 2: Run RED**

```bash
cargo test -p dora-ros2-bridge --features rmw-zenoh --test rmw_zenoh_action
```

- [ ] **Step 3: Extract existing action orchestration**

Move goal maps, concurrency counters, feedback routing, cancellation, and terminal-state handling out of Python and the standalone binary into `transport/action.rs`. Parameterize it only over neutral service and topic entities. Preserve existing timeouts except that Zenoh get-result uses the pinned upstream effectively unbounded timeout.

- [ ] **Step 4: Bind DDS to the shared orchestration**

Adapt current `ros2-client` action entities or their component endpoints to the neutral layer. Run all existing DDS action tests before enabling Zenoh to prove the refactor is behavior-neutral.

- [ ] **Step 5: Bind Zenoh component endpoints**

Construct the three Task 7 services and two Task 6 topics with action-specific types and QoS. Derive action availability from the complete compatible endpoint set.

- [ ] **Step 6: Run GREEN**

```bash
cargo test -p dora-ros2-bridge --features rmw-zenoh --test rmw_zenoh_action
cargo test -p dora-ros2-bridge-python action
cargo test -p dora-ros2-bridge --features ros2-examples action
```

- [ ] **Step 7: Commit shared actions**

```bash
git add libraries/extensions/ros2-bridge binaries/ros2-bridge-node
git commit -m "feat(ros2-bridge): compose actions across transports"
```

### Task 9: Integrate transport selection into the standalone YAML bridge

**Files:**
- Modify: `binaries/ros2-bridge-node/Cargo.toml`
- Modify: `binaries/ros2-bridge-node/src/main.rs`
- Modify: `libraries/core/src/descriptor/validate.rs`
- Create: `binaries/ros2-bridge-node/tests/transport_selection.rs`
- Create: `examples/ros2-bridge/yaml-bridge/dataflow-zenoh.yml`
- Create: `examples/ros2-bridge/yaml-bridge-service/dataflow-client-zenoh.yml`
- Create: `examples/ros2-bridge/yaml-bridge-action/dataflow-zenoh.yml`

**Interfaces:**
- Consumes: Task 1 configuration and Tasks 2/6/7/8 neutral entities
- Produces: YAML-selectable topic/service/action parity

- [ ] **Step 1: Add failing transport-selection tests**

Inject fake DDS and Zenoh context factories. Assert omitted configuration calls DDS, explicit Zenoh passes profile/config URI, invalid Zenoh config fails before `DoraNode::init_from_env`, and `RMW_IMPLEMENTATION` does not select transport.

- [ ] **Step 2: Run RED**

```bash
cargo test -p dora-ros2-bridge-node transport_selection
```

- [ ] **Step 3: Replace concrete endpoint collections**

Change publisher, subscription, service, and action fields in `main.rs` to neutral entity enums. Construct one context from `config.transport`, one node, and all configured endpoints from that node. Remove duplicate RustDDS QoS conversion in favor of shared neutral conversion.

- [ ] **Step 4: Improve startup diagnostics**

Log transport/profile/config source without secrets. When DDS is selected and `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, emit an actionable mismatch warning. When Zenoh is selected, do not call `detect_service_mapping`.

- [ ] **Step 5: Add example configurations**

Add Zenoh variants for one topic, service, and action YAML example. Keep existing examples unchanged to prove the default.

- [ ] **Step 6: Run GREEN and DDS regression checks**

```bash
cargo test -p dora-ros2-bridge-node
cargo test -p dora-core ros2
cargo check -p dora-ros2-bridge-node
```

- [ ] **Step 7: Commit YAML integration**

```bash
git add binaries/ros2-bridge-node libraries/core examples/ros2-bridge
git commit -m "feat(ros2-bridge): select Zenoh transport in YAML"
```

### Task 10: Integrate Python transport selection and node features

**Files:**
- Modify: `libraries/extensions/ros2-bridge/python/Cargo.toml`
- Modify: `libraries/extensions/ros2-bridge/python/src/lib.rs`
- Modify: `libraries/extensions/ros2-bridge/python/src/qos.rs`
- Modify: `libraries/extensions/ros2-bridge/python/test_utils.py`
- Create: `libraries/extensions/ros2-bridge/python/tests/test_transport.py`

**Interfaces:**
- Produces: `Ros2Transport.dds()` and `.zenoh(compatibility, config_uri)`
- Changes: `Ros2Context(..., transport=None)` where `None` means DDS
- Consumes: all neutral transport entities

- [ ] **Step 1: Write failing Python API tests**

```python
def test_context_defaults_to_dds():
    context = dora.Ros2Context(ros_paths=[])
    assert context.transport_kind == "dds"

def test_zenoh_transport_requires_known_profile():
    with pytest.raises(ValueError, match="compatibility"):
        dora.Ros2Transport.zenoh("automatic")
```

Add Rust/PyO3 tests proving `Ros2NodeOptions` does not own transport, context transport is immutable, and existing positional construction remains accepted.

- [ ] **Step 2: Run RED**

```bash
cargo test -p dora-ros2-bridge-python transport
```

- [ ] **Step 3: Add Python configuration classes and neutral fields**

Store `transport::Context` in `Ros2Context`, `transport::Node` in `Ros2Node`, and neutral endpoint enums in Python wrapper classes. Convert `Ros2QosPolicies` into `Ros2Qos`, with DDS conversion occurring only inside the DDS adapter.

- [ ] **Step 4: Enable parameters and rosout after component parity**

Build parameter services/events and rosout from neutral service/topic APIs. Add tests for local get/set, remote `rclpy` set, parameter events, and rosout visibility over Zenoh. If a requested feature has no neutral implementation, return a typed capability error during node construction.

- [ ] **Step 5: Run GREEN and wheel checks**

```bash
cargo test -p dora-ros2-bridge-python
maturin build --manifest-path libraries/extensions/ros2-bridge/python/Cargo.toml
```

Record the wheel-size delta in the PR description and fail dependency review if a second Zenoh stack appears.

- [ ] **Step 6: Commit Python integration**

```bash
git add libraries/extensions/ros2-bridge/python
git commit -m "feat(ros2-bridge): expose Zenoh transport to Python"
```

### Task 11: Integrate generated Rust and C++ surfaces

**Files:**
- Modify: `libraries/extensions/ros2-bridge/msg-gen/src/lib.rs`
- Modify: `libraries/extensions/ros2-bridge/msg-gen/src/types/message.rs`
- Modify: `libraries/extensions/ros2-bridge/msg-gen/src/types/service.rs`
- Modify: `libraries/extensions/ros2-bridge/msg-gen/src/types/action.rs`
- Modify: generated API compile fixtures under `examples/ros2-bridge/rust/` and `examples/ros2-bridge/c++/`

**Interfaces:**
- Preserves: `init_ros2_context()` as DDS
- Produces: `init_ros2_context_with_transport(Ros2TransportConfig)`
- Produces: CXX-safe transport/profile enums without Zenoh/RustDDS internals

- [ ] **Step 1: Add generated-token and compile tests**

Assert generated output contains both initializer signatures, old sample code compiles unchanged, and new Rust/C++ samples can request Humble Zenoh. Assert generated publisher/subscription/service/action structs contain neutral bridge types.

- [ ] **Step 2: Run RED**

```bash
cargo test -p dora-ros2-bridge-msg-gen transport
cargo check -p rust-ros2-example-node
```

- [ ] **Step 3: Generate CXX-safe configuration**

Define a CXX enum for `Dds`, `ZenohHumble`, and `ZenohRep2016`, plus an optional UTF-8 config URI string. Convert it once at context initialization. Reject invalid URI encoding before session open.

- [ ] **Step 4: Replace generated concrete entity types**

Generate neutral context/node/topic/publisher/subscription/service/action wrappers. Keep generated message structs and CDR Serde implementations unchanged.

- [ ] **Step 5: Run GREEN across languages**

```bash
cargo test -p dora-ros2-bridge-msg-gen
cargo check -p rust-ros2-example-node
cargo check -p dora-ros2-bridge --examples --features ros2-examples
```

- [ ] **Step 6: Commit generated API integration**

```bash
git add libraries/extensions/ros2-bridge/msg-gen examples/ros2-bridge/rust examples/ros2-bridge/c++
git commit -m "feat(ros2-bridge): expose Zenoh transport in native APIs"
```

### Task 12: Add real `rmw_zenoh_cpp` interoperability harnesses

**Files:**
- Create: `docker-compose.ros2-zenoh.yml`
- Create: `scripts/ros2-zenoh-interop.sh`
- Create: `tests/ros2-zenoh/README.md`
- Create: `tests/ros2-zenoh/fixtures/`
- Create: `tests/ros2-zenoh/peers/`
- Modify: `.github/workflows/nightly.yml`
- Modify: `scripts/qa/ci-nightly-jobs.sh`
- Modify: `Makefile`

**Interfaces:**
- Proves: Humble and pinned Kilted/REP-2016 interoperability
- Consumes: all implemented endpoint classes and surfaces

- [ ] **Step 1: Create pinned container definitions**

Pin ROS base images by digest and install exact `ros-<distro>-rmw-zenoh-cpp` versions. Run `rmw_zenohd` as a health-checked service. Mount the source read-only except for Cargo target/cache volumes. Record package and upstream commit versions at test startup.

- [ ] **Step 2: Add real Python peers**

Create deterministic `rclpy` publisher/subscriber, service client/server, and action client/server programs using standard messages. Each peer emits machine-readable readiness and result lines and exits nonzero on mismatched payload, metadata, cancellation, or timeout.

- [ ] **Step 3: Implement the matrix driver**

`scripts/ros2-zenoh-interop.sh <humble|kilted> <case|all>` starts the router, waits with a bounded timeout, runs each direction, captures diagnostics, and always tears down containers. Cases are:

```text
topic-pub topic-sub service-client service-server
action-client action-server graph domain namespace qos-transient-local
```

- [ ] **Step 4: Run each phase's initial expected failure before its implementation**

For the first transport PR, run `topic-sub` and retain the failure showing Dora cannot match the `rmw_zenoh` peer. For later PRs, add the relevant case before production code and confirm the expected missing-capability failure.

- [ ] **Step 5: Run the completed matrix**

```bash
scripts/ros2-zenoh-interop.sh humble all
scripts/ros2-zenoh-interop.sh kilted all
```

Expected: every case prints `PASS`, ROS CLI sees Dora entities, domain 42 cannot see domain 0, namespaces resolve correctly, and teardown leaves no containers or routers.

- [ ] **Step 6: Add non-blocking nightly jobs, then promote after stabilization**

Add separate Humble and Kilted jobs rather than altering the existing DDS `ros2-bridge` job. Upload logs on failure. After an agreed stabilization window, make both jobs required for changes under ROS bridge paths.

- [ ] **Step 7: Commit interoperability infrastructure**

```bash
git add docker-compose.ros2-zenoh.yml scripts/ros2-zenoh-interop.sh tests/ros2-zenoh .github/workflows/nightly.yml scripts/qa/ci-nightly-jobs.sh Makefile
git commit -m "test(ros2-bridge): verify rmw zenoh interoperability"
```

### Task 13: Document support and run release gates

**Files:**
- Modify: `examples/ros2-bridge/README.md`
- Modify: `guide/src/advanced/ros2-bridge.md`
- Modify: `guide/src/concepts/dataflow-yaml.md`
- Modify: `docs/testing-guide.md`
- Modify: `README.md`

**Interfaces:**
- Documents: selection, router, profiles, QoS, limitations, diagnostics, examples
- Verifies: all acceptance criteria in the design specification

- [ ] **Step 1: Write documentation assertions before prose**

Add doc tests or descriptor fixture tests for the exact YAML and Python snippets. Add a link checker assertion for upstream design/profile references.

- [ ] **Step 2: Document the operational contract**

Explain explicit transport/profile selection, router startup, config URI precedence, `ROS_DOMAIN_ID`, namespaces, supported QoS, Humble versus REP-2016 identity, and why `zenoh-bridge-ros2dds` is not a substitute. Include troubleshooting for empty graphs, missing hashes, router absence, and transport/RMW mismatch.

- [ ] **Step 3: Run focused validation**

```bash
cargo fmt --all -- --check
cargo clippy -p dora-ros2-bridge -p dora-ros2-bridge-node --all-features -- -D warnings
cargo test -p dora-ros2-bridge-msg-gen
cargo test -p dora-ros2-bridge --all-features
cargo test -p dora-ros2-bridge-python
cargo test -p dora-ros2-bridge-node
scripts/ros2dev.sh verify
scripts/ros2-zenoh-interop.sh humble all
scripts/ros2-zenoh-interop.sh kilted all
```

- [ ] **Step 4: Run repository pre-push gates**

```bash
cargo fmt --all -- --check
cargo clippy --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  -- -D warnings
cargo test --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  --exclude dora-cli-api-python \
  --exclude dora-examples
cargo check --examples
```

Expected: all commands pass. Any environment-dependent skip is listed explicitly and prevents declaring the corresponding compatibility profile complete.

- [ ] **Step 5: Audit the acceptance matrix**

Create the PR validation table with one evidence link/log per design acceptance criterion: four surfaces, two profiles, graph, both topic directions, both service roles, both action roles, QoS, domain, namespace, malformed traffic, shutdown, and DDS regression.

- [ ] **Step 6: Commit documentation**

```bash
git add examples/ros2-bridge/README.md guide docs/testing-guide.md README.md
git commit -m "docs(ros2-bridge): document rmw zenoh transport"
```

## Recommended PR decomposition

1. Tasks 1-4: configuration, abstraction, identities, and golden protocol codecs.
2. Task 5: context and graph discovery.
3. Task 6: topics and QoS.
4. Task 7: services.
5. Task 8: actions and node-feature composition.
6. Tasks 9-11: YAML, Python, Rust, and C++ surfaces.
7. Tasks 12-13: full matrix, CI promotion, and documentation.

Each PR must add its real-peer failing test before claiming its endpoint class, keep all prior matrix entries green, and preserve DDS defaults.

## Resume checkpoint (2026-07-20)

- Humble interoperability matrix: all 10 cases pass.
- Kilted interoperability matrix: all 10 cases pass.
- `scripts/ros2dev.sh verify`: passes all four phases, including Rust, C++, and Python topic/service examples.
- Focused validation passes: formatting, 77 message-generator tests, 46 bridge unit/integration tests, 3 bridge-node transport tests, Python bridge tests, strict bridge/bridge-node/message-generator clippy, C++ ROS-enabled check, and `cargo check --examples`.
- Full-workspace clippy passes through `make qa-fast`; the unwrap/expect count is 161 against a budget of 163.
- `make qa-fast` cannot run audit or typo checks because `cargo-audit` and `typos` are not installed on this host.
- The workspace test baseline is blocked by a reproducible OS archive-write failure (`Bad address`, errno 14) while producing the feature-unified `dora-operator-api-c` static archive. A clean, serial targeted rebuild of `dora-operator-api-c` and `dora-node-api-c` passes, but the full workspace command still fails at archive creation.
- Remaining sequence: resolve or bypass the host archive-write failure, rerun the workspace test baseline, run the Class C fault-tolerance and contract gates, then prepare the acceptance-evidence table and scoped commits.
