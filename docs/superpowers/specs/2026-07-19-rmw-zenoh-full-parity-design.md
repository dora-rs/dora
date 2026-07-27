# Native `rmw_zenoh` Transport for the ROS 2 Bridge

**Issue:** [dora-rs/dora#2735](https://github.com/dora-rs/dora/issues/2735)  
**Status:** Design specification  
**Date:** 2026-07-19

## Summary

Dora will add a native Rust Zenoh transport to its ROS 2 bridge while retaining the existing `ros2-client`/RustDDS transport as the default. The new backend will interoperate directly with `rmw_zenoh_cpp`, including ROS graph discovery, topic publication and subscription, service clients and servers, actions, and the supported QoS subset.

This is not implemented as a switch inside `ros2-client`. Dora will introduce transport-neutral bridge interfaces and two backends:

- `Dds`, adapting the current `ros2-client` behavior without intentional behavior changes.
- `Zenoh`, implementing the public `rmw_zenoh` wire contract with Dora's workspace `zenoh` and `zenoh-ext` dependencies.

Full parity is the end state, delivered in gated increments. Protocol codecs and graph discovery land before data endpoints; topics land before services; actions reuse the completed topic and service layers. Every increment keeps DDS behavior usable and independently testable.

## Goals

1. Let the declarative YAML bridge, Python bridge, generated Rust API, and generated C++ API select DDS or `rmw_zenoh` transport explicitly.
2. Interoperate with real `rmw_zenoh_cpp` peers for:
   - ROS graph visibility;
   - topic publishers and subscriptions;
   - service clients and servers;
   - action clients and servers;
   - parameters and rosout, which are compositions of topics and services.
3. Support the protocol profile used by ROS 2 Humble and the REP-2016 type-hash profile used by current distributions.
4. Preserve DDS as the default and preserve existing configurations and API behavior.
5. Keep Arrow-to-CDR conversion shared between transports.
6. Fail loudly for unsupported profiles, malformed protocol metadata, incompatible QoS, missing type descriptions, or unusable Zenoh configuration.

## Non-goals

- Implementing the ROS RMW C ABI or replacing `rmw_zenoh_cpp`.
- Making `ros2-client` transport-neutral as part of this work.
- Interoperating through `zenoh-bridge-ros2dds`; upstream documents that bridge's key expressions as incompatible with `rmw_zenoh`.
- Supporting an automatic DDS-to-Zenoh relay inside one context.
- Claiming QoS policies that upstream `rmw_zenoh` does not implement.
- Supporting arbitrary future `rmw_zenoh` protocol changes without a new compatibility profile and fixtures.
- Sharing Dora's internal Zenoh session with the ROS bridge. The two protocols have different configuration and lifecycle requirements.

## Evidence and constraints

### Current Dora architecture

The bridge crate reexports `ros2_client` and `rustdds`. Python stores a `ros2_client::Context`, constructs a `ros2_client::Node`, and exposes RustDDS QoS and topic types. Generated Rust and C++ bindings emit the same concrete types. The standalone bridge node also constructs `ros2_client` entities directly. Consequently, adding `zenoh` only to `Cargo.toml` cannot make the bridge transport-selectable.

The Arrow bridge already serializes and deserializes ROS messages through CDR-compatible Serde implementations. This conversion is reusable, but its entry points must be made independent of RustDDS readers and writers.

### Upstream `rmw_zenoh` contract

The upstream design maps one ROS context to one Zenoh session and represents graph entities through Zenoh liveliness tokens. Topic payloads use CDR. Services use queries and queryables. Actions are the standard composition of three services and two topics.

Topic and service data keys have this shape:

```text
<domain-id>/<fully-qualified-name>/<DDS-type-name>/<type-hash>
```

Current profiles use a REP-2016 hash such as `RIHS01_<hex>`. The Humble branch uses the literal `TypeHashNotSupported`. This difference is part of endpoint identity, so the profile must be explicit and testable.

Entity liveliness keys begin with `@ros2_lv` and encode the domain, Zenoh ID, node and entity IDs, entity kind, enclave, namespace, node name, endpoint name, DDS type, type hash, and QoS. Node, publisher, subscription, service, and client entities use `NN`, `MP`, `MS`, `SS`, and `SC` respectively.

The data attachment encodes:

1. signed 64-bit sequence number;
2. signed 64-bit source timestamp in Unix nanoseconds;
3. one-byte GID length;
4. a 16-byte GID.

All fields follow the upstream Zenoh bytes serializer representation. Golden interoperability fixtures, not a hand-written interpretation of prose alone, define the accepted bytes.

### Distribution support

`rmw_zenoh_cpp` became Tier 1 in ROS 2 Kilted. A maintained Humble branch exists and is explicitly in scope because issue #2735 reports Humble. Initial profiles are therefore:

- `humble`: `TypeHashNotSupported` endpoint identity and Humble golden fixtures.
- `rep2016`: type hashes obtained from installed ROS type descriptions or an equivalent verified REP-2016 implementation.

`rep2016` is a protocol family, not an assertion that every future ROS distribution is wire-identical. Integration CI pins a named ROS distribution and `rmw_zenoh` package version.

## User-facing configuration

### Declarative bridge

`Ros2BridgeConfig` gains:

```rust
pub transport: Ros2TransportConfig,
```

with the serialized form:

```yaml
ros2:
  transport:
    kind: zenoh
    compatibility: humble
    config_uri: /etc/zenoh/dora-ros2-session.json5
  topic: /odom
  message_type: nav_msgs/Odometry
  direction: subscribe
```

The default is:

```yaml
transport:
  kind: dds
```

The Rust model is a tagged enum:

```rust
#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RmwZenohCompatibility {
    Humble,
    Rep2016,
}
```

Transport is never inferred from `RMW_IMPLEMENTATION`. That variable describes the RMW loaded by ROS client libraries; Dora's native bridge does not load one. Explicit configuration avoids silent changes and permits DDS and Zenoh bridge nodes in the same dataflow.

`config_uri` has the same meaning as upstream `ZENOH_SESSION_CONFIG_URI`. Resolution order is:

1. explicit `config_uri`;
2. `ZENOH_SESSION_CONFIG_URI`;
3. a checked-in Dora default derived from the pinned upstream profile.

Invalid or unreadable configuration is a startup error. A missing compatible router is reported with a bounded readiness timeout and actionable text; the bridge does not spawn or kill a router.

### Python

Python adds immutable configuration classes:

```python
Ros2Transport.dds()
Ros2Transport.zenoh(compatibility="humble", config_uri=None)
Ros2Context(ros_paths=None, transport=Ros2Transport.dds())
```

Transport belongs to `Ros2Context`, because upstream maps a context to one session. It is not selectable per publisher or subscription. `Ros2NodeOptions` remains node-specific and does not acquire transport state.

### Generated Rust and C++ APIs

Generated context initialization gains an overload/config argument while preserving the existing no-argument DDS initializer:

```rust
init_ros2_context()                         // DDS compatibility entry point
init_ros2_context_with_transport(config)   // explicit backend
```

C++ receives a small generated `Ros2TransportConfig` bridge type rather than exposing Zenoh or RustDDS types. Existing generated code continues to compile unchanged.

## Internal architecture

### Module layout

```text
libraries/extensions/ros2-bridge/src/
  lib.rs
  transport/
    mod.rs
    dds.rs
    types.rs
    zenoh/
      mod.rs
      attachment.rs
      compatibility.rs
      graph.rs
      keyexpr.rs
      pubsub.rs
      qos.rs
      service.rs
```

`transport/types.rs` owns backend-independent names, QoS, message metadata, request IDs, and errors. The DDS adapter translates those into existing RustDDS/`ros2-client` types. The Zenoh backend never exposes `zenoh` types through the public bridge API.

### Transport-neutral interfaces

The interface is capability-oriented rather than one large object-safe trait. Concrete enums avoid async-trait allocation and make shutdown ownership explicit:

```rust
pub enum Ros2Context {
    Dds(dds::Context),
    Zenoh(zenoh::Context),
}

pub enum Ros2Node {
    Dds(dds::Node),
    Zenoh(zenoh::Node),
}

pub enum Publisher<M> {
    Dds(dds::Publisher<M>),
    Zenoh(zenoh::Publisher<M>),
}
```

Equivalent enums cover subscriptions, service clients, and service servers. Actions remain transport-neutral orchestration over these entity interfaces.

The shared types include:

```rust
pub struct Ros2Qos {
    pub reliability: Reliability,
    pub durability: Durability,
    pub history: History,
    pub liveliness: Liveliness,
}

pub struct MessageMetadata {
    pub sequence_number: i64,
    pub source_timestamp_ns: i64,
    pub publisher_gid: [u8; 16],
}

pub struct RequestId {
    pub sequence_number: i64,
    pub client_gid: [u8; 16],
}
```

The current `rustdds` reexports remain available for source compatibility during this feature, but new transport-neutral APIs do not add further RustDDS coupling. Removing legacy reexports is a separate breaking change.

### Type identity

Every endpoint is created from a resolved `RosTypeIdentity`:

```rust
pub struct RosTypeIdentity {
    pub ros_name: String,
    pub dds_name: String,
    pub hash: TypeHash,
}

pub enum TypeHash {
    HumbleUnsupported,
    Rep2016(String),
}
```

For messages, `dds_name` follows ROS introspection naming, for example `std_msgs::msg::dds_::String_`. Services use the service base DDS type after stripping request/response suffixes, matching upstream. Actions resolve identities for their generated service and message endpoints.

For `rep2016`, Dora first loads installed `type_description_interfaces` artifacts generated by ROS. If an installed package does not provide the complete type description needed to establish the official hash, endpoint creation fails with the package, type, profile, and searched paths. Dora does not invent a hash from `.msg` text alone. A future verified pure-Rust REP-2016 generator can replace this loader behind the same interface.

### Zenoh context and graph

One Zenoh context owns:

- one configured session;
- the ROS domain ID;
- a random stable-per-context node/entity ID allocator;
- a 16-byte GID generator;
- a graph cache;
- one liveliness subscriber and initial liveliness query;
- cancellation and shutdown state.

Creating a node declares its `NN` token. Creating an endpoint declares its endpoint token only after the data entity is ready. Drop order is endpoint token, data entity, node token, graph subscriber, session. Partial creation rolls back already-created resources.

The graph cache parses only tokens matching its domain. Malformed remote tokens are logged and ignored with rate limiting. Duplicate puts and deletes are idempotent. It drives:

- service availability;
- action server availability derived from its component endpoints;
- graph-facing API queries;
- QoS compatibility diagnostics.

It does not gate ordinary topic delivery: a subscriber begins receiving as soon as its Zenoh entity is declared.

### Topic data flow

Publication:

```text
Arrow value -> shared CDR serializer -> Zenoh payload
            + sequence/timestamp/GID -> attachment
            + resolved topic identity -> exact key expression
```

Subscription validates the key by construction, validates and parses the attachment, bounds payload size before deserialization, then passes CDR bytes to the shared Arrow deserializer. Invalid samples are reported and skipped without terminating the subscription stream.

QoS mapping follows pinned upstream behavior:

- best effort -> Zenoh best-effort reliability and drop congestion control;
- reliable -> Zenoh reliable reliability;
- reliable + keep-all -> blocking congestion control;
- transient-local publisher -> advanced publisher cache;
- transient-local subscriber -> advanced subscriber history query;
- keep-last depth -> cache/history bound;
- unsupported deadline and liveliness events -> explicit unsupported result.

The implementation records the exact upstream version used for each mapping in module documentation and tests.

### Services

A Zenoh service server declares a complete queryable on the resolved service key. A client declares a querier targeting all complete queryables. Requests carry serialized request CDR and the standard attachment. Responses preserve the request sequence and client GID in their attachment.

The existing Dora-facing request ID remains an opaque string, backed internally by the transport-neutral `RequestId`. Pending request bounds and expiration remain unchanged.

Service readiness is true when the graph contains at least one compatible `SS` entity for the same domain, name, type identity, and compatible QoS. The current bounded retry behavior remains available, but it observes the Zenoh graph cache instead of a DDS spinner.

### Actions and parameters

Actions are constructed from:

- `<action>/_action/send_goal` service;
- `<action>/_action/get_result` service;
- `<action>/_action/cancel_goal` service;
- `<action>/_action/feedback` topic;
- `<action>/_action/status` topic.

The Zenoh backend uses the same transport-neutral action orchestration as DDS. The `get_result` query receives the upstream long-duration timeout policy. Goal limits, feedback backpressure, cancellation, and terminal-state semantics stay at the bridge layer.

Parameters and rosout continue to be node features composed from standard ROS services and topics. They are enabled for Zenoh only after service and topic parity is complete. Until then, requesting them with Zenoh returns a clear capability error rather than creating a partially visible node.

## Error handling and observability

Startup errors include a stable category and context:

- invalid transport configuration;
- unsupported compatibility profile;
- missing type description/hash;
- invalid ROS name or DDS type name;
- Zenoh session open failure;
- router/readiness timeout;
- entity or liveliness declaration failure.

Runtime errors distinguish malformed remote data from local transport failure. Malformed samples and liveliness tokens are skipped with rate-limited warnings. Session closure terminates streams with a transport-closed error. Publisher congestion follows configured QoS and emits counters for dropped samples.

Tracing fields include `ros.transport`, `ros.domain_id`, `ros.node`, `ros.entity_kind`, `ros.name`, and `ros.compatibility`. GIDs and payloads are not logged by default.

## Compatibility and migration

- Existing YAML without `transport` remains DDS.
- Existing `Ros2Context()` remains DDS.
- Existing generated `init_ros2_context()` remains DDS.
- Existing DDS service mapping detection remains inside the DDS adapter.
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp` no longer produces the misleading generic unknown-RMW warning when a Zenoh transport is explicitly configured. With DDS selected it still warns that the variable does not match the active native transport.
- The bridge remains marked unstable, but configuration parsing still follows additive compatibility rules.

## Testing strategy

### Unit and golden tests

Protocol tests require no ROS installation:

- configuration defaulting and validation;
- DDS type-name construction;
- Humble and REP-2016 endpoint keys;
- all entity liveliness keys and parse round trips;
- QoS token encoding and compatibility;
- attachment bytes, boundaries, and malformed inputs;
- graph put/delete/idempotence/domain isolation;
- request correlation and expiry;
- action endpoint expansion;
- missing type-description errors.

Golden values are generated once from pinned upstream `rmw_zenoh_cpp` test helpers or captured interoperable processes and checked into fixtures with provenance. Tests never calculate expected output with the same Dora function under test.

### Rust-only Zenoh integration

Two Dora contexts connected through a test router verify lifecycle, pub/sub, queries, transient-local history, cancellation, shutdown, and malformed-peer isolation. These tests prove internal consistency but do not count as ROS interoperability evidence.

### Real ROS interoperability

Dedicated Linux jobs run pinned containers for Humble and Kilted (or the selected REP-2016 distribution), start `rmw_zenohd`, and set `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. Each matrix entry verifies:

1. Dora publisher to `rclpy` subscriber.
2. `rclpy` publisher to Dora subscriber.
3. Dora service client to `rclpy` server.
4. `rclpy` client to Dora service server.
5. Dora action client to `rclpy` action server, including feedback and cancellation.
6. `rclpy` action client to Dora action server, including rejection and terminal states.
7. `ros2 node list`, `ros2 topic info`, `ros2 service list`, and `ros2 action list` visibility.
8. namespaces and nonzero `ROS_DOMAIN_ID` isolation.
9. best-effort/reliable and volatile/transient-local cases supported upstream.

Existing DDS examples and `scripts/ros2dev.sh verify` remain mandatory regression coverage.

## Delivery sequence and merge gates

1. **Foundation:** configuration, shared types, type identity, protocol codecs, fixtures. Gate: unit/golden tests and unchanged DDS tests.
2. **Graph:** session lifecycle, node/endpoint tokens, graph cache. Gate: Dora-to-Dora graph tests plus real `ros2 node/topic` visibility.
3. **Topics:** both directions and supported QoS. Gate: Humble and REP-2016 interop topic matrix.
4. **Services:** both roles and readiness. Gate: service matrix plus pending-request stress tests.
5. **Actions and node features:** both roles, parameters, rosout. Gate: action matrix and ROS CLI visibility.
6. **All surfaces and documentation:** YAML, Python, generated Rust, generated C++. Gate: source-compatibility compile tests and all ROS2 QA.

No phase may claim compatibility based only on two Dora processes. At least one real `rmw_zenoh_cpp` peer is required for each implemented endpoint class.

## Risks and mitigations

| Risk | Mitigation |
|---|---|
| Upstream private protocol drift | Explicit profiles, pinned fixtures, pinned integration images, no `auto` profile |
| Incorrect REP-2016 identity | Load authoritative installed type descriptions; fail closed when unavailable |
| Zenoh 1.8/1.9 skew for Humble | Exercise the exact Humble package against Dora 1.9 in CI before claiming support |
| Public RustDDS coupling | Add neutral wrappers and keep legacy reexports; defer removal |
| QoS overclaim | Match pinned upstream mappings and return unsupported errors for missing semantics |
| Graph appears correct while payloads fail | Separate graph and data assertions in real-peer tests |
| Payload works while graph is invisible | Make ROS CLI graph assertions a release gate |
| Python wheel growth or feature conflicts | Measure artifacts, share workspace Zenoh version, and test maturin builds |
| Router lifecycle confusion | Never spawn implicitly; bounded readiness diagnostics and documented setup |
| Large review surface | Merge in dependency-ordered phases with independent gates |

## Acceptance criteria

The feature is complete only when:

- all four existing bridge surfaces can select DDS or Zenoh without breaking their DDS defaults;
- Humble and one REP-2016 distribution pass the real-peer topic, service, action, and graph matrix;
- supported QoS cases match the pinned upstream behavior;
- domain and namespace isolation are verified;
- malformed remote traffic cannot crash or unboundedly allocate;
- transport shutdown releases tokens and terminates streams cleanly;
- existing DDS ROS2 QA remains green;
- documentation identifies router requirements, profile selection, supported QoS, and distribution pins.

## Authoritative upstream references

- `rmw_zenoh` repository and interoperability boundary: <https://github.com/ros2/rmw_zenoh>
- `rmw_zenoh` design: <https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md>
- ROS 2 Kilted release notes: <https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html>
- `zenoh-bridge-ros2dds`, which is not an `rmw_zenoh` compatibility layer: <https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds>
