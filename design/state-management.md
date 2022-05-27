# State Management

Most operations require to keep some sort of state between calls. This document describes the different ways to handle state in dora.

## Internal State

Operators are `struct` or object instances, so they can keep internal state between invocations. This state is private to the operator. When an operator exits or crashes, its internal state is lost.

## Saving State

To make themselves resilient against crashes, operators can use dora's state management. The dora runtime provides each operator with a private key-value store (KVS). Operators can save serialized state into the KVS by using the `save_state` function of the runtime:

```rust
fn save_state(key: &str, value: Vec<u8>)
```

The runtime only stores the latest value for each key, so subsequent writes to the same key replace the earlier values. Serialization is required because the state must be self-contained (i.e. no pointers to other memory) and consistent (i.e. no half-updated state). Otherwise, state recovery might not be possible after an operator crash.

To keep the performance overhead of this function low, it is recommended to use a suitable serialization format that stores the data with minimal memory and compute overhead. Text-based formats such as JSON are not recommended. Also, fast-changing state should be stored under a separate key to minimize the amount of state that needs to be written.

### State Recovery

When an operator crashes, the dora runtime restarts it and supplies it with the last version of the saved state. It does this by calling the operator's `restore_state` method:

```rust
fn restore_state(&mut self, state: HashMap<String, Vec<u8>>)
```

In this method, the operator should deserialize and apply all state entries, and perform all custom consistency checks that are necessary.

## Sharing State

To share state between operators, dora provides access to a node-local key-value store:

```rust
fn kvs_write(key: &str, value: Vec<u8>)
```
```rust
fn kvs_read(key: &str) -> Vec<u8>
```

Todo:

- Consistency?
- Anna?


## Custom Nodes

Custom nodes have full control over the execution, so they can implement their own state management. Shared state can be accessed through the `kvs_read` and `kvs_write` functions of the dora library, which are equivalent to the respective functions provided by the dora runtime.

Since custom nodes cannot use the recovery feature of the dora runtime, the `save_state`/`restore_state` functions are not available for them.
