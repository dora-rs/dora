# Dora C++ Dataflow Example

This example shows how to create dora operators and custom nodes with C++.

Dora does not provide a C++ API yet, but we can create adapters for either the C or Rust API. The `operator-rust-api` and `node-rust-api` folders implement an example operator and node based on dora's Rust API, using the `cxx` crate for bridging. The `operator-c-api` and `node-c-api` show how to create operators and nodes based on dora's C API. Both approaches work, so you can choose the API that fits your application better.
