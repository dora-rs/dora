# Middleware (communication) layer abstraction (MLA)

`dora` needs to implement MLA as a separate crate to provides a middleware abstraction layer that enables scalable, high performance communications for inter async tasks, intra-process (OS threads), interprocess communication on a single computer node or between different nodes in a computer network. MLA needs to support different communication patterns:
- publish-subscribe push / push pattern - the published message is pushed to subscribers
- publish-subscribe push / pull pattern - the published message is write to storage and later pulled by subscribers
- Request / reply pattern
- Point-to-point pattern
- Client / Server pattern

The MLA needs to abstract following details: 

- inter-async tasks (e.g., tokio channels), intraprocess (OS threads, e.g., shared memory), interprocess and inter-host / inter-network communication
- different transport layer implementations (shared memory, UDP, TCP)
- builtin support for multiple serialization / deserialization protocols, e.g, capnproto, protobuf, flatbuffers etc
- different language bindings to Rust, Python, C, C++ etc
- telemetry tools for logs, metrics, distributed tracing, live data monitoring (e.g., tap a live data), recording and replay

Rust eco-system has abundant crates to provide underlaying communications, e.g.,:
- tokio / crossbeam provides different types of channels serving different purpose: mpsc, oneshot, broadcast, watch etc
- Tonic provides gRPC services
- Tower provides request/reply service
- Zenoh middleware provides many different pub/sub capabilities

MLA also needs to provide high level APIs:
- publish(topic, value, optional fields):- optional fields may contain senders' identify to help MLA logics to satify above requirements
- subscriber(topic, optional fields)-> future streams
- put(key, value, optional fields)
- get(key, optional fields) -> value
- send(key, msg, optional fields)
- recv(key, optional fields)->value
