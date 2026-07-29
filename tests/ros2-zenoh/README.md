# Native rmw_zenoh interoperability

Run `scripts/ros2-zenoh-interop.sh humble all` or replace `humble` with
`kilted`. The harness uses digest-pinned ROS images and version-pinned
`rmw_zenoh_cpp`, emits `READY`/`PASS` records, applies bounded timeouts, and
always removes its Compose project. These jobs are intentionally nightly until
the compatibility matrix has stabilized.

The matrix covers both topic directions, both service roles, both action roles,
graph discovery, domain isolation, namespaces, and transient-local QoS.
