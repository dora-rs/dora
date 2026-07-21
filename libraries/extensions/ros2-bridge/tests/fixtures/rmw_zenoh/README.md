# rmw_zenoh protocol fixtures

Protocol layouts were checked against `ros2/rmw_zenoh` commits `16520b7cdbf9543128fe57369948cfd404b0750a` (Humble) and `4a1e3b1385826cfca3b2e4b9765a54df7f1f10d5` (Rolling/Kilted-compatible sources): data keys from `rmw_node_data.cpp`, liveliness tokens and QoS components from `liveliness_utils.cpp`, and attachment field order from `attachment_helpers.cpp`. The integration test encodes the pinned representative values directly so failures show the complete expected protocol strings.
