#!/usr/bin/env bash
# Builds the generated C++ ROS 2 bindings (dora-node-api-cxx) against a small
# fixture package, then runs a C++ service client/server round trip over
# native Zenoh for both identity profiles. CI has no ROS install, so without
# this the generated cxx code is never compiled. Needs clang++, not ROS.
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=cargo-target-dir.sh
source "$ROOT/scripts/cargo-target-dir.sh"
TARGET_DIR="$(cargo_target_dir "$ROOT")"
FIXTURE="$ROOT/tests/ros2-cxx-zenoh"

export AMENT_PREFIX_PATH="$FIXTURE/ament"
cargo build --manifest-path "$ROOT/Cargo.toml" -p dora-node-api-cxx --features ros2-bridge

BRIDGE="$TARGET_DIR/cxxbridge/dora-node-api-cxx/install"
case "$(uname -s)" in
  Darwin)
    SYSTEM_LIBS=(-framework CoreServices -framework Security -framework SystemConfiguration
      -framework Foundation -framework CoreFoundation -framework IOKit -framework CoreWLAN
      -l objc -l resolv -l pthread -l c -l m)
    ;;
  *)
    SYSTEM_LIBS=(-l m -l rt -l dl -l z -pthread)
    ;;
esac

OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
clang++ -std=c++17 \
  "$FIXTURE/roundtrip.cc" \
  "$BRIDGE/dora-node-api.cc" \
  "$BRIDGE/ros2-bridge/msg/example_interfaces.cc" \
  "$BRIDGE/ros2-bridge/impl.cc" \
  -I "$BRIDGE" -L "$TARGET_DIR/debug" -l dora_node_api_cxx "${SYSTEM_LIBS[@]}" \
  -o "$OUT/roundtrip"

for profile in humble rep2016; do
  if command -v timeout >/dev/null; then
    timeout -k 10s 120s "$OUT/roundtrip" "$profile"
  else
    "$OUT/roundtrip" "$profile"
  fi
done
