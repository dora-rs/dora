#!/usr/bin/env bash
# Builds the generated C++ ROS 2 bindings (dora-node-api-cxx) against small
# fixture packages, then runs C++ service and action client/server round trips
# over native Zenoh for both identity profiles. CI has no ROS install, so without
# this the generated cxx code is never compiled. Needs clang++, not ROS.
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=cargo-target-dir.sh
source "$ROOT/scripts/cargo-target-dir.sh"
TARGET_DIR="$(cargo_target_dir "$ROOT")"
FIXTURE="$ROOT/tests/ros2-cxx-zenoh"

OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

# Install the generated headers into a scratch dir rather than the usual
# `target/cxxbridge/dora-node-api-cxx/install`, so a developer's own C++ build
# pointing there isn't left with the fixture's bindings. (The static library
# is rebuilt for the fixture too; the next build with the real
# AMENT_PREFIX_PATH regenerates it, as build.rs reruns when that changes.)
export AMENT_PREFIX_PATH="$FIXTURE/ament"
export DORA_NODE_API_CXX_INSTALL="$OUT/install"
cargo build --manifest-path "$ROOT/Cargo.toml" -p dora-node-api-cxx --features ros2-bridge

BRIDGE="$DORA_NODE_API_CXX_INSTALL"
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

clang++ -std=c++17 \
  "$FIXTURE/roundtrip.cc" \
  "$BRIDGE/dora-node-api.cc" \
  "$BRIDGE"/ros2-bridge/msg/*.cc \
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
