#!/usr/bin/env bash
#
# Deployment smoke for the `dora-slim` image. Runs *inside* the built container
# -- see `.github/workflows/docker-image.yml`. "Verifying the image" in
# README.md has the run-it-by-hand recipe.
#
# `docker build` exiting 0 only says the layers assembled. This asserts the
# thing the image exists for: that the `dora` it ships can host a dataflow --
# spawn two nodes, route an Arrow message from one to the other, and shut them
# both down again.
#
# The nodes are written out below rather than mounting one of the repo's
# `examples/`, which would be less code. The examples are maintained against
# the workspace bindings and gated by workspace tests; this container runs the
# *published* `dora-rs` from PyPI, and those two have drifted before (#1710).
# Mounting an example would couple this gate to that drift, and red a
# `docker/**` PR over something that has nothing to do with the image. What is
# written here uses only API that dora's 1.x guarantee freezes.

set -euo pipefail

cd "$(mktemp -d)"

# Report the pair the image shipped, and then use it exactly as shipped.
# `dora-rs-cli` depends on `dora-rs >= 0.3.9` -- an open range resolved at build
# time, so the CLI and the Python bindings can come from different releases.
# Deliberately not pinned or reinstalled here: what a user gets is the whole
# subject of a deployment smoke, and the 1.x guarantee says the pair has to
# interoperate. If it does not, the run below fails, and that is the finding.
dora --version
python - <<'PY'
import dora
import pyarrow

print(f"smoke: dora-rs {getattr(dora, '__version__', 'unknown')}, pyarrow {pyarrow.__version__}")
PY

cat > source.py <<'PY'
"""Timer-driven source: one Arrow array per tick."""

import pyarrow as pa
from dora import Node

node = Node()
sent = 0
for event in node:
    if event["type"] != "INPUT":
        continue
    node.send_output("value", pa.array([sent]))
    sent += 1
PY

cat > sink.py <<'PY'
"""Sink: prints the marker smoke.sh greps for, once routing is proven."""

from dora import Node

WANTED = 3

node = Node()
seen = 0
for event in node:
    if event["type"] != "INPUT":
        continue
    # Read the payload back, don't just count events: a value that survives the
    # daemon as an Arrow array is what proves the data path, not the wake-up.
    value = event["value"].to_pylist()
    assert len(value) == 1 and isinstance(value[0], int), f"unexpected payload {value!r}"
    seen += 1
    if seen == WANTED:
        # Keep this literal in sync with the grep at the end of smoke.sh.
        print("DORA_SLIM_SMOKE_OK", flush=True)

print(f"smoke: sink saw {seen} message(s)", flush=True)
PY

cat > dataflow.yml <<'YAML'
nodes:
  - id: source
    path: source.py
    inputs:
      tick: dora/timer/millis/100
    outputs:
      - value

  - id: sink
    path: sink.py
    inputs:
      value: source/value
YAML

# No `dora build` first: `run.rs` calls `build_dataflow` itself before starting
# the daemon, and neither node has a `build:` key anyway.
dora run dataflow.yml --stop-after 10s 2>&1 | tee run.log

# `dora run --stop-after` exits 0 whenever the daemon survived the window,
# whether or not a node did its job -- on Unix a node killed at teardown reports
# a clean Signal(15). So the verdict has to come from the node's own output.
# `assert_clean_dataflow_run` in scripts/qa/ci-nightly-jobs.sh is the
# workspace-side form of the same assertion (#1863); keep them in step.
if ! grep -q DORA_SLIM_SMOKE_OK run.log; then
    echo "smoke: sink never reported receiving messages -- see the log above" >&2
    exit 1
fi

echo "smoke: dataflow ran inside the image"
