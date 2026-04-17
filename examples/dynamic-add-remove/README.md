# Dynamic Add/Remove Example

Demonstrates dynamically adding and removing nodes from a running dataflow
using `dora node add` and `dora node remove`.

## Architecture

**Initial topology:**
```
Timer (5 Hz) --> Sender --> value --> Receiver
```

**After adding filter node:**
```
Timer (5 Hz) --> Sender --> value --> Filter --> output --> Receiver
                                      (even only)
```

## Nodes

**sender** (`sender.py`) — Emits incrementing integers on each tick.

**receiver** (`receiver.py`) — Logs every value received on any input.

**filter** (`filter.py`) — Dynamically added. Passes through only even numbers.

## Run

### Step 1: Start the base dataflow

```bash
pip install dora-rs pyarrow
dora up
dora start examples/dynamic-add-remove/dataflow.yml --detach --name demo
```

The receiver logs all integers from the sender.

### Step 2: Add a filter node

```bash
dora node add --from-yaml examples/dynamic-add-remove/filter-node.yml --dataflow demo
```

The filter node is registered in the dataflow but not yet wired.

### Step 3: Wire the filter into the pipeline

```bash
# Connect sender's output to filter's input
dora node connect --dataflow demo sender/value filter/input

# Connect filter's output to receiver
dora node connect --dataflow demo filter/output receiver/filtered
```

Now the receiver sees both:
- `value` (all integers, from sender directly)
- `filtered` (even only, from filter via the new mapping)

### Step 4: Remove the filter

```bash
dora node remove demo filter
```

The filter node stops. The receiver continues receiving from the sender
on the original `value` input.

### Step 5: Clean up

```bash
dora stop --all
dora down
```

## What This Demonstrates

| Feature | Command |
|---------|---------|
| Dynamic node addition | `dora node add --from-yaml` |
| Dynamic node removal | `dora node remove` |
| Live topology wiring | `dora node connect` |
| No dataflow restart needed | All changes are live |
| Backward compatible | Existing nodes unaffected |
