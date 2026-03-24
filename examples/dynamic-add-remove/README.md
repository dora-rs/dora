# Dynamic Add/Remove Example

Demonstrates dynamically adding and removing nodes from a running dataflow
using `adora node add` and `adora node remove`.

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
pip install adora-rs pyarrow
adora up
adora start examples/dynamic-add-remove/dataflow.yml --detach --name demo
```

The receiver logs all integers from the sender.

### Step 2: Add a filter node

```bash
adora node add --from-yaml examples/dynamic-add-remove/filter-node.yml --dataflow demo
```

The filter node starts and begins receiving from `sender/value`.
Even numbers pass through to `filter/output`.

### Step 3: Wire filter output to receiver

```bash
# Connect filter's output to receiver's new input
adora node connect demo filter/output receiver/filtered
```

Now the receiver sees both:
- `value` (all integers, from sender directly)
- `filtered` (even only, from filter)

### Step 4: Remove the filter

```bash
adora node remove demo filter
```

The filter node stops. The receiver continues receiving from the sender
on the original `value` input.

### Step 5: Clean up

```bash
adora stop --all
adora down
```

## What This Demonstrates

| Feature | Command |
|---------|---------|
| Dynamic node addition | `adora node add --from-yaml` |
| Dynamic node removal | `adora node remove` |
| Live topology wiring | `adora node connect` |
| No dataflow restart needed | All changes are live |
| Backward compatible | Existing nodes unaffected |
