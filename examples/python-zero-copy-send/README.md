# Python Zero-Copy Send Example

This example demonstrates the zero-copy send functionality in the Dora Python Node API.

## Overview

The zero-copy send API allows you to efficiently send large amounts of data without unnecessary copying. Instead of creating your data in Python and then copying it into Dora's send buffer, you can:

1. Allocate a buffer through Dora
2. Write your data directly into that buffer
3. Send it with minimal overhead

## Implementation Details

### Rust Side

The implementation consists of two main components:

1. **`SampleHandler`** (`apis/python/node/src/sample_handler.rs`):
   - Manages the allocated `DataSample`
   - Implements Python's context manager protocol (`__enter__` and `__exit__`)
   - Provides `as_memoryview()` method to expose the buffer as a writable memoryview
   - Automatically sends the data when the context manager exits

2. **`Node.send_output_raw()`** (`apis/python/node/src/lib.rs`):
   - Allocates a `DataSample` of the specified size
   - Returns a `SampleHandler` that can be used with `with` syntax
   - Validates the output ID before allocation

### Python Side

The API is designed to work naturally with Python's context manager syntax:

```python
with node.send_output_raw("output_id", data_length) as buffer:
    # buffer is a writable memoryview - work with it directly
    buffer[:] = your_data
# Data is automatically sent here
```

The context manager returns a writable memoryview that wraps the underlying buffer. You can:

- Use it directly with Python's buffer protocol
- Access and modify bytes using standard indexing: `buffer[i] = value`
- Use slicing: `buffer[:] = data`
- Convert to NumPy if needed with `np.asarray(buffer)` for zero-copy access

## Usage

```python
from dora import Node

node = Node()

# Allocate a 1MB buffer
with node.send_output_raw("large_data", 1024 * 1024) as buffer:
    # buffer is a writable memoryview - fill with your data directly
    for i in range(len(buffer)):
        buffer[i] = i % 256

# Data is automatically sent when exiting the with block
```

### Using with NumPy (optional)

If you want to use NumPy, you can easily convert the memoryview:

```python
import numpy as np

with node.send_output_raw("large_data", 1024 * 1024) as buffer:
    # Convert to numpy for convenient operations (zero-copy view)
    np_array = np.asarray(buffer)

    # Fill with your data
    np_array[:] = np.random.randint(0, 256, size=1024*1024, dtype=np.uint8)
```

## Benefits

1. **Memory Efficiency**: No unnecessary copies - write directly to the send buffer
2. **Performance**: Especially beneficial for large data (> 4KB uses shared memory)
3. **Pythonic API**: Uses familiar `with` statement syntax and standard buffer protocol
4. **No Dependencies**: Works with Python's built-in memoryview, no external libraries required
5. **Flexible**: Compatible with NumPy and any library that supports the buffer protocol

## When to Use

- Sending large arrays or images (> 4KB)
- High-frequency data streaming where performance matters
- When you're generating data that can be written directly to a buffer

## Alternative: Manual Send

If you don't want to use the context manager, you can manually call `send()`:

```python
sample = node.send_output_raw("output_id", data_length)
buffer = sample.as_memoryview()
# ... fill the buffer ...
sample.send()  # Manually send
```

## Implementation Notes

- The buffer is allocated using Dora's `allocate_data_sample()` which automatically uses shared memory for large buffers (>= 4KB)
- The memoryview wraps the underlying memory without copying
- The `SampleHandler` maintains ownership of the buffer until `send()` is called
- After sending, the sample cannot be reused (calling `as_memoryview()` or `send()` again will raise an error)
