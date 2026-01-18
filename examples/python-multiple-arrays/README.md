# Fast Multi-Array Messaging with Dora

Hi there! This example shows you how to send multiple numpy arrays (like several sensor readings or images) in a single Dora message with lightning-fast performance.

## Why this matters
You might have noticed that using `numpy_array.tolist()` to package data is... well, pretty slow. That's because it converts every single pixel or number into a Python object, which is heavy work for the CPU.

## The Secret Sauce
The trick is to keep things in binary format! We use `numpy_array.ravel()` to flatten the arrays efficiently and pass them straight to `pyarrow`. This lets us achieve near zero-copy performance.

On the receiving end, we simply convert back to numpy and reshape. Easy peasy!

## Give it a spin

1.  **Get Set Up**: Make sure you have `dora` installed and the binary in your PATH.
2.  **Install the goods**:
    ```bash
    pip install numpy pyarrow
    ```
3.  **Run it**:
    ```bash
    dora up
    dora start dataflow.yml
    ```

You should see something awesome like this in your terminal:
```
Sent message with 3 arrays. Encoding time: 0.000345s
Received and decoded. Shape1: (480, 640, 3), Shape2: (480, 640, 3), State: (1, 6). Time: 0.000210s
```