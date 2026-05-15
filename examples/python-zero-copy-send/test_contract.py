#!/usr/bin/env python3
"""Buffer-protocol contract tests for `node.send_output_raw`.

Exercises the failure modes the API promises to enforce — the safety
contract that backs the zero-copy guarantee. Run inside a dora dataflow
context (so we have a real `Node`); reports per-case pass/fail to stdout
and exits non-zero on any failure.

This complements the Rust unit tests in `apis/python/node/src/sample_handler.rs`,
which cover the atomic state machine. The cases here cover the Python-level
API contract that depends on the state machine being wired correctly:

- send() while a buffer view is still open must error
- view-after-send must raise BufferError
- double-send must error
- as_buffer / as_memoryview must be idempotent (one cached instance per handler)
- context-manager auto-send works when no views leak

Requires Python >= 3.11 (matching the send_output_raw availability gate).
"""

import sys
import traceback

import numpy as np
from dora import Node


# Output IDs declared in contract_dataflow.yml.
OUT = "test_data"


def case_happy_path_context_manager(node):
    """The recommended path: `with ... as buf:` writes + auto-sends."""
    with node.send_output_raw(OUT, 64) as buf:
        np.asarray(buf, dtype=np.uint8)[:] = np.arange(64, dtype=np.uint8)
    # If we got here, the context manager completed cleanly.


def case_happy_path_manual(node):
    """The manual path: explicit `as_memoryview` + `release` + `send`."""
    sample = node.send_output_raw(OUT, 64)
    mv = sample.as_memoryview()
    arr = np.asarray(mv, dtype=np.uint8)
    arr[:] = np.arange(64, dtype=np.uint8)
    del arr
    mv.release()
    sample.send()


def case_send_while_view_open_errors(node):
    """`send()` must refuse while a memoryview is still checked out."""
    sample = node.send_output_raw(OUT, 64)
    mv = sample.as_memoryview()
    try:
        sample.send()
    except RuntimeError as exc:
        msg = str(exc)
        assert "still open" in msg or "view" in msg, (
            f"expected 'still open' / 'view' in error, got: {msg!r}"
        )
    else:
        raise AssertionError("send() should have refused while view was open")
    # Clean up so the handler is in a sendable state — verify rollback worked.
    mv.release()
    sample.send()


def case_view_after_send_raises_buffer_error(node):
    """After `send()`, any new view via the cached SampleBuffer must BufferError."""
    sample = node.send_output_raw(OUT, 64)
    cached_buf = sample.as_buffer()  # grab the underlying SampleBuffer
    sample.send()
    # cached_buf's state.valid is now False; memoryview() must fail.
    try:
        memoryview(cached_buf)
    except BufferError:
        pass  # expected
    else:
        raise AssertionError(
            "memoryview() on a sent-then-cached SampleBuffer should raise BufferError"
        )


def case_double_send_errors(node):
    """`send()` twice on the same handler must error."""
    sample = node.send_output_raw(OUT, 64)
    sample.send()
    try:
        sample.send()
    except Exception as exc:
        msg = str(exc)
        assert "already" in msg or "sent" in msg, (
            f"expected 'already'/'sent' in error, got: {msg!r}"
        )
    else:
        raise AssertionError("second send() should have failed")


def case_as_buffer_is_idempotent(node):
    """Repeated `as_buffer()` returns the same Python object (cached)."""
    sample = node.send_output_raw(OUT, 64)
    buf_a = sample.as_buffer()
    buf_b = sample.as_buffer()
    assert buf_a is buf_b, "as_buffer() must return the cached SampleBuffer"
    # Cleanup: send so we don't leak the handler.
    sample.send()


def case_as_memoryview_is_idempotent(node):
    """Repeated `as_memoryview()` returns the same memoryview; view_count stays at 1."""
    sample = node.send_output_raw(OUT, 64)
    mv_a = sample.as_memoryview()
    mv_b = sample.as_memoryview()
    assert mv_a is mv_b, "as_memoryview() must return the cached PyMemoryView"
    # Single release should drop view_count to 0; send() should then succeed.
    mv_a.release()
    sample.send()


def case_access_after_send_raises_runtime_error(node):
    """`as_buffer()` / `as_memoryview()` after `send()` must error with RuntimeError."""
    sample = node.send_output_raw(OUT, 64)
    sample.send()
    try:
        sample.as_memoryview()
    except RuntimeError:
        pass
    else:
        raise AssertionError("as_memoryview() after send() should raise RuntimeError")


def case_buffer_outlives_dropped_handler_safely(node):
    """Regression: SampleHandler dropped while SampleBuffer is alive must not UAF.

    Previously, the DataSample was owned by SampleHandler.inner. Dropping the
    handler released the DataSample while a held SampleBuffer still pointed
    into it. Acquiring a new view through the buffer would read freed memory.

    Fix: state.valid is flipped on SampleHandler::Drop, so __getbuffer__ sees
    valid=false and raises BufferError. The DataSample is held alive by the
    state Arc (referenced by the SampleBuffer) until the buffer + any
    memoryview chain releases.
    """
    sample = node.send_output_raw(OUT, 64)
    buf = sample.as_buffer()
    del sample  # SampleHandler drops; state.valid flips to false
    try:
        memoryview(buf)
    except BufferError:
        pass  # expected
    else:
        raise AssertionError(
            "memoryview() on a SampleBuffer whose parent handler was dropped "
            "must raise BufferError"
        )


def case_exit_with_exception_does_not_send(node):
    """Regression: `with` body raising must NOT publish the partial buffer.

    Previously, __exit__ ignored exc_type and always called send(). If the
    body raised mid-fill, dora would still ship the half-written buffer
    immediately before propagating the exception — delivering corrupt data
    downstream.

    Fix: __exit__ checks exc_type and skips send() on exceptional exit.

    Test strategy: capture the SampleHandler in a name, drive a `with`-body
    exception through it, then verify the handler can STILL be sent.

    - With the buggy code, __exit__ would have already taken the sample
      via send(); a follow-up sample.send() would raise "Sample has already
      been sent".
    - With the fix, __exit__ skips send() on exceptional exit; the handler
      remains sendable, and our follow-up send() must succeed.

    (Verifying that the original `with` body's exception propagates is
    not load-bearing — Python's `with` semantics propagate it regardless
    of what __exit__ does, so that assertion alone can't distinguish old
    from new behavior.)
    """
    sentinel = RuntimeError("intentional - testing exception path")
    sample = node.send_output_raw(OUT, 64)
    raised = False
    try:
        with sample as buf:
            np.asarray(buf, dtype=np.uint8)[:] = 0
            raise sentinel
    except RuntimeError as exc:
        assert exc is sentinel, (
            f"original exception should propagate unchanged, got: {exc!r}"
        )
        raised = True
    assert raised, "the sentinel exception should have propagated"

    # The load-bearing assertion: the handler is still sendable because
    # __exit__ skipped send() on exceptional exit. With the old buggy
    # code, this would raise "Sample has already been sent".
    sample.send()


CASES = [
    case_happy_path_context_manager,
    case_happy_path_manual,
    case_send_while_view_open_errors,
    case_view_after_send_raises_buffer_error,
    case_double_send_errors,
    case_as_buffer_is_idempotent,
    case_as_memoryview_is_idempotent,
    case_access_after_send_raises_runtime_error,
    case_buffer_outlives_dropped_handler_safely,
    case_exit_with_exception_does_not_send,
]


def main() -> int:
    node = Node()
    failures = 0
    for case in CASES:
        name = case.__name__
        try:
            case(node)
        except Exception:
            failures += 1
            print(f"FAIL  {name}")
            traceback.print_exc()
        else:
            print(f"PASS  {name}")

    total = len(CASES)
    print(f"\n{total - failures}/{total} contract cases passed")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
