//! Zero-copy send via Python's buffer protocol.
//!
//! [`SampleHandler`] is the Python object returned by `node.send_output_raw()`.
//! It owns a pre-allocated [`DataSample`] (shared-memory- or aligned-heap-backed)
//! and lets the caller write directly into the underlying buffer via a writable
//! `memoryview`, then `send()`s the buffer with no additional copies.
//!
//! On Python `>= 3.11` the buffer-protocol slots `bf_getbuffer` / `bf_releasebuffer`
//! are part of the stable C API, so we expose a [`SampleBuffer`] pyclass that
//! implements `__getbuffer__` / `__releasebuffer__` and is composed into a
//! `PyMemoryView` for ergonomic use.
//!
//! Older Python versions only get a stub on [`crate::Node::send_output_raw`] that
//! raises `NotImplementedError` with an actionable message.
//!
//! Safety model
//! ============
//!
//! The [`DataSample`] is owned by an `Arc<`[`SampleBufferState`]`>`, shared by
//! `SampleHandler` and any `SampleBuffer` it produces. This lets the buffer (and
//! any `memoryview` derived from it) keep the underlying allocation alive even
//! if the `SampleHandler` is dropped first — eliminating a use-after-free that
//! would otherwise be reachable via `buf = sample.as_buffer(); del sample;
//! memoryview(buf)`.
//!
//! Three invariants prevent unsafe access:
//!
//! 1. `SampleHandler::send()` flips `valid = false` *before* taking the
//!    `DataSample` out of `state.sample`. Any subsequent `__getbuffer__` call
//!    sees `valid == false` and raises `BufferError`.
//! 2. `send()` refuses to proceed while `view_count > 0`. The decrement happens
//!    in `__releasebuffer__`; the increment happens *after* the view fill block
//!    so a panic during fill cannot leave the counter permanently elevated.
//! 3. `SampleHandler::drop()` flips `valid = false` so an unsent handler's
//!    early drop still rejects new view acquisitions. The `DataSample` stays
//!    in `state.sample` until the last `SampleBuffer`/memoryview also drops.

#[cfg(Py_3_11)]
use pyo3::types::PyMemoryView;
#[cfg(Py_3_11)]
use std::ffi::{c_int, c_void};
use std::ops::Deref;
#[cfg(Py_3_11)]
use std::ptr;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicIsize, Ordering};

use dora_message::metadata::ArrowTypeInfo;
use dora_node_api::dora_core::metadata::ArrowTypeInfoExt;
use dora_node_api::{DataSample, DoraNode, MetadataParameters, dora_core::config::DataId};
use dora_operator_api_python::DelayedCleanup;
#[cfg(Py_3_11)]
use pyo3::exceptions::PyBufferError;
#[cfg(Py_3_11)]
use pyo3::ffi;
use pyo3::types::PyAnyMethods;
use pyo3::{Bound, Py, PyAny, PyResult, Python, pyclass, pymethods};

// ---------------------------------------------------------------------------
// Shared liveness + view-count state + ownership of the DataSample
// ---------------------------------------------------------------------------

/// Shared (via `Arc`) between `SampleHandler` and any `SampleBuffer` it hands out.
///
/// Owns the [`DataSample`]. `send()` takes it; otherwise it lives here until
/// the last `Arc` reference drops — which means a `SampleBuffer` (and any
/// `memoryview` derived from it) keeps the allocation alive even if the
/// `SampleHandler` is dropped before `send()`.
///
/// The `valid` flag plus the `view_count` counter together prevent
/// use-after-free (see the module-level docs for the three invariants).
struct SampleBufferState {
    /// `true` while the underlying `DataSample` is still accessible via the
    /// buffer protocol. Set to `false` by `send()` (after view_count == 0)
    /// and by `SampleHandler::drop()` (regardless of view_count).
    valid: AtomicBool,
    /// Number of `Py_buffer` views currently checked out via `__getbuffer__`
    /// that have not yet been released. Incremented *after* the view is
    /// fully initialised so that a panic during initialisation cannot leave
    /// the counter permanently elevated.
    view_count: AtomicIsize,
    /// Owns the `DataSample`. `send()` takes it; `SampleHandler::drop()`
    /// leaves it. Behind a `Mutex` because the state is shared across the
    /// Python/Rust boundary and Rust requires `Sync` for `Arc<T>`'s `Send`
    /// implementation — PyO3's GIL serialises pymethods, so contention is
    /// impossible in practice.
    sample: Mutex<Option<DataSample>>,
}

impl SampleBufferState {
    fn new(sample: DataSample) -> Arc<Self> {
        Arc::new(Self {
            valid: AtomicBool::new(true),
            view_count: AtomicIsize::new(0),
            sample: Mutex::new(Some(sample)),
        })
    }
}

// ---------------------------------------------------------------------------
// SampleBuffer – Python buffer-protocol object (zero-copy)
// ---------------------------------------------------------------------------

/// A Python object that exposes the pre-allocated output-sample memory via
/// the [Python buffer protocol](https://docs.python.org/3/c-api/buffer.html).
///
/// Available on Python >= 3.11, where `bf_getbuffer` / `bf_releasebuffer`
/// are part of the stable C API.
///
/// Permission model
/// ----------------
/// - **Writable** (`readonly = 0`) as long as `state.valid == true`.
/// - Acquiring a new view *after* `send()` or after `SampleHandler` drop
///   raises `BufferError`.
/// - `send()` refuses to proceed while any view is still checked out,
///   preventing use-after-free.
///
/// Python usage
/// ------------
/// ```python
/// buf = handler.as_buffer()
/// mv  = memoryview(buf)                           # writable memoryview
/// arr = numpy.frombuffer(buf, dtype=numpy.uint8)  # zero-copy numpy array
/// struct.pack_into("4f", buf, 0, *values)         # struct write
/// ```
#[cfg(Py_3_11)]
#[pyclass]
pub struct SampleBuffer {
    /// Raw pointer into the `DataSample` owned by `state.sample`.
    ///
    /// # Safety
    /// Valid iff `state.valid == true`. `SampleBuffer` holds an `Arc` to
    /// `state`, so the `DataSample` (inside `state.sample`) cannot be
    /// dropped while any `SampleBuffer` (and therefore any memoryview
    /// derived from it via `Py_buffer.obj`) is alive.
    ptr: *mut u8,
    len: usize,
    state: Arc<SampleBufferState>,
}

// SAFETY: `SampleBuffer` contains a raw `*mut u8` so it isn't `Send`/`Sync`
// by default, but it is safe to share across threads in our specific usage:
//
// - The only ways to acquire a `&SampleBuffer` are through PyO3's `Bound<'_, T>`
//   / `Py<T>` machinery (`__getbuffer__` takes `Bound<'_, Self>`,
//   `__releasebuffer__` takes `&self`). PyO3 requires the GIL to be held for
//   both, statically via the `Python<'_>` token threaded through every
//   `#[pymethods]` entry point.
// - The data the pointer references lives in a `DataSample` owned by
//   `state.sample` (an `Arc<SampleBufferState>`). The `SampleBuffer` holds
//   that same `Arc`, so the `DataSample` cannot be dropped while the
//   `SampleBuffer` is alive. `send()` won't take the `DataSample` until
//   `state.view_count == 0`, and `__getbuffer__` won't hand out a view once
//   `state.valid == false`. So whenever the pointer is actually dereferenced
//   (by CPython, through the `Py_buffer`), the allocation is guaranteed live.
//
// The `unsafe impl` is required because the raw pointer would otherwise
// disqualify the type from auto-`Send`/`Sync`. Marker traits themselves
// don't enforce the GIL-holding contract above — PyO3's API does.
#[cfg(Py_3_11)]
unsafe impl Send for SampleBuffer {}
#[cfg(Py_3_11)]
unsafe impl Sync for SampleBuffer {}

#[cfg(Py_3_11)]
#[pymethods]
impl SampleBuffer {
    /// Called by Python whenever a buffer view is requested:
    /// `memoryview(buf)`, `numpy.frombuffer(buf)`, `struct.pack_into`, ...
    ///
    /// Raises `BufferError` after `send()` or after the parent `SampleHandler`
    /// has been dropped.
    ///
    /// # Safety
    /// PyO3 requires this to be `unsafe fn`; the raw `*mut Py_buffer`
    /// pointer is provided by the CPython runtime and is guaranteed valid
    /// for the duration of the call.
    unsafe fn __getbuffer__(
        slf: Bound<'_, Self>,
        view: *mut ffi::Py_buffer,
        flags: c_int,
    ) -> PyResult<()> {
        let this = slf.borrow();

        if !this.state.valid.load(Ordering::Acquire) {
            return Err(PyBufferError::new_err(
                "Cannot acquire buffer view: the parent SampleHandler has been \
                 sent or dropped",
            ));
        }

        if view.is_null() {
            return Err(PyBufferError::new_err("View pointer is null"));
        }

        // SAFETY: `view` is a valid, non-null `Py_buffer *` provided by
        // CPython. We own the pointed-to memory for the duration of this
        // call and the returned view.
        unsafe {
            // A new strong reference keeps `slf` alive for the view's lifetime.
            (*view).obj = slf.into_any().into_ptr();

            (*view).buf = this.ptr as *mut c_void;
            (*view).len = this.len as ffi::Py_ssize_t;
            (*view).readonly = 0; // writable
            (*view).itemsize = 1;

            // Use a `'static` C-string literal for the format so we don't
            // heap-allocate (and don't need to reclaim) per __getbuffer__ call.
            // `"B"` is the buffer-protocol format code for unsigned bytes.
            (*view).format = if (flags & ffi::PyBUF_FORMAT) == ffi::PyBUF_FORMAT {
                c"B".as_ptr().cast_mut()
            } else {
                ptr::null_mut()
            };

            (*view).ndim = 1;

            (*view).shape = if (flags & ffi::PyBUF_ND) == ffi::PyBUF_ND {
                &raw mut (*view).len
            } else {
                ptr::null_mut()
            };

            (*view).strides = if (flags & ffi::PyBUF_STRIDES) == ffi::PyBUF_STRIDES {
                &raw mut (*view).itemsize
            } else {
                ptr::null_mut()
            };

            (*view).suboffsets = ptr::null_mut();
            (*view).internal = ptr::null_mut();
        }

        // Increment *after* filling the view so that a panic in the fill block
        // above cannot leave `view_count` permanently incremented without a
        // matching `__releasebuffer__` call, which would permanently block `send()`.
        this.state.view_count.fetch_add(1, Ordering::AcqRel);

        Ok(())
    }

    /// Called by Python when the consumer is done with the buffer view.
    ///
    /// Must not fail — any error would be silently sent to
    /// `sys.unraisablehook`.
    ///
    /// # Safety
    /// Same contract as `__getbuffer__`: `view` is a valid `Py_buffer *`
    /// for the duration of the call.
    unsafe fn __releasebuffer__(&self, _view: *mut ffi::Py_buffer) {
        // `__getbuffer__` points `(*view).format` at a `'static` C-string,
        // so there is nothing to free here. Just decrement the view counter
        // that was incremented after the matching __getbuffer__ completed.
        self.state.view_count.fetch_sub(1, Ordering::AcqRel);
    }
}

// ---------------------------------------------------------------------------
// SampleMeta
// ---------------------------------------------------------------------------

/// Per-send metadata. Lives in `SampleHandler.meta` until `send()` takes it.
struct SampleMeta {
    data_id: DataId,
    type_info: ArrowTypeInfo,
    parameters: MetadataParameters,
}

// ---------------------------------------------------------------------------
// SampleHandler
// ---------------------------------------------------------------------------

/// Returned by `Node.send_output_raw`.
///
/// Holds the per-send metadata plus an `Arc<SampleBufferState>` that owns the
/// pre-allocated [`DataSample`]. On Python >= 3.11 the context-manager and
/// `as_buffer()` API are available; both let Python write into the sample
/// via the buffer protocol and ship it with no additional copies.
///
/// ```python
/// with node.send_output_raw("out", 1024 * 1024) as buf:
///     memoryview(buf)[:] = my_bytes   # or: numpy / struct / ...
/// # buf is sent automatically on __exit__ (only if the body did NOT raise)
/// ```
///
/// Manual usage (Python >= 3.11):
///
/// ```python
/// handler = node.send_output_raw("out", length)
/// buf = handler.as_buffer()
/// memoryview(buf)[:] = my_bytes
/// handler.send()
/// ```
#[pyclass]
pub struct SampleHandler {
    node: DelayedCleanup<DoraNode>,
    /// Per-send metadata. Taken by `send()`.
    meta: Option<SampleMeta>,
    /// Shared state that owns the `DataSample`. Survives `SampleHandler::drop()`
    /// iff any `SampleBuffer` (and thus any memoryview chain) keeps it alive.
    state: Arc<SampleBufferState>,
    /// Lazily created on the first call to `as_buffer()` / `__enter__`.
    /// Only present on Python >= 3.11 where the buffer protocol is stable.
    #[cfg(Py_3_11)]
    buffer: Option<Py<SampleBuffer>>,
    /// The `memoryview` created by `__enter__` / `as_memoryview()`.
    /// Cached so that `__exit__` can close it (releasing its buffer view)
    /// before `send()` checks that `view_count == 0`.
    #[cfg(Py_3_11)]
    memoryview: Option<Py<PyMemoryView>>,
}

impl SampleHandler {
    pub fn new(
        data_length: usize,
        data_id: DataId,
        parameters: MetadataParameters,
        node: DelayedCleanup<DoraNode>,
    ) -> eyre::Result<Self> {
        let sample = node.get_mut().allocate_data_sample(data_length)?;
        let type_info = ArrowTypeInfo::byte_array(data_length);
        Ok(Self {
            node,
            meta: Some(SampleMeta {
                data_id,
                type_info,
                parameters,
            }),
            state: SampleBufferState::new(sample),
            #[cfg(Py_3_11)]
            buffer: None,
            #[cfg(Py_3_11)]
            memoryview: None,
        })
    }
}

impl Drop for SampleHandler {
    fn drop(&mut self) {
        // Invalidate the state so any new `__getbuffer__` call fails with
        // `BufferError`. Existing memoryviews keep working — their cached
        // `Py_buffer.buf` pointer references the `DataSample` which is held
        // alive by the `state` Arc (via `SampleBuffer.state` and the
        // memoryview's strong ref on `SampleBuffer`). Once those views are
        // released, the last Arc drops, `state.sample` drops, and the
        // `DataSample` is reclaimed.
        self.state.valid.store(false, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// Python methods available on all supported Python versions
// ---------------------------------------------------------------------------

#[pymethods]
impl SampleHandler {
    /// Manually send the sample (alternative to the context manager).
    ///
    /// Returns an error if any buffer views are still open on the associated
    /// `SampleBuffer` — release all `memoryview` / numpy references first.
    pub fn send(&mut self) -> eyre::Result<()> {
        // 1. Guard: can only send once.
        let meta = self
            .meta
            .take()
            .ok_or_else(|| eyre::eyre!("Sample has already been sent"))?;

        // 2. Atomically (under the GIL + the state mutex): check view_count,
        //    invalidate the buffer, take the DataSample. The mutex is held for
        //    the whole sequence so the read-then-write is a single critical
        //    section.
        let sample = {
            let mut sample_lock = self
                .state
                .sample
                .lock()
                .map_err(|_| eyre::eyre!("DataSample lock poisoned"))?;

            #[cfg(Py_3_11)]
            {
                let view_count = self.state.view_count.load(Ordering::Acquire);
                if view_count != 0 {
                    // Roll back the meta take so the handler stays sendable.
                    self.meta = Some(meta);
                    eyre::bail!(
                        "Cannot send: {view_count} buffer view(s) are still open. \
                         Release all memoryview / numpy references before calling send()."
                    );
                }
            }

            // Invalidate the SampleBuffer *before* the DataSample is freed so
            // any Python handle that later calls __getbuffer__ receives a
            // clear BufferError instead of accessing freed memory.
            self.state.valid.store(false, Ordering::Release);
            sample_lock.take()
        };

        let sample = sample.ok_or_else(|| eyre::eyre!("DataSample missing (concurrent send?)"))?;

        // 3. Send (zero additional copies).
        self.node
            .get_mut()
            .send_output_sample(meta.data_id, meta.type_info, meta.parameters, Some(sample))
            .map_err(|e| eyre::eyre!("failed to send sample: {e}"))?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Python buffer-protocol methods – Python >= 3.11 only
// ---------------------------------------------------------------------------

#[cfg(Py_3_11)]
#[pymethods]
impl SampleHandler {
    /// Return a `SampleBuffer` that exposes the pre-allocated memory via
    /// Python's buffer protocol (writable, zero-copy).
    ///
    /// The same `SampleBuffer` instance is returned on every call (lazily
    /// created on first access).
    ///
    /// Raises `RuntimeError` after `send()` has been called or after the
    /// handler was dropped.
    pub fn as_buffer(&mut self, py: Python<'_>) -> PyResult<Py<SampleBuffer>> {
        if !self.state.valid.load(Ordering::Acquire) {
            return Err(pyo3::exceptions::PyRuntimeError::new_err(
                "Cannot access buffer: the sample has already been sent",
            ));
        }

        // Return the cached instance if we already created one.
        if let Some(buf) = &self.buffer {
            return Ok(buf.clone_ref(py));
        }

        // Extract a stable raw pointer to the DataSample's bytes. The
        // `DataSample` is heap-allocated via `AVec`, so its buffer address
        // does not move; the pointer is valid as long as the `DataSample`
        // remains inside `state.sample`. `send()` is the only path that
        // takes the sample out, and it requires `view_count == 0` first —
        // so any view we hand out via this `ptr` will be released before
        // the memory can be reclaimed.
        let (ptr, len) = {
            let sample_lock = self.state.sample.lock().map_err(|_| {
                pyo3::exceptions::PyRuntimeError::new_err("DataSample lock poisoned")
            })?;
            let sample = sample_lock.as_ref().ok_or_else(|| {
                pyo3::exceptions::PyRuntimeError::new_err(
                    "DataSample missing in state (concurrent send?)",
                )
            })?;
            let slice: &[u8] = sample.deref();
            (slice.as_ptr() as *mut u8, slice.len())
        };

        let py_buf = Py::new(
            py,
            SampleBuffer {
                ptr,
                len,
                state: self.state.clone(),
            },
        )?;
        self.buffer = Some(py_buf.clone_ref(py));
        Ok(py_buf)
    }

    /// Return a writable Python `memoryview` over the pre-allocated memory
    /// (zero-copy).
    ///
    /// Wraps the `SampleBuffer` in a native Python `memoryview` so callers can
    /// use it directly with slice syntax, `struct.pack_into`, NumPy, etc.
    ///
    /// Raises `RuntimeError` after `send()` has been called or after the
    /// handler was dropped.
    ///
    /// ```python
    /// sample = node.send_output_raw("out", 1024)
    /// mv = sample.as_memoryview()
    /// np.asarray(mv, dtype=np.uint8)[:] = data
    /// sample.send()
    /// ```
    pub fn as_memoryview(&mut self, py: Python<'_>) -> PyResult<Py<PyMemoryView>> {
        // Return the cached memoryview if one already exists. This keeps
        // view_count at 1 regardless of how many times the caller invokes
        // as_memoryview(), so send() is never blocked by phantom open views.
        if let Some(mv) = &self.memoryview {
            return Ok(mv.clone_ref(py));
        }

        let buf = self.as_buffer(py)?;
        let bound_buf = buf.into_bound(py);
        let mv = PyMemoryView::from(&bound_buf.into_any())?;
        let mv_owned = mv.unbind();
        // Cache for `__exit__` to close before calling `send()`.
        self.memoryview = Some(mv_owned.clone_ref(py));
        Ok(mv_owned)
    }

    /// Enter the context manager — returns a writable `memoryview` over the
    /// pre-allocated sample buffer.
    fn __enter__(&mut self, py: Python<'_>) -> PyResult<Py<PyMemoryView>> {
        self.as_memoryview(py)
    }

    /// Exit the context manager.
    ///
    /// - **Normal exit** (`exc_type is None`): release the cached memoryview
    ///   and call `send()`.
    /// - **Exceptional exit**: release the cached memoryview but do **NOT**
    ///   send — the buffer may contain partial or uninitialized data, and
    ///   shipping it would deliver a corrupt frame to downstream nodes
    ///   immediately before the node fails. The handler's `Drop` invalidates
    ///   the state and reclaims the `DataSample`; the original exception
    ///   propagates as normal.
    fn __exit__(
        &mut self,
        exc_type: &Bound<'_, PyAny>,
        _exc_value: &Bound<'_, PyAny>,
        _traceback: &Bound<'_, PyAny>,
    ) -> PyResult<bool> {
        // Always release the cached memoryview so view_count drops back to 0
        // — regardless of whether we send or not. The user is still
        // responsible for releasing any derived views (e.g. numpy arrays)
        // they created inside the `with` block.
        Python::attach(|py| {
            if let Some(mv) = self.memoryview.take() {
                let _ = mv.call_method0(py, "release");
            }
        });

        // Do NOT send on exceptional exit — the buffer contents are
        // unreliable when the body raised mid-fill.
        if !exc_type.is_none() {
            return Ok(false); // propagate the original exception
        }

        self.send().map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to send: {e}"))
        })?;
        Ok(false) // do not suppress exceptions
    }
}

// ---------------------------------------------------------------------------
// State-machine tests
// ---------------------------------------------------------------------------

#[cfg(all(test, Py_3_11))]
mod tests {
    use super::SampleBufferState;
    use std::sync::atomic::Ordering;

    /// `DataSample` requires a real node to construct, so the tests use a
    /// `DataSample`-typed `Option::None` shim. They exercise the atomic state
    /// machine only — the buffer-protocol contract and the
    /// drop-without-send / exit-with-exception fixes are covered end-to-end
    /// by `examples/python-zero-copy-send/test_contract.py`.
    fn fresh_state() -> std::sync::Arc<SampleBufferState> {
        std::sync::Arc::new(SampleBufferState {
            valid: std::sync::atomic::AtomicBool::new(true),
            view_count: std::sync::atomic::AtomicIsize::new(0),
            sample: std::sync::Mutex::new(None),
        })
    }

    #[test]
    fn fresh_state_is_valid_with_no_views() {
        let state = fresh_state();
        assert!(state.valid.load(Ordering::Acquire));
        assert_eq!(state.view_count.load(Ordering::Acquire), 0);
    }

    #[test]
    fn view_count_reflects_paired_get_release() {
        let state = fresh_state();
        state.view_count.fetch_add(1, Ordering::AcqRel);
        state.view_count.fetch_add(1, Ordering::AcqRel);
        assert_eq!(state.view_count.load(Ordering::Acquire), 2);
        state.view_count.fetch_sub(1, Ordering::AcqRel);
        state.view_count.fetch_sub(1, Ordering::AcqRel);
        assert_eq!(state.view_count.load(Ordering::Acquire), 0);
    }

    #[test]
    fn invalidation_is_one_way_and_observable() {
        let state = fresh_state();
        assert!(state.valid.load(Ordering::Acquire));
        state.valid.store(false, Ordering::Release);
        assert!(!state.valid.load(Ordering::Acquire));
    }

    #[test]
    fn view_count_is_independent_of_valid_flag() {
        let state = fresh_state();
        state.view_count.fetch_add(1, Ordering::AcqRel);
        assert!(state.valid.load(Ordering::Acquire));
        state.view_count.fetch_sub(1, Ordering::AcqRel);
        assert!(state.valid.load(Ordering::Acquire));
    }
}
