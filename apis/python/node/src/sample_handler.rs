#[cfg(Py_3_11)]
use pyo3::types::PyMemoryView;
#[cfg(Py_3_11)]
use std::ffi::{c_int, c_void, CString};
use std::ops::DerefMut;
#[cfg(Py_3_11)]
use std::ptr;
use std::sync::atomic::{AtomicBool, AtomicIsize, Ordering};
use std::sync::Arc;

use dora_core::metadata::ArrowTypeInfoExt;
use dora_message::metadata::ArrowTypeInfo;
use dora_node_api::{dora_core::config::DataId, DataSample, DoraNode, MetadataParameters};
use dora_operator_api_python::DelayedCleanup;
use eyre;
#[cfg(Py_3_11)]
use pyo3::exceptions::PyBufferError;
#[cfg(Py_3_11)]
use pyo3::ffi;
use pyo3::{pyclass, pymethods, Bound, Py, PyAny, PyResult, Python};

// ---------------------------------------------------------------------------
// Shared liveness + view-count state  (only needed for the buffer protocol)
// ---------------------------------------------------------------------------

/// Shared (via `Arc`) between `SampleHandler` and the `SampleBuffer` it
/// hands out.
///
/// Invariant: `valid` is set to `false` by `SampleHandler::send()` *before*
/// the `DataSample` allocation is released.  Because `send()` also checks
/// that `view_count == 0` before proceeding, no live `Py_buffer` view can
/// ever read freed memory.
#[cfg(Py_3_11)]
struct SampleBufferState {
    /// `true` while the underlying `DataSample` is still alive.
    valid: AtomicBool,
    /// Number of `Py_buffer` views currently checked out via
    /// `__getbuffer__` that have not yet been released.
    /// Incremented *after* the view is fully initialised so that a panic
    /// during initialisation cannot leave the counter permanently elevated.
    view_count: AtomicIsize,
}

#[cfg(Py_3_11)]
impl SampleBufferState {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            valid: AtomicBool::new(true),
            view_count: AtomicIsize::new(0),
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
/// - **Writable** (`readonly = 0`) as long as `SampleHandler::send()` has
///   not been called.
/// - Acquiring a new view *after* `send()` raises `BufferError`.
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
    /// Raw pointer into the `DataSample` allocation owned by `SampleHandler`.
    ///
    /// # Safety
    /// Valid iff `state.valid == true`.  `SampleHandler::send()` sets
    /// `valid = false` and waits for `view_count == 0` before the
    /// `DataSample` is released, so the pointer is always live while any
    /// `Py_buffer` view that references it is open.
    ptr: *mut u8,
    len: usize,
    state: Arc<SampleBufferState>,
}

// `SampleBuffer` is only ever touched while the GIL is held.
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
    /// Raises `BufferError` after `send()` has been called.
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
                "Cannot acquire buffer view after send() \
                 - the sample has already been sent",
            ));
        }

        if view.is_null() {
            return Err(PyBufferError::new_err("View pointer is null"));
        }

        // SAFETY: `view` is a valid, non-null `Py_buffer *` provided by
        // CPython.  We own the pointed-to memory for the duration of this
        // call and the returned view.
        unsafe {
            // A new strong reference keeps `slf` alive for the view's lifetime.
            (*view).obj = slf.into_any().into_ptr();

            (*view).buf = this.ptr as *mut c_void;
            (*view).len = this.len as ffi::Py_ssize_t;
            (*view).readonly = 0; // writable
            (*view).itemsize = 1;

            // Only allocate the format CString when the caller needs it.
            (*view).format = if (flags & ffi::PyBUF_FORMAT) == ffi::PyBUF_FORMAT {
                CString::new("B").unwrap().into_raw()
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
    /// Must not fail - any error would be silently sent to
    /// `sys.unraisablehook`.
    ///
    /// # Safety
    /// Same contract as `__getbuffer__`: `view` is a valid `Py_buffer *`
    /// for the duration of the call.
    unsafe fn __releasebuffer__(&self, view: *mut ffi::Py_buffer) {
        // Free the format CString we heap-allocated in `__getbuffer__`.
        // SAFETY: `(*view).format` is either null or a pointer we produced
        // via `CString::into_raw`; `CString::from_raw` reclaims it.
        let fmt = unsafe { (*view).format };
        if !fmt.is_null() {
            drop(unsafe { CString::from_raw(fmt) });
        }

        self.state.view_count.fetch_sub(1, Ordering::AcqRel);
    }
}

// ---------------------------------------------------------------------------
// SampleInner
// ---------------------------------------------------------------------------

struct SampleInner {
    data_id: DataId,
    type_info: ArrowTypeInfo,
    parameters: MetadataParameters,
    sample: DataSample,
}

// ---------------------------------------------------------------------------
// SampleHandler
// ---------------------------------------------------------------------------

/// Returned by `Node.send_output_raw`.
///
/// Allocates a shared-memory (or aligned-heap) region via
/// `DoraNode::allocate_data_sample`.
///
/// On Python >= 3.11 the context-manager and `as_buffer()` API are
/// available, exposing the memory as a zero-copy, writable buffer-protocol
/// object:
///
/// ```python
/// with node.send_output_raw("out", 1024 * 1024) as buf:
///     memoryview(buf)[:] = my_bytes   # or: numpy / struct / ...
/// # buf is sent automatically on __exit__
/// ```
///
/// Manual usage (all Python versions that support this method):
///
/// ```python
/// handler = node.send_output_raw("out", length)
/// buf = handler.as_buffer()          # Python >= 3.11 only
/// memoryview(buf)[:] = my_bytes
/// handler.send()
/// ```
#[pyclass]
pub struct SampleHandler {
    node: DelayedCleanup<DoraNode>,
    inner: Option<SampleInner>,
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
        let inner = SampleInner {
            data_id,
            type_info,
            parameters,
            sample,
        };
        Ok(Self {
            node,
            inner: Some(inner),
            #[cfg(Py_3_11)]
            buffer: None,
            #[cfg(Py_3_11)]
            memoryview: None,
        })
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
    /// `SampleBuffer` - release all `memoryview` / numpy references first.
    pub fn send(&mut self) -> eyre::Result<()> {
        // -- 1. Guard: can only send once ------------------------------------
        let SampleInner {
            data_id,
            type_info,
            parameters,
            sample,
        } = self
            .inner
            .take()
            .ok_or_else(|| eyre::eyre!("Sample has already been sent"))?;

        // -- 2. Guard: all buffer views must be released first ---------------
        //      Only compiled in when the buffer protocol is available.
        #[cfg(Py_3_11)]
        if let Some(buf) = &self.buffer {
            let view_count =
                Python::attach(|py| buf.borrow(py).state.view_count.load(Ordering::Acquire));

            if view_count != 0 {
                // Roll the take back so the handler stays in a consistent state.
                self.inner = Some(SampleInner {
                    data_id,
                    type_info,
                    parameters,
                    sample,
                });
                eyre::bail!(
                    "Cannot send: {view_count} buffer view(s) are still open. \
                     Release all memoryview / numpy references before calling send()."
                );
            }

            // Invalidate the SampleBuffer *before* the DataSample is freed so
            // that any Python handle that later calls __getbuffer__ receives a
            // clear BufferError instead of accessing freed memory.
            Python::attach(|py| {
                buf.borrow(py).state.valid.store(false, Ordering::Release);
            });
        }

        // -- 3. Send (zero additional copies) --------------------------------
        self.node
            .get_mut()
            .send_output_sample(data_id, type_info, parameters, Some(sample))
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
    /// Raises `RuntimeError` after `send()` has been called.
    pub fn as_buffer(&mut self, py: Python<'_>) -> PyResult<Py<SampleBuffer>> {
        // (kept for internal use and for callers that want the raw SampleBuffer)
        if self.inner.is_none() {
            return Err(pyo3::exceptions::PyRuntimeError::new_err(
                "Cannot access buffer after send() \
                 - the sample has already been sent",
            ));
        }

        // Return the cached instance if we already created one.
        if let Some(buf) = &self.buffer {
            return Ok(buf.clone_ref(py));
        }

        // First call: borrow the DataSample slice and wrap the raw pointer.
        let inner = self.inner.as_mut().unwrap();
        let slice: &mut [u8] = inner.sample.deref_mut();
        let ptr = slice.as_mut_ptr();
        let len = slice.len();

        let py_buf = Py::new(
            py,
            SampleBuffer {
                ptr,
                len,
                state: SampleBufferState::new(),
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
    /// Raises `RuntimeError` after `send()` has been called.
    ///
    /// ```python
    /// sample = node.send_output_raw("out", 1024)
    /// mv = sample.as_memoryview()
    /// np.asarray(mv, dtype=np.uint8)[:] = data
    /// sample.send()
    /// ```
    pub fn as_memoryview<'py>(&mut self, py: Python<'py>) -> PyResult<Py<PyMemoryView>> {
        // Return the cached memoryview if one already exists.  This keeps
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

    /// Enter the context manager – returns a writable `memoryview` over the
    /// pre-allocated sample buffer.
    fn __enter__<'py>(&mut self, py: Python<'py>) -> PyResult<Py<PyMemoryView>> {
        self.as_memoryview(py)
    }

    /// Exit the context manager and send the data.
    fn __exit__(
        &mut self,
        _exc_type: &Bound<'_, PyAny>,
        _exc_value: &Bound<'_, PyAny>,
        _traceback: &Bound<'_, PyAny>,
    ) -> PyResult<bool> {
        // Close the memoryview returned by `__enter__` so that its buffer view
        // on `SampleBuffer` is released before `send()` checks `view_count`.
        // The user is still responsible for releasing any derived views (e.g.
        // numpy arrays) they created inside the `with` block.
        Python::attach(|py| {
            if let Some(mv) = self.memoryview.take() {
                let _ = mv.call_method0(py, "release");
            }
        });
        self.send().map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to send: {e}"))
        })?;
        Ok(false) // do not suppress exceptions
    }
}
