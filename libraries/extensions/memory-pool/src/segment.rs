//! A mapped `DORADMA` segment: the data plane of the memory-pool transport.
//!
//! The control plane (the daemon's pool table, orphan sweep and free
//! notifications) is unchanged and lives in `MemoryPoolManager`; this type only
//! owns the bytes.
//!
//! Dropping a [`PoolSegment`] unmaps but never unlinks — releasing a pool is
//! the daemon's job, and a reader may legitimately outlive its writer's handle.

use shared_memory_extended::{Shmem, ShmemConf};

use crate::doradma::{
    self, OFFSET_WRITE_GEN, ParsedHeader, data_offset_for, metadata_json, parse_header,
    write_header,
};
use crate::naming;
use crate::seqlock;

/// Re-exported because it appears in [`PoolSegment::begin_read`]'s signature.
/// The `seqlock` module itself is crate-private: nothing outside this crate
/// has any business publishing a generation, which is what `seqlock`'s writer
/// half lets you do with nothing but an `unsafe` block and a fabricated
/// token — exactly the hole [`PoolSegment::begin_write`] closes.
pub use crate::seqlock::OpeningSample;

/// Mirrors the daemon's per-pool size cap so a node fails locally, with a
/// message naming the limit, instead of after a round trip.
const MAX_POOL_BYTES: usize = 1024 * 1024 * 1024;

/// How a receiver is expected to reach the payload.
///
/// `Shmem` and `Unified` have the same *layout* on the wire — both allocate the
/// full data region and set `ipc_flag = 0` — so a receiver reaches the payload
/// the same way for either, and mapping one for CUDA works whichever it is.
/// They are not byte-identical: the metadata JSON records a different
/// `pinned_type` (`"cpu"` / `"cuda"`) and a different `transport` string, and
/// since those strings differ in length so does `json_len`. Nothing reads
/// around that — `json_len` and `data_offset` come out of the header — but a
/// test that diffs two segments will see it.
///
/// Nothing in this crate branches on `pinned_type` — [`Transport`], resolved
/// against the header's `ipc_flag`, is what every path here acts on. It is
/// written because a *Python* consumer branches on it: it takes the
/// torch-backed CUDA read path unless `pinned_type` is exactly `"cpu"`, so a
/// `Unified` pool is not readable by a CPU Python node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transport {
    /// CPU receiver: read the data region directly.
    Shmem,
    /// CUDA receiver on a platform without CUDA IPC (Tegra / integrated GPU):
    /// `cudaHostRegister(..., cudaHostRegisterMapped)` the whole segment and
    /// take the device alias with `cudaHostGetDevicePointer`.
    Unified,
    /// CUDA receiver on a discrete GPU: the payload lives in device memory and
    /// the segment carries only the header plus an IPC handle. Produced by the
    /// Python binding; this crate reads it but never writes it.
    Ipc,
}

impl Transport {
    /// The `pinned_type` value recorded in the metadata JSON and in the daemon
    /// parameters. Keeps Python's two-valued vocabulary.
    pub fn pinned_type(self) -> &'static str {
        match self {
            Transport::Shmem => "cpu",
            Transport::Unified | Transport::Ipc => "cuda",
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Transport::Shmem => "shmem",
            Transport::Unified => "unified",
            Transport::Ipc => "ipc",
        }
    }
}

impl std::str::FromStr for Transport {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "shmem" => Ok(Transport::Shmem),
            "unified" => Ok(Transport::Unified),
            "ipc" => Ok(Transport::Ipc),
            other => Err(format!(
                "unknown transport `{other}`; expected `shmem`, `unified` or `ipc`"
            )),
        }
    }
}

/// Reject a requested pool size before anything is allocated.
///
/// Split out from [`PoolSegment::create`] so the rule can be tested without
/// `ftruncate`ing a real gigabyte-sized segment.
fn validate_pool_size(size: usize) -> Result<(), String> {
    if size == 0 {
        return Err("pool size must be greater than zero".to_string());
    }
    if size > MAX_POOL_BYTES {
        return Err(format!(
            "pool size {size} bytes exceeds the daemon's 1 GiB per-pool cap"
        ));
    }
    Ok(())
}

/// Bytes per element for the dtype strings dora's senders actually write.
///
/// Covers numpy names (`float32`), torch's `str(tensor.dtype)` form
/// (`torch.float32`, which is what `dora.cuda.get_tensor_info` writes) and the
/// numpy array-interface typestr (`<f4`, `|b1`), whose trailing digits *are*
/// the item size. `None` means "not a vocabulary we know", which is not an
/// error: a sender may legitimately use a dtype we have not enumerated, and
/// [`validate_shape`] falls back to a one-byte-per-element bound for those.
fn element_size(dtype: &str) -> Option<usize> {
    let name = dtype.strip_prefix("torch.").unwrap_or(dtype);
    match name {
        "bool" | "int8" | "uint8" => return Some(1),
        "int16" | "uint16" | "float16" | "bfloat16" => return Some(2),
        "int32" | "uint32" | "float32" => return Some(4),
        "int64" | "uint64" | "float64" | "complex64" => return Some(8),
        "complex128" | "float128" => return Some(16),
        "complex256" => return Some(32),
        _ => {}
    }
    // numpy array-interface typestr: byte-order char, kind char, item size.
    // `S`/`U` are excluded — their trailing count is characters, not bytes.
    let mut chars = name.chars();
    let order = chars.next()?;
    let kind = chars.next()?;
    if !matches!(order, '<' | '>' | '|' | '=') || !matches!(kind, 'b' | 'i' | 'u' | 'f' | 'c') {
        return None;
    }
    chars.as_str().parse::<usize>().ok().filter(|&n| n > 0)
}

/// Check that the declared shape describes something that fits in the declared
/// payload.
///
/// A consumer builds its view straight out of `shape` and `dtype` — the C++
/// line is `cv::Mat(shape[0], shape[1], CV_8UC4, ptr)` — so a producer
/// declaring `size: 64` with `shape: [4096, 4096]` would hand it a 16 MB view
/// of a 64-byte buffer while passing every other check in this file.
///
/// The bound is `<=`, not `==`. Only an over-large shape is a memory-safety
/// problem; a payload larger than the shape needs is merely unused, it is what
/// a producer that page-aligns its pool size legitimately writes, and Python's
/// own reader accepts it (`apis/python/node/dora/cuda.py` rejects only on
/// `expected_bytes > size`). Requiring equality would break interop in exchange
/// for no safety.
fn validate_shape(dtype: &str, shape: &[usize], declared_size: usize) -> Result<(), String> {
    // An absent `shape` key decodes to an empty vec, indistinguishable from a
    // genuine scalar shape. Both fall out correctly without a special case:
    // the product of no dimensions is 1, so a shapeless segment is checked as
    // a single element and can never exceed a pool that holds anything at all.
    let mut elements: usize = 1;
    for &dim in shape {
        elements = elements
            .checked_mul(dim)
            .ok_or_else(|| format!("metadata shape {shape:?} overflows when multiplied out"))?;
    }
    let (bytes, unit) = match element_size(dtype) {
        Some(element) => (
            elements.checked_mul(element).ok_or_else(|| {
                format!("metadata shape {shape:?} of `{dtype}` overflows when sized")
            })?,
            format!("`{dtype}` ({element} bytes/element)"),
        ),
        // Unknown dtype: still enforce the weakest true lower bound, one byte
        // per element. That is what catches the shape/size mismatch above.
        None => (
            elements,
            format!("unrecognized dtype `{dtype}` (assuming 1 byte/element)"),
        ),
    };
    if bytes > declared_size {
        return Err(format!(
            "metadata shape {shape:?} of {unit} needs {bytes} bytes but the pool declares only \
             {declared_size}"
        ));
    }
    Ok(())
}

/// A mapped pool segment, from either side.
///
/// # The payload copy is a deliberate data race
///
/// A reader copies the payload while the writer may be overwriting it: that
/// copy is a plain, non-atomic access to memory another process can be storing
/// to, which is a data race under Rust's memory model and something a threaded
/// Miri or TSan run will flag. This is not an oversight and it is not fixable
/// in a seqlock — mitigating it, rather than preventing it, is the whole
/// design. The reader is never promised a coherent copy, only the ability to
/// *detect* an incoherent one: [`begin_read`](Self::begin_read) and
/// [`read_valid`](Self::read_valid) bracket the copy, and the caller discards
/// it whenever the closing sample disagrees with the opening one, so a torn
/// copy is never handed on. Making the copy itself race-free would mean
/// per-element atomics, which would cost more than the zero-copy transport
/// saves and still would not help the C++ and Python peers on the other end —
/// the wire format is plain bytes. What *is* race-free is the detection
/// mechanism: the generation word is only ever touched through `AtomicU64`
/// (see [`crate::seqlock`]).
pub struct PoolSegment {
    shmem: Shmem,
    header: ParsedHeader,
    transport: Transport,
    name: String,
    /// The baseline generation of an open write cycle, if one is open.
    ///
    /// Held here rather than returned to the caller. A caller-held token
    /// crossing an FFI boundary is one wrong integer away from publishing an
    /// even generation over a frame that is still being overwritten — the one
    /// failure a seqlock cannot detect afterwards, since the corrupt frame then
    /// looks complete to every reader forever. Keeping it inside also makes a
    /// doubled `begin_write` and a missing `end_write` observable, neither of
    /// which a token-passing API can see.
    pending_write: Option<u64>,
}

// Hand-written because `Shmem` is not `Debug`, and required rather than
// optional: `Result<PoolSegment, String>::unwrap_err()` — how every rejection
// test here asserts — does not compile without it, and the geometry printed
// below is what a failed assertion has to show to be worth reading.
//
// It reports no mapping address on purpose: an address is meaningless in any
// other process (see `MemoryPoolMetadata::ptr`) and only invites cross-process
// misuse.
impl std::fmt::Debug for PoolSegment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PoolSegment")
            .field("name", &self.name)
            .field("transport", &self.transport)
            .field("segment_bytes", &self.segment_bytes())
            .field("payload_offset", &self.payload_offset())
            .field("declared_size", &self.declared_size())
            .field("payload_len", &self.payload_len())
            .field("dtype", &self.dtype())
            .field("shape", &self.shape())
            .field("ipc_present", &self.ipc_present())
            .finish()
    }
}

// `Shmem` holds raw pointers, so it is neither `Send` nor `Sync`. A
// `PoolSegment` is owned by exactly one thread (the node's event loop) and the
// mapping address is stable for the life of the mapping.
unsafe impl Send for PoolSegment {}

impl PoolSegment {
    /// Create and map a new segment, writing the header and metadata JSON.
    ///
    /// The segment is created with `set_owner(false)` so dropping this handle
    /// does not unlink it: the daemon owns the segment's lifetime and unlinks
    /// it on free, on dataflow shutdown, or during its orphan sweep.
    pub fn create(
        name: &str,
        size: usize,
        dtype: &str,
        shape: &[usize],
        transport: Transport,
    ) -> Result<Self, String> {
        validate_pool_size(size)?;
        validate_shape(dtype, shape, size)?;
        if transport == Transport::Ipc {
            return Err(
                "transport `ipc` requires exporting a CUDA IPC handle, which this binding \
                 does not do; use `unified` for a CUDA receiver"
                    .to_string(),
            );
        }

        let json = metadata_json(
            size,
            dtype,
            shape,
            transport.pinned_type(),
            transport.as_str(),
        );
        let total = data_offset_for(json.len()) + size;

        let mut shmem = ShmemConf::new()
            .os_id(name)
            .size(total)
            .writable(true)
            .create()
            .map_err(|e| {
                format!(
                    "failed to create pool segment `{name}` ({e}); a stale segment or another \
                     node may already hold that name"
                )
            })?;
        shmem.set_owner(false);

        // A `&mut [u8]` over the whole segment is sound here in a way it would
        // not be in `open`: this segment was just created `O_EXCL` and has not
        // been registered with the daemon, so no peer can know its name yet,
        // let alone map it. This is the only moment the payload region has no
        // possible concurrent accessor.
        let header = {
            let buf = unsafe { std::slice::from_raw_parts_mut(shmem.as_ptr(), total) };
            write_header(buf, &json).and_then(|()| parse_header(buf, total))
        };
        let header = match header {
            Ok(header) => header,
            Err(e) => {
                // Unreachable today — `write_header` cannot fail once `total`
                // covers the header, and `parse_header` cannot reject bytes
                // `write_header` just produced. Hardening, not a live leak fix:
                // the failure it guards against would be silent and permanent,
                // because `set_owner(false)` means dropping `shmem` here unmaps
                // without unlinking and strands a name `O_EXCL` can never
                // reuse.
                let _ = naming::unlink_segment(name);
                return Err(e);
            }
        };

        Ok(Self {
            shmem,
            header,
            transport,
            name: name.to_string(),
            pending_write: None,
        })
    }

    /// Map an existing segment created by this crate or by the Python binding.
    pub fn open(name: &str) -> Result<Self, String> {
        let shmem = ShmemConf::new()
            .os_id(name)
            .writable(true)
            .open()
            .map_err(|e| format!("failed to open pool segment `{name}`: {e}"))?;

        let segment_bytes = shmem.len();
        let base = shmem.as_ptr() as *const u8;
        let name_err = |e: String| format!("pool segment `{name}`: {e}");

        // Two steps so that no `&[u8]` ever covers the payload. The payload is
        // being written by a peer process, and a shared reference to it would
        // alias a concurrent write — UB whether or not anything reads through
        // the reference. The header+metadata region is safe to reference
        // because it is written once, before the pool is registered, and a
        // consumer only learns the pool exists through that registration.
        //
        // `header_region_len` caps the length at `data_offset`, so the slice
        // provably stops at the payload start rather than merely inside the
        // mapping. It reads only bytes 0..HEADER_SIZE, which is why the first
        // slice is clamped to the mapping length.
        let region_len = {
            let fixed = unsafe {
                std::slice::from_raw_parts(base, segment_bytes.min(doradma::HEADER_SIZE))
            };
            doradma::header_region_len(fixed, segment_bytes).map_err(name_err)?
        };
        let header_region = unsafe { std::slice::from_raw_parts(base, region_len) };

        // `parse_header` has already validated `data_offset`, `json_len` and
        // the declared payload size against the mapping length, and derived the
        // transport from the header's `ipc_flag`. Re-checking `size` here would
        // be worse than redundant: this side cannot see `ipc_flag`'s gating, and
        // a size check applied unconditionally would reject every
        // Python-written CUDA pool, whose segment is header-only while its JSON
        // still declares the full tensor size.
        let header = parse_header(header_region, segment_bytes).map_err(name_err)?;
        // Machine-checks what `header_region_len` promised, against the
        // authoritative parsed value rather than the raw one it read.
        debug_assert!(
            region_len <= header.data_offset,
            "the header slice reached into the payload: {region_len} > {}",
            header.data_offset
        );
        let transport = header
            .metadata
            .transport
            .parse::<Transport>()
            .map_err(name_err)?;
        // `create` refuses a zero-size pool, but a segment this side merely
        // opens was written by someone else — possibly the Python binding,
        // possibly by hand. A zero-length payload is exactly what consumers
        // read as "there is no payload here and there never will be": the C++
        // binding's `try_read_pool` returns `Unavailable` for it, the same
        // answer it gives for an `ipc` pool and for a freed one. A foreign
        // segment declaring `size: 0` would otherwise open cleanly, stay
        // alive, report transport `shmem`, and be unreadable forever with
        // nothing for the caller to diagnose it by. Rejecting it here is what
        // makes "payload_len == 0 implies ipc or freed" true.
        if header.metadata.size == 0 {
            return Err(name_err("pool size must be greater than zero".to_string()));
        }
        // `size` bounds the mapping (except for `ipc`), but nothing so far
        // relates it to `shape` — and `shape` is what a consumer builds its
        // view from.
        validate_shape(
            &header.metadata.dtype,
            &header.metadata.shape,
            header.metadata.size,
        )
        .map_err(name_err)?;

        Ok(Self {
            shmem,
            header,
            transport,
            name: name.to_string(),
            pending_write: None,
        })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// Base of the mapping. This — not the payload start — is what
    /// `cudaHostRegister` must be given: it is page-aligned, while the payload
    /// start is only 256-byte aligned.
    pub fn shm_base(&self) -> u64 {
        self.shmem.as_ptr() as u64
    }

    /// Total mapped bytes, i.e. what to pass alongside [`shm_base`](Self::shm_base).
    pub fn segment_bytes(&self) -> usize {
        self.shmem.len()
    }

    /// Byte offset of the payload within the mapping, or `None` for
    /// [`Transport::Ipc`], whose payload is not in this mapping at all. Pair
    /// with [`payload_len`](Self::payload_len).
    ///
    /// Exists for the CUDA path, which cannot use
    /// [`payload`](Self::payload)'s host address: it registers the whole
    /// mapping with `cudaHostRegister(shm_base(), segment_bytes())`, takes the
    /// device alias with `cudaHostGetDevicePointer`, and then needs this
    /// offset to reach the payload on the device side. Returning an `Option`
    /// applies the same discipline as `payload()` — `shm_base() +
    /// payload_offset()` cannot be formed on an IPC pool without unwrapping a
    /// `None`.
    pub fn payload_offset(&self) -> Option<usize> {
        match self.transport {
            Transport::Ipc => None,
            Transport::Shmem | Transport::Unified => Some(self.header.data_offset),
        }
    }

    /// The payload region: its start address and its length in bytes, or `None`
    /// when there is no payload in this mapping.
    ///
    /// The only way to obtain a payload pointer, deliberately. `None` is
    /// returned for [`Transport::Ipc`], whose data lives in an IPC-imported
    /// device buffer while its segment holds nothing but the header — so the
    /// natural `memcpy(ptr, src, len)` on the other side of the bridge cannot
    /// be handed a one-past-the-end pointer together with a multi-megabyte
    /// length. The returned pair always satisfies
    /// `ptr + len <= shm_base() + segment_bytes()`.
    pub fn payload(&self) -> Option<(u64, usize)> {
        match self.transport {
            Transport::Ipc => None,
            Transport::Shmem | Transport::Unified => Some((
                self.shm_base() + self.header.data_offset as u64,
                self.header.metadata.size,
            )),
        }
    }

    /// Bytes of payload actually present in this mapping — zero for
    /// [`Transport::Ipc`]. This, never [`declared_size`](Self::declared_size),
    /// is what bounds a copy through [`payload`](Self::payload).
    pub fn payload_len(&self) -> usize {
        self.payload().map_or(0, |(_, len)| len)
    }

    /// The `size` field as the metadata JSON declares it.
    ///
    /// Not a bound on this mapping. For [`Transport::Ipc`] it describes the
    /// tensor in the IPC-imported device buffer, and a Python CUDA pool
    /// allocates only the header — so this routinely exceeds
    /// [`segment_bytes`](Self::segment_bytes). Use
    /// [`payload_len`](Self::payload_len) for anything that indexes memory.
    pub fn declared_size(&self) -> usize {
        self.header.metadata.size
    }

    pub fn dtype(&self) -> &str {
        &self.header.metadata.dtype
    }

    pub fn shape(&self) -> &[usize] {
        &self.header.metadata.shape
    }

    /// How a receiver reaches this segment's payload.
    ///
    /// This — not the JSON's `pinned_type` — is the validated answer: it comes
    /// from the header's `ipc_flag`, which the JSON's `transport` key must
    /// agree with. The JSON also carries a `pinned_type`, but nothing
    /// reconciles the two, so there is deliberately no accessor handing that
    /// second, unvalidated string back out. [`Transport::pinned_type`] derives
    /// it where it is needed: the metadata this crate writes, and the daemon
    /// parameters the C++ binding registers.
    pub fn transport(&self) -> Transport {
        self.transport
    }

    pub fn ipc_present(&self) -> bool {
        self.header.ipc_present
    }

    /// The 64-byte CUDA IPC handle, or `None` unless [`ipc_present`](Self::ipc_present).
    /// Only a segment written by the Python binding on a discrete GPU has one.
    pub fn ipc_handle(&self) -> Option<&[u8; doradma::IPC_HANDLE_LEN]> {
        self.header.ipc_handle.as_ref()
    }

    fn gen_ptr(&self) -> *mut u64 {
        unsafe { self.shmem.as_ptr().add(OFFSET_WRITE_GEN) as *mut u64 }
    }

    /// Open a write cycle, marking the payload incomplete until
    /// [`end_write`](Self::end_write) closes it.
    ///
    /// Fails on an IPC-backed pool — there is no payload here to write — and on
    /// a second call while a cycle is already open. The baseline generation is
    /// kept in the segment rather than handed back, so no caller can supply the
    /// wrong one.
    ///
    /// # Two contracts this method cannot enforce
    ///
    /// **One writer per pool, process-wide.** `pending_write` is per-handle, so
    /// it catches a doubled `begin_write` on *this* `PoolSegment` and nothing
    /// else. Two handles onto the same segment — two [`open`](Self::open)
    /// calls, or a writer in another process — each see the generation, each
    /// either increment it or skip the increment because it is already odd,
    /// and each come away with the same baseline; both then interleave payload
    /// stores and both publish an even generation over the result. Readers
    /// accept it, because from the outside it is indistinguishable from one
    /// clean write. Nothing in a seqlock can detect this: the single-writer
    /// rule is the premise the whole scheme rests on, not something it checks.
    /// A pool has exactly one producer, and that is the dataflow's job to
    /// arrange.
    ///
    /// **`PoolSegment` is `Send` but not `Sync`.** A handle may be moved to
    /// another thread; it may not be shared with one. `&self` methods look
    /// harmless but read the same generation word the writer half mutates, and
    /// the write cycle is `&mut self` precisely so the borrow checker
    /// serialises it — which it can only do within a single thread.
    pub fn begin_write(&mut self) -> Result<(), String> {
        if self.transport == Transport::Ipc {
            return Err(format!(
                "pool `{}` is ipc-backed: its payload lives in device memory, not in this \
                 segment, so there is nothing here to write to",
                self.name
            ));
        }
        if self.pending_write.is_some() {
            return Err(format!(
                "pool `{}` already has a write in progress; end_write must close a cycle \
                 before begin_write opens another",
                self.name
            ));
        }
        self.pending_write = Some(unsafe { seqlock::begin_write(self.gen_ptr()) });
        Ok(())
    }

    /// Close the open write cycle. Does nothing when none is open.
    ///
    /// `ok == false` leaves the generation **odd**, so every reader rejects the
    /// pool until the next successful write. There is no rolling back to the
    /// previous frame: the payload is written in place, so a failed write has
    /// already destroyed it.
    pub fn end_write(&mut self, ok: bool) {
        let Some(pre_write_gen) = self.pending_write.take() else {
            return;
        };
        unsafe { seqlock::end_write(self.gen_ptr(), pre_write_gen, ok) }
    }

    /// Whether a write cycle is currently open.
    ///
    /// Lets a caller detect a missing [`end_write`](Self::end_write) — a bug
    /// that is otherwise invisible until readers start rejecting every frame.
    pub fn write_in_progress(&self) -> bool {
        self.pending_write.is_some()
    }

    /// Copy `data` into the payload under the seqlock.
    ///
    /// `data` must be exactly [`payload_len`](Self::payload_len) bytes. A pool
    /// frame is fixed-size: nothing on the wire records a per-frame length, so
    /// a short write would leave the previous frame's bytes in the tail and
    /// then publish the whole buffer as a complete new frame. A caller with
    /// genuinely variable-length payloads should write through
    /// [`payload`](Self::payload) under
    /// [`begin_write`](Self::begin_write)/[`end_write`](Self::end_write) and
    /// carry the length in its own message alongside the pool reference.
    pub fn write(&mut self, data: &[u8]) -> Result<(), String> {
        let Some((ptr, len)) = self.payload() else {
            return Err(format!(
                "pool `{}` is ipc-backed: its payload lives in device memory, not in this \
                 segment, so there is nothing here to write to",
                self.name
            ));
        };
        if data.len() != len {
            return Err(format!(
                "write of {} bytes does not fill the {len}-byte pool `{}`; a pool frame is \
                 fixed-size, and a partial write would publish the previous frame's tail as \
                 part of this one",
                data.len(),
                self.name
            ));
        }
        self.begin_write()?;
        unsafe {
            std::ptr::copy_nonoverlapping(data.as_ptr(), ptr as *mut u8, len);
        }
        self.end_write(true);
        Ok(())
    }

    /// Sample the generation before reading the payload.
    ///
    /// Spins while a write is in progress, up to a fixed budget, then returns
    /// the odd sample it saw — which makes [`read_valid`](Self::read_valid)
    /// return false. A writer killed mid-write therefore cannot hang a reader.
    pub fn begin_read(&self) -> seqlock::OpeningSample {
        const SPIN_BUDGET: u32 = 10_000;
        for _ in 0..SPIN_BUDGET {
            let sample = unsafe { seqlock::begin_read(self.gen_ptr()) };
            if sample.is_complete() {
                return sample;
            }
            std::hint::spin_loop();
        }
        unsafe { seqlock::begin_read(self.gen_ptr()) }
    }

    /// True when the payload read since [`begin_read`](Self::begin_read) is
    /// intact: no write started or finished in between.
    pub fn read_valid(&self, opening: seqlock::OpeningSample) -> bool {
        unsafe { seqlock::read_completed(self.gen_ptr(), opening) }
    }
}

/// These tests create real files in `/dev/shm`, a process-global namespace.
/// They are Linux-only for two reasons that are properties of the platform,
/// not of this crate: macOS caps a POSIX shm name at 31 bytes (every name
/// built here is longer) and neither macOS nor Windows has the `/dev/shm`
/// path `unlink` removes. The memory-pool transport is already Linux-only —
/// `MemoryPoolManager::free_shared_memory` returns an "unavailable on this
/// platform" error everywhere else.
#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;

    /// Every test needs a segment name no other test (or run) uses, because
    /// /dev/shm is process-global.
    fn unique_pool_id(tag: &str) -> String {
        use std::sync::atomic::{AtomicU64, Ordering};
        static COUNTER: AtomicU64 = AtomicU64::new(0);
        format!(
            "{tag}-{}-{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        )
    }

    /// Owns a segment *name* and unlinks it on drop.
    ///
    /// A test that panics half-way through must still clean up: `create` is
    /// `O_EXCL`, so a leaked segment does not merely litter `/dev/shm`, it
    /// makes that name permanently unusable and turns one failure into a
    /// cascade of false ones. Unlinking from `Drop` runs on the panic path too,
    /// which a trailing cleanup call does not.
    struct SegmentGuard {
        name: String,
    }

    impl SegmentGuard {
        fn new(tag: &str) -> Self {
            let name = crate::naming::segment_name("dftest", "nodetest", &unique_pool_id(tag))
                .expect("valid segment name");
            Self { name }
        }

        fn name(&self) -> &str {
            &self.name
        }
    }

    impl Drop for SegmentGuard {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(format!("/dev/shm/{}", self.name));
        }
    }

    fn segment_exists(name: &str) -> bool {
        std::path::Path::new(&format!("/dev/shm/{name}")).exists()
    }

    fn create_test_segment(tag: &str, size: usize) -> (SegmentGuard, PoolSegment) {
        let guard = SegmentGuard::new(tag);
        let seg = PoolSegment::create(guard.name(), size, "uint8", &[size], Transport::Shmem)
            .expect("create segment");
        (guard, seg)
    }

    /// Read the payload back the way a consumer does: through the checked
    /// accessor, never by reconstructing the pointer.
    fn read_payload(seg: &PoolSegment) -> Vec<u8> {
        let (ptr, len) = seg.payload().expect("a non-ipc pool has a payload");
        unsafe { std::slice::from_raw_parts(ptr as *const u8, len) }.to_vec()
    }

    /// Create a segment from a hand-written metadata JSON, the way a peer in
    /// another language would. `payload_bytes` is allocated after the header;
    /// `ipc_flag` sets header byte 24.
    fn write_raw_segment(guard: &SegmentGuard, json: &str, payload_bytes: usize, ipc_flag: bool) {
        let total = doradma::data_offset_for(json.len()) + payload_bytes;
        let mut shmem = ShmemConf::new()
            .os_id(guard.name())
            .size(total)
            .writable(true)
            .create()
            .expect("create raw segment");
        shmem.set_owner(false);
        let buf = unsafe { std::slice::from_raw_parts_mut(shmem.as_ptr(), total) };
        write_header(buf, json).expect("write header");
        if ipc_flag {
            buf[doradma::OFFSET_IPC_FLAG..doradma::OFFSET_IPC_FLAG + 8]
                .copy_from_slice(&1u64.to_le_bytes());
        }
    }

    /// The segment shape the Python binding produces for a CUDA receiver on a
    /// discrete GPU: header-only allocation, `ipc_flag = 1`, no `transport`
    /// key, and a JSON `size` describing the tensor in the IPC-imported device
    /// buffer — far larger than the segment itself.
    fn create_python_style_ipc_segment(guard: &SegmentGuard, declared_size: usize) {
        write_raw_segment(
            guard,
            &format!(
                r#"{{"size":{declared_size},"dtype":"uint8","shape":[{declared_size}],"pinned_type":"cuda"}}"#
            ),
            0,
            true,
        );
    }

    #[test]
    fn create_then_open_sees_the_same_geometry() {
        let (guard, writer) = create_test_segment("geometry", 128);
        let reader = PoolSegment::open(guard.name()).expect("open");

        assert_eq!(reader.declared_size(), 128);
        assert_eq!(reader.payload_len(), 128);
        assert_eq!(reader.dtype(), "uint8");
        assert_eq!(reader.shape(), &[128]);
        assert_eq!(reader.payload_offset(), writer.payload_offset());
        assert_eq!(reader.segment_bytes(), writer.segment_bytes());
        assert_eq!(reader.transport(), Transport::Shmem);
        assert_eq!(reader.transport().pinned_type(), "cpu");
        assert!(!reader.ipc_present());
        assert!(reader.ipc_handle().is_none());
    }

    #[test]
    fn payload_written_by_the_writer_is_visible_to_the_reader() {
        let (guard, mut writer) = create_test_segment("payload", 64);
        writer.write(&[0xAB; 64]).expect("write");

        let reader = PoolSegment::open(guard.name()).expect("open");
        let seen = read_payload(&reader);
        assert!(
            seen.iter().all(|&b| b == 0xAB),
            "reader saw {seen:?}, not the written payload"
        );
    }

    /// The payload must start after the header — writing at the mapping base
    /// would overwrite the header the reader depends on — and must end inside
    /// the mapping.
    #[test]
    fn the_payload_lies_after_the_header_and_inside_the_mapping() {
        let (guard, mut writer) = create_test_segment("offset", 32);
        let offset = writer.payload_offset().expect("payload offset");
        assert!(offset >= doradma::HEADER_SIZE);
        let (ptr, len) = writer.payload().expect("payload");
        assert_eq!(ptr, writer.shm_base() + offset as u64);
        assert!(
            ptr + len as u64 <= writer.shm_base() + writer.segment_bytes() as u64,
            "the payload must not extend past the mapping"
        );
        writer.write(&[0xCD; 32]).expect("write");

        let reader = PoolSegment::open(guard.name()).expect("open");
        assert_eq!(
            reader.declared_size(),
            32,
            "the header must survive the payload write"
        );
        assert!(read_payload(&reader).iter().all(|&b| b == 0xCD));
    }

    #[test]
    fn a_completed_write_leaves_an_even_generation_a_reader_accepts() {
        let (guard, mut writer) = create_test_segment("seqlock", 32);
        writer.write(&[1; 32]).expect("write");

        let reader = PoolSegment::open(guard.name()).expect("open");
        let opening = reader.begin_read();
        assert!(opening.is_complete());
        assert!(reader.read_valid(opening));
    }

    /// The reader must see a torn frame as invalid, not as data.
    #[test]
    fn a_write_in_progress_is_rejected_by_the_reader() {
        let (guard, mut writer) = create_test_segment("torn", 32);
        let reader = PoolSegment::open(guard.name()).expect("open");

        let opening = reader.begin_read();
        writer.begin_write().expect("begin");
        assert!(!reader.read_valid(opening), "mid-write must be invalid");
        writer.end_write(true);
        assert!(
            !reader.read_valid(opening),
            "a new frame invalidates the old sample"
        );
    }

    /// `begin_read` gives up after its spin budget and hands back the odd
    /// sample it saw. `read_valid` must reject that on parity alone — the two
    /// samples agree, so equality is not enough.
    #[test]
    fn an_odd_opening_sample_is_rejected_on_parity_even_though_both_samples_agree() {
        let (guard, mut writer) = create_test_segment("oddsample", 32);
        let reader = PoolSegment::open(guard.name()).expect("open");

        writer.begin_write().expect("begin");
        let opening = reader.begin_read();
        assert!(
            !opening.is_complete(),
            "begin_read must return the odd sample it gave up on"
        );
        assert!(
            !reader.read_valid(opening),
            "an odd sample must never be accepted"
        );
        writer.end_write(true);
    }

    /// The baseline generation lives in the segment, so there is no token a
    /// caller can get wrong. The only ways to reach `end_write` are with a
    /// matching `begin_write` or with none at all, and neither can publish an
    /// even generation over a frame that is still being written.
    #[test]
    fn end_write_without_begin_write_cannot_publish_a_frame() {
        let (guard, mut writer) = create_test_segment("nobegin", 32);
        let reader = PoolSegment::open(guard.name()).expect("open");

        // A write is in flight, marked by the odd generation.
        writer.begin_write().expect("begin");
        assert!(!reader.begin_read().is_complete());

        // A second handle onto the same segment closes a cycle it never opened.
        // With a caller-supplied token this is the call that would publish
        // `pre + 2` over the in-flight frame and make it look complete to every
        // reader, forever.
        let mut stray = PoolSegment::open(guard.name()).expect("open");
        assert!(!stray.write_in_progress());
        stray.end_write(true);

        assert!(
            !reader.begin_read().is_complete(),
            "a stray end_write must not have published the in-flight frame"
        );
        writer.end_write(true);
        assert!(
            reader.begin_read().is_complete(),
            "the real writer must still be able to close its cycle"
        );
    }

    #[test]
    fn a_second_begin_write_is_refused_rather_than_losing_the_baseline() {
        let (_guard, mut writer) = create_test_segment("doublebegin", 32);
        writer.begin_write().expect("first begin");
        assert!(writer.write_in_progress());

        let err = writer.begin_write().unwrap_err();
        assert!(
            err.contains("already has a write in progress"),
            "unexpected error: {err}"
        );

        writer.end_write(true);
        assert!(!writer.write_in_progress());
    }

    /// The bracket form must refuse an ipc pool exactly as `write` does — it is
    /// the form a zero-copy node uses, so guarding only `write` guards the path
    /// nobody takes.
    #[test]
    fn begin_write_is_refused_on_an_ipc_pool() {
        let guard = SegmentGuard::new("ipcbegin");
        create_python_style_ipc_segment(&guard, 8 * 1024 * 1024);

        let mut seg = PoolSegment::open(guard.name()).expect("open");
        let err = seg.begin_write().unwrap_err();
        assert!(err.contains("ipc-backed"), "unexpected error: {err}");
        assert!(
            !seg.write_in_progress(),
            "a refused begin_write must not leave a cycle open"
        );
    }

    /// A frame is fixed-size. A short write would leave the previous frame's
    /// tail in place and publish the result as a complete new frame.
    #[test]
    fn write_rejects_a_payload_that_does_not_fill_the_pool() {
        let (guard, mut writer) = create_test_segment("shortwrite", 16);
        writer.write(&[0xFF; 16]).expect("first full frame");

        let err = writer.write(&[0x11; 8]).unwrap_err();
        assert!(
            err.contains("8 bytes does not fill the 16-byte pool"),
            "unexpected error: {err}"
        );

        let reader = PoolSegment::open(guard.name()).expect("open");
        assert!(
            read_payload(&reader).iter().all(|&b| b == 0xFF),
            "a rejected short write must not have touched the frame"
        );
    }

    #[test]
    fn write_rejects_more_bytes_than_the_pool_holds() {
        let (_guard, mut writer) = create_test_segment("overflow", 16);
        let err = writer.write(&[0; 17]).unwrap_err();
        assert!(
            err.contains("17 bytes does not fill the 16-byte pool"),
            "unexpected error: {err}"
        );
    }

    /// The size rule as a pure predicate, so the boundary can be pinned without
    /// `ftruncate`ing a real gigabyte.
    #[test]
    fn pool_size_must_be_nonzero_and_within_the_daemons_cap() {
        assert!(
            validate_pool_size(0)
                .unwrap_err()
                .contains("greater than zero")
        );
        assert!(validate_pool_size(1).is_ok());
        assert!(validate_pool_size(MAX_POOL_BYTES).is_ok(), "the cap itself");
        let err = validate_pool_size(MAX_POOL_BYTES + 1).unwrap_err();
        assert!(err.contains("1 GiB"), "unexpected error: {err}");
    }

    #[test]
    fn create_rejects_a_zero_size_pool() {
        let guard = SegmentGuard::new("zero");
        let err =
            PoolSegment::create(guard.name(), 0, "uint8", &[0], Transport::Shmem).unwrap_err();
        assert!(err.contains("greater than zero"), "unexpected error: {err}");
        assert!(
            !segment_exists(guard.name()),
            "a rejected size must not leave a segment behind"
        );
    }

    /// A foreign segment can declare `size: 0` even though `create` never
    /// writes one. It maps fine, reports `shmem`, and stays alive — and every
    /// consumer reads its zero-length payload as "no payload, permanently",
    /// the answer reserved for `ipc` and for a freed pool. `open` has to
    /// refuse it, or that reading is wrong and the caller has nothing to
    /// diagnose the difference with.
    #[test]
    fn open_rejects_a_foreign_segment_declaring_a_zero_size() {
        let guard = SegmentGuard::new("zeroopen");
        write_raw_segment(
            &guard,
            r#"{"size":0,"dtype":"uint8","shape":[0],"pinned_type":"cpu"}"#,
            0,
            false,
        );
        let err = PoolSegment::open(guard.name()).unwrap_err();
        assert!(err.contains("greater than zero"), "unexpected error: {err}");
    }

    /// Writing `ipc` means exporting a CUDA IPC handle, which this crate never
    /// does. Accepting it would produce a segment whose header claims a handle
    /// that is all zeros.
    #[test]
    fn create_rejects_the_ipc_transport_it_cannot_export_a_handle_for() {
        let guard = SegmentGuard::new("ipccreate");
        let err =
            PoolSegment::create(guard.name(), 64, "uint8", &[64], Transport::Ipc).unwrap_err();
        assert!(err.contains("use `unified`"), "unexpected error: {err}");
        assert!(
            !segment_exists(guard.name()),
            "a rejected transport must not leave a segment behind"
        );
    }

    /// `unified` allocates the same *layout* as `shmem` — no IPC flag and the
    /// full data region — so a reader reaches the payload identically for
    /// either. The bytes are not identical: the JSON records a different
    /// `transport` and a different `pinned_type`, and so a different
    /// `json_len`. This asserts the layout, which is what interoperates.
    #[test]
    fn unified_allocates_the_same_payload_layout_as_shmem() {
        let guard = SegmentGuard::new("unified");
        let seg =
            PoolSegment::create(guard.name(), 64, "uint8", &[64], Transport::Unified).unwrap();
        assert_eq!(
            seg.segment_bytes(),
            seg.payload_offset().expect("payload offset") + 64
        );

        let reader = PoolSegment::open(guard.name()).expect("open");
        assert_eq!(reader.transport(), Transport::Unified);
        assert_eq!(reader.transport().pinned_type(), "cuda");
        assert_eq!(reader.declared_size(), 64);
        assert_eq!(reader.payload_len(), 64);
        assert!(!reader.ipc_present());
    }

    /// The transport comes from the header's `ipc_flag`, never from the JSON's
    /// `pinned_type`. This is the segment where the two disagree: a
    /// hand-rolled sender declaring `pinned_type: "cuda"` with `ipc_flag = 0`
    /// and a full data region. `Shmem` is the actionable answer — the payload
    /// really is here in shmem — and it is the only one that was checked
    /// against the header.
    #[test]
    fn the_transport_follows_the_ipc_flag_when_the_json_pinned_type_disagrees() {
        let guard = SegmentGuard::new("pinnedskew");
        write_raw_segment(
            &guard,
            r#"{"size":64,"dtype":"uint8","shape":[64],"pinned_type":"cuda"}"#,
            64,
            false,
        );
        let seg = PoolSegment::open(guard.name()).expect("open");
        assert_eq!(
            seg.transport(),
            Transport::Shmem,
            "the transport must follow the ipc flag, not the unchecked json field"
        );
        assert_eq!(seg.payload_len(), 64, "the payload is here in the mapping");
    }

    /// `open` must not run a payload-size check of its own. `parse_header`
    /// gates that check on the header's `ipc_flag`, and this is the segment
    /// shape the gate exists for: a size check applied here would reject every
    /// Python-written GPU pool.
    #[test]
    fn open_accepts_a_python_written_header_only_cuda_pool() {
        let guard = SegmentGuard::new("pyipc");
        create_python_style_ipc_segment(&guard, 8 * 1024 * 1024);

        let reader = PoolSegment::open(guard.name()).expect("a header-only ipc pool must open");
        assert_eq!(reader.transport(), Transport::Ipc);
        assert!(reader.ipc_present());
        assert!(reader.ipc_handle().is_some());
        assert_eq!(reader.declared_size(), 8 * 1024 * 1024);
        assert!(
            reader.segment_bytes() < reader.declared_size(),
            "the point of this segment is that its declared size exceeds it"
        );
    }

    /// The flip side of accepting that segment. `declared_size` no longer
    /// bounds the mapping, so the payload accessor must refuse to hand out a
    /// pointer at all rather than one that is already past the end.
    #[test]
    fn an_ipc_pool_hands_out_no_payload_pointer() {
        let guard = SegmentGuard::new("ipcpayload");
        create_python_style_ipc_segment(&guard, 8 * 1024 * 1024);

        let mut seg = PoolSegment::open(guard.name()).expect("open");
        assert!(seg.payload().is_none());
        assert!(seg.payload_offset().is_none());
        assert_eq!(seg.payload_len(), 0);
        assert_eq!(seg.declared_size(), 8 * 1024 * 1024);
        // Reaching past the private field: the offset that would otherwise
        // have been handed out is exactly one past the end of the mapping.
        assert_eq!(seg.header.data_offset, seg.segment_bytes());

        let err = seg.write(&[0xAB; 4096]).unwrap_err();
        assert!(err.contains("ipc-backed"), "unexpected error: {err}");
    }

    /// A consumer builds its view out of `shape` and `dtype`. A shape that
    /// multiplies out past the payload is a 16 MB window onto a 64-byte buffer,
    /// and passes every other check in this file.
    #[test]
    fn open_rejects_a_shape_that_does_not_fit_the_declared_size() {
        let guard = SegmentGuard::new("bigshape");
        write_raw_segment(
            &guard,
            r#"{"size":64,"dtype":"uint8","shape":[4096,4096],"pinned_type":"cpu"}"#,
            64,
            false,
        );
        let err = PoolSegment::open(guard.name()).unwrap_err();
        assert!(
            err.contains("needs 16777216 bytes but the pool declares only 64"),
            "unexpected error: {err}"
        );
    }

    /// The same hazard through the element size rather than the element count.
    #[test]
    fn open_rejects_a_shape_whose_dtype_makes_it_too_large() {
        let guard = SegmentGuard::new("bigdtype");
        write_raw_segment(
            &guard,
            r#"{"size":64,"dtype":"float64","shape":[32],"pinned_type":"cpu"}"#,
            64,
            false,
        );
        let err = PoolSegment::open(guard.name()).unwrap_err();
        assert!(
            err.contains("needs 256 bytes but the pool declares only 64"),
            "unexpected error: {err}"
        );
    }

    /// An unrecognized dtype must not be rejected — a sender may legitimately
    /// use one we have not enumerated — but the one-byte-per-element lower
    /// bound still applies, which is what catches the hazard above.
    #[test]
    fn an_unrecognized_dtype_keeps_the_one_byte_per_element_bound() {
        let ok = SegmentGuard::new("weirdok");
        write_raw_segment(
            &ok,
            r#"{"size":64,"dtype":"posit8","shape":[8,8],"pinned_type":"cpu"}"#,
            64,
            false,
        );
        PoolSegment::open(ok.name()).expect("an unknown dtype must still open");

        let bad = SegmentGuard::new("weirdbad");
        write_raw_segment(
            &bad,
            r#"{"size":64,"dtype":"posit8","shape":[4096,4096],"pinned_type":"cpu"}"#,
            64,
            false,
        );
        let err = PoolSegment::open(bad.name()).unwrap_err();
        assert!(
            err.contains("unrecognized dtype `posit8`"),
            "unexpected error: {err}"
        );
    }

    /// The bound is `<=`, not `==`: a pool larger than its shape needs is safe,
    /// is what a producer that page-aligns its allocation writes, and is
    /// accepted by Python's own reader (`dora/cuda.py` rejects only on
    /// `expected_bytes > size`). Rejecting it would break interop for no safety
    /// gain.
    #[test]
    fn a_pool_larger_than_its_shape_needs_is_accepted() {
        let guard = SegmentGuard::new("padded");
        write_raw_segment(
            &guard,
            r#"{"size":4096,"dtype":"uint8","shape":[100],"pinned_type":"cpu"}"#,
            4096,
            false,
        );
        let seg = PoolSegment::open(guard.name()).expect("an over-allocated pool must open");
        assert_eq!(seg.declared_size(), 4096);
    }

    #[test]
    fn element_size_knows_the_vocabularies_dora_senders_write() {
        // numpy names, torch's `str(tensor.dtype)`, numpy typestr.
        assert_eq!(element_size("uint8"), Some(1));
        assert_eq!(element_size("float32"), Some(4));
        assert_eq!(element_size("torch.float64"), Some(8));
        assert_eq!(element_size("torch.bfloat16"), Some(2));
        assert_eq!(element_size("<f4"), Some(4));
        assert_eq!(element_size("|b1"), Some(1));
        assert_eq!(element_size("<i8"), Some(8));
        // Name forms of the wide numpy types; their typestr forms (`<f16`,
        // `<c32`) already fall out of the generic parse below.
        assert_eq!(element_size("float128"), Some(16));
        assert_eq!(element_size("complex256"), Some(32));
        assert_eq!(element_size("<f16"), Some(16));
        // Not a vocabulary we know: the caller falls back to the 1-byte bound.
        assert_eq!(element_size("posit8"), None);
        assert_eq!(element_size("<U5"), None, "character counts are not bytes");
        assert_eq!(element_size(""), None);
    }

    #[test]
    fn a_shape_that_overflows_when_multiplied_is_rejected_not_wrapped() {
        let err = validate_shape("uint8", &[usize::MAX, 4], 64).unwrap_err();
        assert!(err.contains("overflows"), "unexpected error: {err}");
        let err = validate_shape("float64", &[usize::MAX / 4], 64).unwrap_err();
        assert!(err.contains("overflows"), "unexpected error: {err}");
    }

    /// An absent `shape` key decodes to an empty vec, which the check must
    /// skip: it is indistinguishable from a genuine scalar and neither can
    /// index out of bounds. Rejecting it would break the documented degradation
    /// for older senders.
    #[test]
    fn an_absent_shape_is_not_treated_as_a_zero_sized_tensor() {
        assert!(validate_shape("float64", &[], 4096).is_ok());
        let guard = SegmentGuard::new("noshape");
        write_raw_segment(
            &guard,
            r#"{"size":64,"dtype":"uint8","pinned_type":"cpu"}"#,
            64,
            false,
        );
        let seg = PoolSegment::open(guard.name()).expect("a shapeless segment must open");
        assert!(seg.shape().is_empty());
    }

    #[test]
    fn create_rejects_a_shape_that_does_not_fit_the_size_it_was_given() {
        let guard = SegmentGuard::new("createshape");
        let err =
            PoolSegment::create(guard.name(), 64, "float32", &[64], Transport::Shmem).unwrap_err();
        assert!(err.contains("needs 256 bytes"), "unexpected error: {err}");
        assert!(!segment_exists(guard.name()));
    }

    /// Overwrite one 8-byte little-endian header field of an existing segment.
    fn patch_header_field(guard: &SegmentGuard, offset: usize, value: u64) {
        let shmem = ShmemConf::new()
            .os_id(guard.name())
            .writable(true)
            .open()
            .expect("open for patching");
        let buf = unsafe { std::slice::from_raw_parts_mut(shmem.as_ptr(), shmem.len()) };
        buf[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }

    /// `json_len` is attacker-controlled and unvalidated when `open` needs it,
    /// yet `open` must bound its slice with it. Both values here would clamp
    /// harmlessly against the segment length and still span the payload, so
    /// `open` has to reject them outright — and the message must show the
    /// bounding step rejected them, not `parse_header` afterwards, because by
    /// then the offending slice would already have been formed.
    #[test]
    fn open_rejects_a_json_len_that_would_make_the_header_slice_span_the_payload() {
        // Larger than data_offset, but still inside the segment.
        let a = SegmentGuard::new("badjsonlen");
        PoolSegment::create(a.name(), 8192, "uint8", &[8192], Transport::Shmem).expect("create");
        patch_header_field(&a, doradma::OFFSET_JSON_LEN, 4096);
        let err = PoolSegment::open(a.name()).unwrap_err();
        assert!(
            err.contains("cannot bound the header region"),
            "unexpected error: {err}"
        );

        // Large enough that HEADER_SIZE + json_len wraps.
        let b = SegmentGuard::new("hugejsonlen");
        PoolSegment::create(b.name(), 8192, "uint8", &[8192], Transport::Shmem).expect("create");
        patch_header_field(&b, doradma::OFFSET_JSON_LEN, u64::MAX);
        let err = PoolSegment::open(b.name()).unwrap_err();
        assert!(
            err.contains("cannot bound the header region"),
            "unexpected error: {err}"
        );
    }

    /// A segment too short to hold even the fixed header must be rejected by
    /// the bounding step, before anything indexes into it.
    ///
    /// Deliberately 8 bytes, not merely under 256: `json_len_field` reads the
    /// `json_len` field at bytes 8..16, so a segment shorter than 16 is the
    /// only size at which dropping its length guard is an out-of-bounds index
    /// rather than a redundant check `parse_header` would repeat anyway.
    #[test]
    fn open_rejects_a_segment_shorter_than_the_fixed_header() {
        let guard = SegmentGuard::new("stub");
        let mut shmem = ShmemConf::new()
            .os_id(guard.name())
            .size(8)
            .writable(true)
            .create()
            .expect("create stub segment");
        shmem.set_owner(false);
        drop(shmem);

        let err = PoolSegment::open(guard.name()).unwrap_err();
        assert!(err.contains("too small"), "unexpected error: {err}");
    }

    #[test]
    fn open_fails_with_a_clear_message_when_the_segment_does_not_exist() {
        let guard = SegmentGuard::new("missing");
        let err = PoolSegment::open(guard.name()).unwrap_err();
        assert!(err.contains(guard.name()), "unexpected error: {err}");
    }

    #[test]
    fn dropping_a_segment_does_not_unlink_it() {
        let (guard, writer) = create_test_segment("droptest", 32);
        drop(writer);
        // Still openable: only the daemon's unlink removes a segment.
        let reader = PoolSegment::open(guard.name()).expect("segment must survive drop");
        drop(reader);
        assert!(segment_exists(guard.name()));

        crate::naming::unlink_segment(guard.name()).expect("unlink");
        assert!(
            PoolSegment::open(guard.name()).is_err(),
            "unlink must remove it"
        );
    }
}
