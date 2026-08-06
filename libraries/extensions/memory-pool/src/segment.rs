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
use crate::seqlock;

/// Mirrors the daemon's per-pool size cap so a node fails locally, with a
/// message naming the limit, instead of after a round trip.
const MAX_POOL_BYTES: usize = 1024 * 1024 * 1024;

/// How a receiver is expected to reach the payload.
///
/// `Shmem` and `Unified` are byte-identical on the wire — both allocate the
/// full data region and set `ipc_flag = 0`. They differ only in the declared
/// `pinned_type`, which tells a receiver whether mapping the segment for CUDA
/// is worth doing.
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

/// Unlink a segment name from `/dev/shm`, tolerating an already-gone file.
fn unlink_segment(name: &str) {
    #[cfg(target_os = "linux")]
    {
        let path = format!("/dev/shm/{name}");
        match std::fs::remove_file(&path) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => tracing::warn!("failed to unlink pool segment {path}: {e}"),
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = name;
    }
}

/// A mapped pool segment, from either side.
///
/// # The payload copy is a deliberate data race
///
/// A reader copies the payload while the writer may be overwriting it: that
/// copy is a plain, non-atomic access to memory another process can be storing
/// to, which is a data race under Rust's memory model and something a threaded
/// Miri or TSan run will flag. This is not an oversight and it is not
/// fixable in a seqlock — mitigating it, rather than preventing it, is the
/// whole design. The reader is never promised a coherent copy, only the
/// ability to *detect* an incoherent one: [`begin_read`](Self::begin_read) and
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
}

// Hand-written because `Shmem` is not `Debug`. Reports the geometry a
// mismatched producer/consumer pair needs to diagnose itself, and no mapping
// address — an address is meaningless in any other process (see
// `MemoryPoolMetadata::ptr`) and only invites cross-process misuse.
impl std::fmt::Debug for PoolSegment {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PoolSegment")
            .field("name", &self.name)
            .field("transport", &self.transport)
            .field("segment_bytes", &self.segment_bytes())
            .field("data_offset", &self.data_offset())
            .field("size", &self.size())
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
        if size == 0 {
            return Err("pool size must be greater than zero".to_string());
        }
        if size > MAX_POOL_BYTES {
            return Err(format!(
                "pool size {size} bytes exceeds the daemon's 1 GiB per-pool cap"
            ));
        }
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

        let header = {
            let buf = unsafe { std::slice::from_raw_parts_mut(shmem.as_ptr(), total) };
            write_header(buf, &json).and_then(|()| parse_header(buf))
        };
        let header = match header {
            Ok(header) => header,
            Err(e) => {
                // `set_owner(false)` means dropping `shmem` here would unmap
                // without unlinking, leaving a segment nothing owns and
                // `create` can never reuse — `shm_open` is `O_EXCL`, so that
                // pool id would fail for the life of the machine.
                unlink_segment(name);
                return Err(e);
            }
        };

        Ok(Self {
            shmem,
            header,
            transport,
            name: name.to_string(),
        })
    }

    /// Map an existing segment created by this crate or by the Python binding.
    pub fn open(name: &str) -> Result<Self, String> {
        let shmem = ShmemConf::new()
            .os_id(name)
            .writable(true)
            .open()
            .map_err(|e| format!("failed to open pool segment `{name}`: {e}"))?;

        let buf = unsafe { std::slice::from_raw_parts(shmem.as_ptr() as *const u8, shmem.len()) };
        // `parse_header` has already validated `data_offset`, `json_len` and
        // the declared payload size against the mapping length, and derived
        // the transport from the header's `ipc_flag`. Re-checking `size` here
        // would be worse than redundant: this side cannot see `ipc_flag`'s
        // gating, and a size check applied unconditionally would reject every
        // Python-written CUDA pool, whose segment is header-only while its
        // JSON still declares the full tensor size.
        // Name the segment: a header rejection here is usually a version or
        // language mismatch with whoever wrote it, and the bare message says
        // nothing about which pool went wrong.
        let header = parse_header(buf).map_err(|e| format!("pool segment `{name}`: {e}"))?;
        let transport = header
            .metadata
            .transport
            .parse::<Transport>()
            .map_err(|e| format!("pool segment `{name}`: {e}"))?;

        Ok(Self {
            shmem,
            header,
            transport,
            name: name.to_string(),
        })
    }

    /// Unlink the segment from `/dev/shm`. Existing mappings stay valid until
    /// their holders drop them, as POSIX requires.
    pub fn unlink(&self) {
        unlink_segment(&self.name);
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// Base of the mapping. This — not [`host_ptr`](Self::host_ptr) — is what
    /// `cudaHostRegister` must be given: it is page-aligned, while the payload
    /// start is only 256-byte aligned.
    pub fn shm_base(&self) -> u64 {
        self.shmem.as_ptr() as u64
    }

    /// Total mapped bytes, i.e. what to pass alongside [`shm_base`](Self::shm_base).
    pub fn segment_bytes(&self) -> usize {
        self.shmem.len()
    }

    pub fn data_offset(&self) -> usize {
        self.header.data_offset
    }

    /// Start of the payload: `shm_base() + data_offset()`.
    ///
    /// `host_ptr() .. host_ptr() + size()` is inside the mapping for every
    /// transport except [`Transport::Ipc`], where the segment is header-only
    /// and there is no payload here at all — see [`size`](Self::size).
    pub fn host_ptr(&self) -> u64 {
        self.shm_base() + self.header.data_offset as u64
    }

    /// Payload bytes as declared in the metadata JSON.
    ///
    /// For [`Transport::Ipc`] this describes the tensor in the IPC-imported
    /// device buffer, **not** this segment: a Python CUDA pool allocates only
    /// the header, so `size()` there routinely exceeds
    /// [`segment_bytes`](Self::segment_bytes). Anything that turns
    /// [`host_ptr`](Self::host_ptr) plus this value into a slice must exclude
    /// that case, as [`write`](Self::write) does.
    pub fn size(&self) -> usize {
        self.header.metadata.size
    }

    pub fn dtype(&self) -> &str {
        &self.header.metadata.dtype
    }

    pub fn shape(&self) -> &[usize] {
        &self.header.metadata.shape
    }

    pub fn pinned_type(&self) -> &str {
        &self.header.metadata.pinned_type
    }

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

    /// Open a write cycle; pass the returned value to [`end_write`](Self::end_write).
    pub fn begin_write(&mut self) -> u64 {
        unsafe { seqlock::begin_write(self.gen_ptr()) }
    }

    /// Close a write cycle.
    ///
    /// `ok == false` leaves the generation **odd**, so every reader rejects the
    /// pool until the next successful write. There is no rolling back to the
    /// previous frame: the payload is written in place, so a failed write has
    /// already destroyed it.
    pub fn end_write(&mut self, pre_write_gen: u64, ok: bool) {
        unsafe { seqlock::end_write(self.gen_ptr(), pre_write_gen, ok) }
    }

    /// Copy `data` into the payload under the seqlock.
    ///
    /// Refuses an IPC-backed pool outright. For every other transport
    /// `parse_header` has already established `data_offset + size <=
    /// segment_bytes`, so bounding the copy by [`size`](Self::size) keeps it
    /// inside the mapping — but that invariant is exactly the one an IPC pool
    /// does not hold: its `size` describes a tensor in device memory while its
    /// segment is header-only, so the same bound would authorise a write far
    /// past the end of the mapping.
    pub fn write(&mut self, data: &[u8]) -> Result<(), String> {
        if self.transport == Transport::Ipc {
            return Err(format!(
                "pool `{}` is ipc-backed: its payload lives in device memory, not in this \
                 segment, so there is nothing here to write to",
                self.name
            ));
        }
        if data.len() > self.size() {
            return Err(format!(
                "write of {} bytes exceeds the {}-byte pool",
                data.len(),
                self.size()
            ));
        }
        let pre = self.begin_write();
        unsafe {
            std::ptr::copy_nonoverlapping(data.as_ptr(), self.host_ptr() as *mut u8, data.len());
        }
        self.end_write(pre, true);
        Ok(())
    }

    /// Sample the generation before reading the payload.
    ///
    /// Spins while a write is in progress, up to a fixed budget, then returns
    /// the odd value it saw — which makes [`read_valid`](Self::read_valid)
    /// return false. A writer killed mid-write therefore cannot hang a reader.
    pub fn begin_read(&self) -> u64 {
        const SPIN_BUDGET: u32 = 10_000;
        for _ in 0..SPIN_BUDGET {
            let g = unsafe { seqlock::begin_read(self.gen_ptr()) };
            if seqlock::is_complete(g) {
                return g;
            }
            std::hint::spin_loop();
        }
        unsafe { seqlock::begin_read(self.gen_ptr()) }
    }

    /// True when the payload read after [`begin_read`](Self::begin_read) is
    /// intact: no write started or finished in between.
    ///
    /// Must use `seqlock::end_read`, not `begin_read`: the closing sample needs
    /// the acquire edge on the other side of the load, ordering the payload
    /// reads *before* it. Sampling with `begin_read` here would let a
    /// weakly-ordered CPU hoist the second generation load above the payload
    /// reads and accept a torn frame. The two functions return the same value,
    /// so no test can catch that swap — this comment is the only guard.
    pub fn read_valid(&self, generation: u64) -> bool {
        seqlock::is_complete(generation)
            && unsafe { seqlock::end_read(self.gen_ptr()) } == generation
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
    /// cascade of false ones. Unlinking from `Drop` runs on the panic path
    /// too, which a trailing `unlink()` call does not.
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

    fn create_test_segment(tag: &str, size: usize) -> (SegmentGuard, PoolSegment) {
        let guard = SegmentGuard::new(tag);
        let seg = PoolSegment::create(guard.name(), size, "uint8", &[size], Transport::Shmem)
            .expect("create segment");
        (guard, seg)
    }

    #[test]
    fn create_then_open_sees_the_same_geometry() {
        let (guard, writer) = create_test_segment("geometry", 128);
        let reader = PoolSegment::open(guard.name()).expect("open");

        assert_eq!(reader.size(), 128);
        assert_eq!(reader.dtype(), "uint8");
        assert_eq!(reader.shape(), &[128]);
        assert_eq!(reader.data_offset(), writer.data_offset());
        assert_eq!(reader.segment_bytes(), writer.segment_bytes());
        assert_eq!(reader.transport(), Transport::Shmem);
        assert_eq!(reader.pinned_type(), "cpu");
        assert!(!reader.ipc_present());
        assert!(reader.ipc_handle().is_none());
    }

    #[test]
    fn payload_written_by_the_writer_is_visible_to_the_reader() {
        let (guard, mut writer) = create_test_segment("payload", 64);
        writer.write(&[0xAB; 64]).expect("write");

        let reader = PoolSegment::open(guard.name()).expect("open");
        let seen = unsafe { std::slice::from_raw_parts(reader.host_ptr() as *const u8, 64) };
        assert!(
            seen.iter().all(|&b| b == 0xAB),
            "reader saw {seen:?}, not the written payload"
        );
    }

    /// The payload must start at `data_offset`, not at the mapping base:
    /// writing at the base would overwrite the header the reader depends on.
    #[test]
    fn the_payload_starts_after_the_header_and_leaves_it_intact() {
        let (guard, mut writer) = create_test_segment("offset", 32);
        assert_eq!(writer.host_ptr(), writer.shm_base() + 512);
        writer.write(&[0xCD; 32]).expect("write");

        let reader = PoolSegment::open(guard.name()).expect("open");
        assert_eq!(
            reader.size(),
            32,
            "the header must survive the payload write"
        );
        let seen = unsafe { std::slice::from_raw_parts(reader.host_ptr() as *const u8, 32) };
        assert!(seen.iter().all(|&b| b == 0xCD));
    }

    #[test]
    fn a_completed_write_leaves_an_even_generation_a_reader_accepts() {
        let (guard, mut writer) = create_test_segment("seqlock", 32);
        writer.write(&[1; 32]).expect("write");

        let reader = PoolSegment::open(guard.name()).expect("open");
        let generation = reader.begin_read();
        assert!(crate::seqlock::is_complete(generation));
        assert!(reader.read_valid(generation));
    }

    /// The reader must see a torn frame as invalid, not as data.
    #[test]
    fn a_write_in_progress_is_rejected_by_the_reader() {
        let (guard, mut writer) = create_test_segment("torn", 32);
        let reader = PoolSegment::open(guard.name()).expect("open");

        let generation = reader.begin_read();
        let pre = writer.begin_write();
        assert!(!reader.read_valid(generation), "mid-write must be invalid");
        writer.end_write(pre, true);
        assert!(
            !reader.read_valid(generation),
            "a new frame invalidates the old sample"
        );
    }

    /// `begin_read` gives up after its spin budget and hands back the odd
    /// value it saw. `read_valid` must reject that on parity alone — the two
    /// samples agree, so equality is not enough.
    #[test]
    fn an_odd_opening_sample_is_rejected_on_parity_even_though_both_samples_agree() {
        let (guard, mut writer) = create_test_segment("oddsample", 32);
        let reader = PoolSegment::open(guard.name()).expect("open");

        let pre = writer.begin_write();
        let generation = reader.begin_read();
        assert!(
            !crate::seqlock::is_complete(generation),
            "begin_read must return the odd value it gave up on"
        );
        assert!(
            !reader.read_valid(generation),
            "an odd sample must never be accepted"
        );
        writer.end_write(pre, true);
    }

    #[test]
    fn write_rejects_more_bytes_than_the_pool_holds() {
        let (_guard, mut writer) = create_test_segment("overflow", 16);
        let err = writer.write(&[0; 17]).unwrap_err();
        assert!(
            err.contains("17 bytes exceeds the 16-byte pool"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn create_rejects_a_zero_size_pool() {
        let guard = SegmentGuard::new("zero");
        let err =
            PoolSegment::create(guard.name(), 0, "uint8", &[0], Transport::Shmem).unwrap_err();
        assert!(err.contains("greater than zero"), "unexpected error: {err}");
    }

    /// Mirrors the daemon's own cap so a node fails locally with a clear
    /// message instead of after a round trip.
    #[test]
    fn create_rejects_a_pool_larger_than_one_gib() {
        let guard = SegmentGuard::new("huge");
        let err = PoolSegment::create(
            guard.name(),
            1024 * 1024 * 1024 + 1,
            "uint8",
            &[1],
            Transport::Shmem,
        )
        .unwrap_err();
        assert!(err.contains("1 GiB"), "unexpected error: {err}");
        // One byte under the cap is a legal request, so the message above
        // cannot be coming from an off-by-one that rejects everything.
        assert!(
            PoolSegment::create(
                guard.name(),
                1024 * 1024 * 1024,
                "uint8",
                &[1],
                Transport::Shmem
            )
            .is_ok(),
            "a pool exactly at the cap must be accepted"
        );
    }

    /// Writing `ipc` means exporting a CUDA IPC handle, which this crate
    /// never does. Accepting it would produce a segment whose header claims
    /// a handle that is all zeros.
    #[test]
    fn create_rejects_the_ipc_transport_it_cannot_export_a_handle_for() {
        let guard = SegmentGuard::new("ipccreate");
        let err =
            PoolSegment::create(guard.name(), 64, "uint8", &[64], Transport::Ipc).unwrap_err();
        assert!(err.contains("use `unified`"), "unexpected error: {err}");
        assert!(
            !std::path::Path::new(&format!("/dev/shm/{}", guard.name())).exists(),
            "a rejected transport must not leave a segment behind"
        );
    }

    #[test]
    fn unified_and_shmem_differ_only_in_the_declared_pinned_type() {
        let guard = SegmentGuard::new("unified");
        let seg =
            PoolSegment::create(guard.name(), 64, "uint8", &[64], Transport::Unified).unwrap();
        assert_eq!(seg.pinned_type(), "cuda");
        assert_eq!(seg.segment_bytes(), seg.data_offset() + 64);

        let reader = PoolSegment::open(guard.name()).expect("open");
        assert_eq!(reader.transport(), Transport::Unified);
        assert_eq!(reader.pinned_type(), "cuda");
        assert_eq!(reader.size(), 64);
        // Byte-identical on the wire apart from the two JSON strings: no IPC
        // flag, and the full data region really is allocated.
        assert!(!reader.ipc_present());
    }

    /// Build the segment shape the Python binding produces for a CUDA
    /// receiver on a discrete GPU: header-only allocation, `ipc_flag = 1`, no
    /// `transport` key, and a JSON `size` describing the tensor in the
    /// IPC-imported device buffer — far larger than the segment itself.
    fn create_python_style_ipc_segment(guard: &SegmentGuard, declared_size: usize) {
        let json = format!(
            r#"{{"size":{declared_size},"dtype":"uint8","shape":[{declared_size}],"pinned_type":"cuda"}}"#
        );
        let total = crate::doradma::data_offset_for(json.len());
        let mut shmem = ShmemConf::new()
            .os_id(guard.name())
            .size(total)
            .writable(true)
            .create()
            .expect("create ipc-style segment");
        shmem.set_owner(false);
        let buf = unsafe { std::slice::from_raw_parts_mut(shmem.as_ptr(), total) };
        write_header(buf, &json).expect("write header");
        buf[crate::doradma::OFFSET_IPC_FLAG..crate::doradma::OFFSET_IPC_FLAG + 8]
            .copy_from_slice(&1u64.to_le_bytes());
    }

    /// `open` must not run a payload-size check of its own. `parse_header`
    /// gates that check on the header's `ipc_flag`, and this is the segment
    /// shape the gate exists for: a size check applied here would reject
    /// every Python-written GPU pool.
    #[test]
    fn open_accepts_a_python_written_header_only_cuda_pool() {
        let guard = SegmentGuard::new("pyipc");
        create_python_style_ipc_segment(&guard, 8 * 1024 * 1024);

        let reader = PoolSegment::open(guard.name()).expect("a header-only ipc pool must open");
        assert_eq!(reader.transport(), Transport::Ipc);
        assert!(reader.ipc_present());
        assert!(reader.ipc_handle().is_some());
        assert_eq!(reader.size(), 8 * 1024 * 1024);
        assert!(
            reader.segment_bytes() < reader.size(),
            "the point of this segment is that its declared size exceeds it"
        );
    }

    /// The flip side of accepting that segment: `size` no longer bounds the
    /// mapping, so a write bounded by `size` would run off the end of it.
    #[test]
    fn write_is_refused_on_an_ipc_pool_whose_payload_is_not_in_this_segment() {
        let guard = SegmentGuard::new("ipcwrite");
        create_python_style_ipc_segment(&guard, 8 * 1024 * 1024);

        let mut seg = PoolSegment::open(guard.name()).expect("open");
        // Well under the declared `size`, so the byte-count check alone would
        // let this through and scribble past the end of the mapping.
        let err = seg.write(&[0xAB; 4096]).unwrap_err();
        assert!(err.contains("ipc-backed"), "unexpected error: {err}");
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
        // Still openable: only unlink() (or the daemon) removes a segment.
        let reader = PoolSegment::open(guard.name()).expect("segment must survive drop");
        reader.unlink();
        assert!(
            PoolSegment::open(guard.name()).is_err(),
            "unlink must remove it"
        );
    }
}
