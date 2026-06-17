use dora_message::{
    arrow_data::ArrayData,
    arrow_schema::DataType,
    metadata::{ArrowTypeInfo, BufferOffset},
};

#[cfg(test)]
mod tests;

/// Why a buffer is not contained in the expected memory region.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RegionContainmentError {
    /// The buffer starts before the region.
    StartsBeforeRegion,
    /// The buffer extends past the region end.
    EndsPastRegion,
}

/// Compute the offset of a buffer `[ptr, ptr + len)` inside a memory region
/// `[region_start, region_start + region_len)`, rejecting buffers that are
/// not fully contained.
///
/// Addresses are plain integers, so this is pure arithmetic with no pointer
/// provenance involved. The comparisons are deliberately formulated with
/// guarded subtractions instead of `ptr + len` / `region_start + region_len`,
/// which would wrap for regions ending exactly at the top of the address
/// space. Two earlier off-by-one bugs lived in this logic (see git history
/// of `from_array`), so it is kept as a pure function with exhaustive Kani
/// proofs (`verification` module below) covering soundness, completeness,
/// and panic-freedom.
fn buffer_offset_in_region(
    ptr: usize,
    len: usize,
    region_start: usize,
    region_len: usize,
) -> Result<usize, RegionContainmentError> {
    if ptr < region_start {
        return Err(RegionContainmentError::StartsBeforeRegion);
    }
    let offset = ptr - region_start;
    if offset > region_len || len > region_len - offset {
        return Err(RegionContainmentError::EndsPastRegion);
    }
    Ok(offset)
}

pub trait ArrowTypeInfoExt {
    fn empty() -> Self;
    fn byte_array(data_len: usize) -> Self;

    /// # Safety
    ///
    /// This function assumes that the `ArrayData` is backed by a memory region that starts at `region_start`
    /// and has a length of `region_len`. It returns an error if the `ArrayData` does not conform to this
    /// assumption (the returned offsets would be meaningless for buffers outside the region).
    unsafe fn from_array(
        array: &ArrayData,
        region_start: *const u8,
        region_len: usize,
    ) -> eyre::Result<Self>
    where
        Self: Sized;
}

impl ArrowTypeInfoExt for ArrowTypeInfo {
    fn empty() -> Self {
        Self {
            data_type: DataType::Null,
            len: 0,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: Vec::new(),
            child_data: Vec::new(),
            field_names: None,
            schema_hash: None,
        }
    }

    fn byte_array(data_len: usize) -> Self {
        Self {
            data_type: DataType::UInt8,
            len: data_len,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: vec![BufferOffset {
                offset: 0,
                len: data_len,
            }],
            child_data: Vec::new(),
            field_names: None,
            schema_hash: None,
        }
    }

    unsafe fn from_array(
        array: &ArrayData,
        region_start: *const u8,
        region_len: usize,
    ) -> eyre::Result<Self> {
        Ok(Self {
            data_type: array.data_type().clone(),
            len: array.len(),
            null_count: array.null_count(),
            validity: array.nulls().map(|b| b.validity().to_owned()),
            offset: array.offset(),
            buffer_offsets: array
                .buffers()
                .iter()
                .map(|b| {
                    let ptr = b.as_ptr();
                    // The containment invariant — [ptr, ptr+len) ⊆
                    // [region_start, region_start+region_len) — is checked by
                    // a pure helper so it can be exhaustively verified with
                    // Kani (two off-by-one bugs lived here previously).
                    // Integer subtraction replaces the former
                    // `ptr.offset_from(region_start)` call, which required
                    // both pointers to share an allocation — a provenance
                    // precondition this function cannot locally guarantee.
                    let offset = buffer_offset_in_region(
                        ptr as usize,
                        b.len(),
                        region_start as usize,
                        region_len,
                    )
                    .map_err(|err| match err {
                        RegionContainmentError::StartsBeforeRegion => eyre::eyre!(
                            "buffer ptr {ptr:p} is before region start {region_start:p}"
                        ),
                        RegionContainmentError::EndsPastRegion => eyre::eyre!(
                            "buffer ptr {ptr:p} + len {} extends past region end \
                             {region_start:p} + {region_len}",
                            b.len(),
                        ),
                    })?;

                    Result::<_, eyre::Report>::Ok(BufferOffset {
                        offset,
                        len: b.len(),
                    })
                })
                .collect::<Result<_, _>>()?,
            child_data: array
                .child_data()
                .iter()
                .map(|c| unsafe { Self::from_array(c, region_start, region_len) })
                .collect::<Result<_, _>>()?,
            field_names: None,
            schema_hash: None,
        })
    }
}

/// Kani proof harnesses (`make qa-kani`). Compiled only under `cargo kani`,
/// never in normal builds or tests. See `docs/formal-verification.md`.
///
/// These proofs are exhaustive over the full `usize` input space (the
/// helper is loop-free pure arithmetic, so no bounding is needed). The
/// containment predicate is stated with guarded subtractions so it stays
/// meaningful even for regions ending exactly at the top of the address
/// space, where `region_start + region_len` would wrap.
#[cfg(kani)]
mod verification {
    use super::{RegionContainmentError, buffer_offset_in_region};

    /// Soundness + panic-freedom: for *any* input, the helper never panics,
    /// and whenever it accepts a buffer, the buffer really is contained in
    /// the region and the returned offset is exact.
    #[kani::proof]
    fn accepted_buffers_are_contained() {
        let ptr: usize = kani::any();
        let len: usize = kani::any();
        let region_start: usize = kani::any();
        let region_len: usize = kani::any();

        if let Ok(offset) = buffer_offset_in_region(ptr, len, region_start, region_len) {
            // The buffer starts inside the region, at the reported offset.
            assert!(region_start <= ptr);
            assert_eq!(offset, ptr - region_start);
            // The buffer ends inside the region (subtraction cannot
            // underflow because offset <= region_len is implied).
            assert!(offset <= region_len);
            assert!(len <= region_len - offset);
        }
    }

    /// Completeness: every contained buffer is accepted — the checks
    /// reject nothing valid (this is the property the two historical
    /// off-by-one bugs violated). Unconditional: covers buffers and
    /// regions ending exactly at the top of the address space.
    #[kani::proof]
    fn contained_buffers_are_accepted() {
        let ptr: usize = kani::any();
        let len: usize = kani::any();
        let region_start: usize = kani::any();
        let region_len: usize = kani::any();

        kani::assume(ptr >= region_start);
        let offset = ptr - region_start;
        kani::assume(offset <= region_len);
        kani::assume(len <= region_len - offset);

        assert_eq!(
            buffer_offset_in_region(ptr, len, region_start, region_len),
            Ok(offset)
        );
    }

    /// Rejection reasons are accurate: a buffer starting before the region
    /// is reported as such, never silently accepted with a wrapped offset.
    #[kani::proof]
    fn buffers_before_region_are_rejected() {
        let ptr: usize = kani::any();
        let len: usize = kani::any();
        let region_start: usize = kani::any();
        let region_len: usize = kani::any();
        kani::assume(ptr < region_start);

        assert_eq!(
            buffer_offset_in_region(ptr, len, region_start, region_len),
            Err(RegionContainmentError::StartsBeforeRegion)
        );
    }
}
