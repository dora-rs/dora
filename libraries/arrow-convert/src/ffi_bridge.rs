//! Zero-copy bridge between dora's internal Arrow major (59) and Arrow 58.
//!
//! Enabled by the `arrow-v58` feature. This is the worked example of
//! cross-major support described in `docs/plan-arrow-version-decoupling.md`;
//! adding further majors means copying this module and swapping the alias.
//!
//! # How it works
//!
//! Both crates implement the [Arrow C Data Interface][spec]: `to_ffi` exports
//! an `ArrayData` as a `(FFI_ArrowArray, FFI_ArrowSchema)` pair, `from_ffi`
//! imports one. Neither call copies buffers — the exported struct holds raw
//! pointers into the original allocation plus a `release` callback and a
//! `private_data` pointer that keeps that allocation alive.
//!
//! [spec]: https://arrow.apache.org/docs/format/CDataInterface.html
//!
//! # The unsafe part, and why it is sound
//!
//! `arrow58::ffi::FFI_ArrowArray` and `arrow::ffi::FFI_ArrowArray` are
//! *distinct Rust types* to rustc, so handing one to the other's `from_ffi`
//! requires a reinterpret. The safety argument is:
//!
//! 1. **Both types are `#[repr(C)]` definitions of the same C struct.** The C
//!    Data Interface fixes the field order, types, and meaning of
//!    `ArrowArray` and `ArrowSchema`. arrow-rs defines `FFI_ArrowArray` /
//!    `FFI_ArrowSchema` as `#[repr(C)]` structs mirroring that layout — that
//!    is their entire purpose, and it is what lets arrow-rs exchange arrays
//!    with pyarrow, DuckDB, and every other implementation.
//! 2. **The spec is frozen.** `ArrowArray`/`ArrowSchema` have not changed
//!    since Arrow 1.0 and the format explicitly guarantees they will not; a
//!    layout change would break every non-Rust consumer simultaneously. So
//!    "the two Rust definitions agree" is not a coincidence to be re-checked
//!    each release, it is the interoperability contract both crates advertise.
//! 3. **We assert what we can mechanically assert.** The `const` blocks below
//!    reject a size or alignment mismatch at *compile* time. That does not
//!    prove field-for-field identity, but it catches the realistic failure
//!    mode (a field added, removed, or retyped) rather than leaving it to
//!    runtime.
//! 4. **Ownership transfers exactly once.** `to_ffi` returns owned structs;
//!    `std::mem::transmute` moves them (it does not copy — the source is
//!    consumed and never dropped by the exporting crate), and `from_ffi`
//!    takes the `FFI_ArrowArray` by value and becomes responsible for calling
//!    `release`. There is no point at which both crates believe they own the
//!    same struct, so no double-free.
//! 5. **Cross-crate `release` is the designed behaviour.** The importing
//!    crate calls the `release` function pointer the *exporting* crate stored
//!    in the struct. That pointer refers to code in the exporting crate,
//!    which is still linked into the same binary, and it frees the exporting
//!    crate's `private_data` with the exporting crate's allocator. This is
//!    exactly the arrangement the C Data Interface exists to support (it is
//!    how arrow-rs frees memory pyarrow allocated), and the
//!    `release_callback_*` tests below exercise it directly.
//!
//! What is *not* covered: if a future arrow-rs release changed
//! `FFI_ArrowArray`'s layout while keeping its size and alignment, the
//! compile-time asserts would not catch it. That would also be a breaking
//! change to arrow-rs's own C interop, so it is treated as out of scope in
//! the same way arrow-rs treats it.

use eyre::{Context, Result};

use crate::DoraArray;

// Layout guards. A mismatch here means the reinterprets below are unsound, so
// fail the build rather than the process.
const _: () = assert!(
    size_of::<arrow58::ffi::FFI_ArrowArray>() == size_of::<arrow::ffi::FFI_ArrowArray>(),
    "FFI_ArrowArray size differs between Arrow 58 and Arrow 59"
);
const _: () = assert!(
    align_of::<arrow58::ffi::FFI_ArrowArray>() == align_of::<arrow::ffi::FFI_ArrowArray>(),
    "FFI_ArrowArray alignment differs between Arrow 58 and Arrow 59"
);
const _: () = assert!(
    size_of::<arrow58::ffi::FFI_ArrowSchema>() == size_of::<arrow::ffi::FFI_ArrowSchema>(),
    "FFI_ArrowSchema size differs between Arrow 58 and Arrow 59"
);
const _: () = assert!(
    align_of::<arrow58::ffi::FFI_ArrowSchema>() == align_of::<arrow::ffi::FFI_ArrowSchema>(),
    "FFI_ArrowSchema alignment differs between Arrow 58 and Arrow 59"
);

/// Import an Arrow 58 array into dora's internal Arrow major.
///
/// Zero-copy: the resulting array points at the same buffers, and keeps the
/// Arrow 58 allocation alive through the C interface's `release` callback.
fn v58_to_internal(array: &dyn arrow58::array::Array) -> Result<DoraArray> {
    let (ffi_array, ffi_schema) = arrow58::ffi::to_ffi(&array.to_data())
        .context("failed to export Arrow 58 array over FFI")?;

    // SAFETY: see the module-level safety argument. Both structs are the same
    // `#[repr(C)]` C Data Interface types, size- and align-checked above; the
    // move transfers ownership (including the duty to call `release`) exactly
    // once, from the Arrow 58 side to the Arrow 59 side.
    let ffi_array: arrow::ffi::FFI_ArrowArray = unsafe { std::mem::transmute(ffi_array) };
    // SAFETY: as above, for the schema struct.
    let ffi_schema: arrow::ffi::FFI_ArrowSchema = unsafe { std::mem::transmute(ffi_schema) };

    // SAFETY: `ffi_array` and `ffi_schema` were produced together by
    // `arrow58::ffi::to_ffi` and describe the same array, which is `from_ffi`'s
    // requirement. Ownership of `ffi_array` moves into `from_ffi`.
    let data = unsafe { arrow::ffi::from_ffi(ffi_array, &ffi_schema) }
        .context("failed to import array")?;
    Ok(crate::internal::from_array_data(data))
}

/// Export a dora payload as an Arrow 58 array.
///
/// Zero-copy, and the mirror image of `v58_to_internal`: the Arrow 58 array
/// keeps dora's buffers alive through the `release` callback dora's Arrow
/// installed.
fn internal_to_v58(data: &DoraArray) -> Result<arrow58::array::ArrayRef> {
    let (ffi_array, ffi_schema) = arrow::ffi::to_ffi(&crate::internal::array_ref(data).to_data())
        .context("failed to export dora payload over FFI")?;

    // SAFETY: see the module-level safety argument; this is the same
    // reinterpret as `v58_to_internal`, with the two majors swapped.
    let ffi_array: arrow58::ffi::FFI_ArrowArray = unsafe { std::mem::transmute(ffi_array) };
    // SAFETY: as above, for the schema struct.
    let ffi_schema: arrow58::ffi::FFI_ArrowSchema = unsafe { std::mem::transmute(ffi_schema) };

    // SAFETY: the pair was produced together by `arrow::ffi::to_ffi` and
    // describes one array; ownership of `ffi_array` moves into `from_ffi`.
    let imported = unsafe { arrow58::ffi::from_ffi(ffi_array, &ffi_schema) }
        .context("failed to import array into Arrow 58")?;
    Ok(arrow58::array::make_array(imported))
}

/// Import an Arrow 58 array as a dora payload.
///
/// ```ignore
/// use arrow58::array::Array;
///
/// let dora = DoraArray::try_from(&concrete_arrow58_array as &dyn Array)?;
/// ```
///
/// The conversion is **fallible**, so this is `TryFrom` and not `From`: the C
/// Data Interface export/import can reject an array (arrow-rs returns an error
/// for the layouts it cannot represent over the interface), and an infallible
/// `From` would have to panic on those.
///
/// It is a *converting* conversion, not a borrowing one: Arrow 58 is not
/// dora's internal major, so reaching it costs a C Data Interface hop. The hop
/// does not copy buffers, but it does allocate the small FFI structs and a new
/// array handle.
///
/// The source type is `&dyn Array` rather than a generic `&A: Array` because
/// coherence forbids the generic form: `impl<A: Array> TryFrom<&A> for
/// DoraArray` collides with core's `impl<T, U: Into<T>> TryFrom<U> for T`,
/// since a downstream crate may add `impl From<&TheirType> for DoraArray`
/// (RFC 2451 permits `impl ForeignTrait<LocalType> for ForeignType`), and `A`
/// could be instantiated at `TheirType`. `&dyn Array` is a concrete foreign
/// type, so no downstream impl can exist and the overlap goes away. The
/// `TryFrom<&ArrayRef>` impl below covers the other common source shape.
impl TryFrom<&dyn arrow58::array::Array> for DoraArray {
    type Error = eyre::Report;

    fn try_from(array: &dyn arrow58::array::Array) -> Result<Self> {
        v58_to_internal(array)
    }
}

/// Import an Arrow 58 [`ArrayRef`](arrow58::array::ArrayRef) as a dora payload.
///
/// ```ignore
/// let dora: DoraArray = (&arrow58_array_ref).try_into()?;
/// ```
///
/// Same conversion as the `&dyn Array` impl above; provided separately because
/// `&Arc<dyn Array>` does not unsize-coerce to `&dyn Array` during trait
/// selection, so `(&array_ref).try_into()` would otherwise not resolve.
impl TryFrom<&arrow58::array::ArrayRef> for DoraArray {
    type Error = eyre::Report;

    fn try_from(array: &arrow58::array::ArrayRef) -> Result<Self> {
        v58_to_internal(array.as_ref())
    }
}

/// Export a dora payload as an Arrow 58 array.
///
/// ```ignore
/// let arrow58_array: arrow58::array::ArrayRef = (&dora).try_into()?;
/// ```
///
/// Fallible for the same reason as the import direction above, and likewise
/// zero-copy: the Arrow 58 array keeps dora's buffers alive through the
/// `release` callback dora's Arrow installed.
impl TryFrom<&DoraArray> for arrow58::array::ArrayRef {
    type Error = eyre::Report;

    fn try_from(data: &DoraArray) -> Result<Self> {
        internal_to_v58(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow58::array::{Array, ArrayRef};

    /// 58 → 59 → 58. The values must survive both hops, and the schema with
    /// them: a mis-set field in the reinterpreted `FFI_ArrowSchema` shows up
    /// here as a changed data type or a decode error.
    #[test]
    fn round_trip_58_59_58() {
        let original = arrow58::array::UInt64Array::from(vec![1u64, 2, 3, 4, 5]);

        let dora = DoraArray::try_from(&original as &dyn Array).expect("58 -> 59");
        assert_eq!(dora.len(), 5);
        assert_eq!(dora.type_name(), "UInt64");
        let values: Vec<u64> = crate::into_vec(&dora).expect("read back");
        assert_eq!(values, vec![1, 2, 3, 4, 5]);

        let back = ArrayRef::try_from(&dora).expect("59 -> 58");
        let back = back
            .as_any()
            .downcast_ref::<arrow58::array::UInt64Array>()
            .expect("still a UInt64Array after the round trip");
        assert_eq!(back.values(), original.values());
        assert_eq!(back.data_type(), original.data_type());
    }

    /// Nulls, strings, and nested types exercise more of the C interface than
    /// a flat primitive: a string array carries three buffers, a struct array
    /// carries children, and a validity bitmap adds a null buffer.
    #[test]
    fn round_trip_58_59_58_nested_and_nullable() {
        use std::sync::Arc;

        let strings = arrow58::array::StringArray::from(vec![Some("a"), None, Some("ccc")]);
        let ints = arrow58::array::Int32Array::from(vec![Some(1), Some(2), None]);
        let original = arrow58::array::StructArray::from(vec![
            (
                Arc::new(arrow58::datatypes::Field::new(
                    "s",
                    arrow58::datatypes::DataType::Utf8,
                    true,
                )),
                Arc::new(strings) as arrow58::array::ArrayRef,
            ),
            (
                Arc::new(arrow58::datatypes::Field::new(
                    "i",
                    arrow58::datatypes::DataType::Int32,
                    true,
                )),
                Arc::new(ints) as arrow58::array::ArrayRef,
            ),
        ]);

        let dora = DoraArray::try_from(&original as &dyn Array).expect("58 -> 59");
        assert_eq!(dora.len(), 3);

        let back = ArrayRef::try_from(&dora).expect("59 -> 58");
        let back = back
            .as_any()
            .downcast_ref::<arrow58::array::StructArray>()
            .expect("still a StructArray");
        assert_eq!(back.len(), 3);
        assert_eq!(back.num_columns(), 2);
        assert_eq!(back.column(0).null_count(), 1);
        assert_eq!(back.column(1).null_count(), 1);
        assert_eq!(back.to_data(), original.to_data());
    }

    /// The release-callback path, which is where a layout mistake becomes a
    /// use-after-free rather than a wrong answer.
    ///
    /// The imported Arrow 59 array is dropped **without ever being read**, so
    /// the only thing that runs is Arrow 59 invoking the `release` callback
    /// Arrow 58 installed. `flag` is owned by the Arrow 58 array's buffer
    /// allocation, so it stays alive until that release actually happens, and
    /// flips exactly then. A double-free would abort the process here; a
    /// missing release would leave the flag unset.
    #[test]
    fn release_callback_runs_when_import_is_dropped_unread() {
        use std::sync::Arc;
        use std::sync::atomic::{AtomicUsize, Ordering};

        struct DropCounter(Arc<AtomicUsize>);
        impl Drop for DropCounter {
            fn drop(&mut self) {
                self.0.fetch_add(1, Ordering::SeqCst);
            }
        }

        let releases = Arc::new(AtomicUsize::new(0));

        let dora = {
            // A buffer whose backing allocation carries `DropCounter`, so the
            // counter fires precisely when Arrow 58's allocation is freed.
            let bytes: Vec<u8> = (0..64u8).collect();
            let owner = Arc::new((bytes.clone(), DropCounter(releases.clone())));
            let ptr = std::ptr::NonNull::new(owner.0.as_ptr() as *mut u8).unwrap();
            // SAFETY: `ptr`/`len` describe `owner.0`'s allocation, which the
            // `Arc` passed as the owner keeps alive for the buffer's lifetime.
            let buffer =
                unsafe { arrow58::buffer::Buffer::from_custom_allocation(ptr, bytes.len(), owner) };
            let array = arrow58::array::UInt8Array::from(
                arrow58::array::ArrayData::builder(arrow58::datatypes::DataType::UInt8)
                    .len(64)
                    .add_buffer(buffer)
                    .build()
                    .unwrap(),
            );

            let dora = DoraArray::try_from(&array as &dyn Array).expect("58 -> 59");
            // Drop every Arrow 58 handle. The allocation must survive, because
            // the imported Arrow 59 array still references it through the C
            // interface.
            drop(array);
            assert_eq!(
                releases.load(Ordering::SeqCst),
                0,
                "Arrow 58 freed the buffer while the Arrow 59 import still held it"
            );
            dora
        };

        // Never read `dora`; just drop it. Arrow 59 must now call the release
        // callback Arrow 58 installed.
        drop(dora);
        assert_eq!(
            releases.load(Ordering::SeqCst),
            1,
            "dropping the imported array must run Arrow 58's release callback exactly once"
        );
    }

    /// The same check for the other direction: Arrow 58 releasing an
    /// allocation Arrow 59 owns, again without the array ever being read.
    #[test]
    fn release_callback_runs_for_export_dropped_unread() {
        use std::sync::Arc;
        use std::sync::atomic::{AtomicUsize, Ordering};

        struct DropCounter(Arc<AtomicUsize>);
        impl Drop for DropCounter {
            fn drop(&mut self) {
                self.0.fetch_add(1, Ordering::SeqCst);
            }
        }

        let releases = Arc::new(AtomicUsize::new(0));

        let exported = {
            let bytes: Vec<u8> = (0..64u8).collect();
            let owner = Arc::new((bytes.clone(), DropCounter(releases.clone())));
            let ptr = std::ptr::NonNull::new(owner.0.as_ptr() as *mut u8).unwrap();
            // SAFETY: as in the test above — the `Arc` owner keeps the
            // allocation alive for the buffer's lifetime.
            let buffer =
                unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, bytes.len(), owner) };
            let data = arrow::array::ArrayData::builder(arrow::datatypes::DataType::UInt8)
                .len(64)
                .add_buffer(buffer)
                .build()
                .unwrap();
            let dora = crate::internal::from_array_data(data);

            let exported = ArrayRef::try_from(&dora).expect("59 -> 58");
            drop(dora);
            assert_eq!(
                releases.load(Ordering::SeqCst),
                0,
                "Arrow 59 freed the buffer while the Arrow 58 export still held it"
            );
            exported
        };

        drop(exported);
        assert_eq!(
            releases.load(Ordering::SeqCst),
            1,
            "dropping the exported array must run Arrow 59's release callback exactly once"
        );
    }

    /// An empty array still has to carry a valid schema across the hop; a
    /// zero-length import used to be the easiest way to trip a null-pointer
    /// assumption in FFI glue.
    #[test]
    fn round_trip_empty_array_preserves_type() {
        // Also the one place the `&ArrayRef` source impl is exercised, so both
        // import shapes stay covered.
        let original: ArrayRef =
            std::sync::Arc::new(arrow58::array::Float32Array::from(Vec::<f32>::new()));
        let dora: DoraArray = (&original).try_into().expect("58 -> 59");
        assert_eq!(dora.len(), 0);
        assert_eq!(dora.type_name(), "Float32");
        let back: ArrayRef = (&dora).try_into().expect("59 -> 58");
        assert_eq!(back.data_type(), &arrow58::datatypes::DataType::Float32);
        assert_eq!(back.len(), 0);
    }
}
