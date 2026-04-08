use adora_message::{
    arrow_data::ArrayData,
    arrow_schema::DataType,
    metadata::{ArrowTypeInfo, BufferOffset},
};
use eyre::Context;

#[cfg(test)]
mod tests;

pub trait ArrowTypeInfoExt {
    fn empty() -> Self;
    fn byte_array(data_len: usize) -> Self;

    /// # Safety
    ///
    /// This function assumes that the `ArrayData` is backed by a memory region that starts at `region_start`
    /// and has a length of `region_len`. It will panic if the `ArrayData` does not conform to this assumption.
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
                    let region_end = region_start as usize + region_len;
                    // Two off-by-one bugs were here previously: `<=` rejected
                    // ptr == region_start (valid) and `>=` rejected empty
                    // buffers at the region's end. The correct invariant is
                    // simply: [ptr, ptr+len) is contained in [region_start,
                    // region_end). Verified by tests in metadata/tests.rs.
                    if (ptr as usize) < region_start as usize {
                        eyre::bail!("buffer ptr {ptr:p} is before region start {region_start:p}");
                    }
                    if ptr as usize + b.len() > region_end {
                        eyre::bail!(
                            "buffer ptr {ptr:p} + len {} extends past region end \
                             {region_start:p} + {region_len}",
                            b.len(),
                        );
                    }
                    let offset = usize::try_from(unsafe { ptr.offset_from(region_start) })
                        .context("offset_from is negative")?;

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
