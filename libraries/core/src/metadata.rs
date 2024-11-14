use dora_message::{
    arrow_data::ArrayData,
    arrow_schema::DataType,
    metadata::{ArrowTypeInfo, BufferOffset},
};
use eyre::Context;

pub trait ArrowTypeInfoExt {
    fn empty() -> Self;
    fn byte_array(data_len: usize) -> Self;
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
                    if ptr as usize <= region_start as usize {
                        eyre::bail!("ptr {ptr:p} starts before region {region_start:p}");
                    }
                    if ptr as usize >= region_start as usize + region_len {
                        eyre::bail!("ptr {ptr:p} starts after region {region_start:p}");
                    }
                    if ptr as usize + b.len() > region_start as usize + region_len {
                        eyre::bail!("ptr {ptr:p} ends after region {region_start:p}");
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
        })
    }
}
