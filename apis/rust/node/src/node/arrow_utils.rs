use arrow::array::ArrayData;
use dora_core::message::{ArrowTypeInfo, BufferOffset};

pub fn required_data_size(array: &ArrayData) -> usize {
    let mut size = 0;
    for buffer in array.buffers() {
        size += buffer.len();
    }
    for child in array.child_data() {
        size += required_data_size(child);
    }
    size
}

pub fn copy_array_into_sample(
    target_buffer: &mut [u8],
    arrow_array: &ArrayData,
) -> eyre::Result<ArrowTypeInfo> {
    let mut next_offset = 0;
    copy_array_into_sample_inner(target_buffer, &mut next_offset, arrow_array)
}

fn copy_array_into_sample_inner(
    target_buffer: &mut [u8],
    next_offset: &mut usize,
    arrow_array: &ArrayData,
) -> eyre::Result<ArrowTypeInfo> {
    let mut buffer_offsets = Vec::new();
    for buffer in arrow_array.buffers().iter() {
        let len = buffer.len();
        assert!(
            target_buffer[*next_offset..].len() >= len,
            "target buffer too small (total_len: {}, offset: {}, required_len: {len})",
            target_buffer.len(),
            *next_offset,
        );
        target_buffer[*next_offset..][..len].copy_from_slice(buffer.as_slice());
        buffer_offsets.push(BufferOffset {
            offset: *next_offset,
            len,
        });
        *next_offset += len;
    }

    let mut child_data = Vec::new();
    for child in arrow_array.child_data() {
        let child_type_info = copy_array_into_sample_inner(target_buffer, next_offset, child)?;
        child_data.push(child_type_info);
    }

    Ok(ArrowTypeInfo {
        data_type: arrow_array.data_type().clone(),
        len: arrow_array.len(),
        null_count: arrow_array.null_count(),
        validity: arrow_array.nulls().map(|b| b.validity().to_owned()),
        offset: arrow_array.offset(),
        buffer_offsets,
        child_data,
    })
}
