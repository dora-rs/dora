//! Utility functions for converting Arrow arrays to/from raw data.
//!
use arrow::array::{ArrayData, BufferSpec};
use dora_message::metadata::{ArrowTypeInfo, BufferOffset};
use eyre::Context;

/// Calculates the data size in bytes required for storing a continuous copy of the given Arrow
/// array.
pub fn required_data_size(array: &ArrayData) -> usize {
    let mut next_offset = 0;
    required_data_size_inner(array, &mut next_offset);
    next_offset
}
fn required_data_size_inner(array: &ArrayData, next_offset: &mut usize) {
    let layout = arrow::array::layout(array.data_type());
    for (buffer, spec) in array.buffers().iter().zip(&layout.buffers) {
        // consider alignment padding
        if let BufferSpec::FixedWidth { alignment, .. } = *spec {
            *next_offset = (*next_offset).div_ceil(alignment) * alignment;
        }
        *next_offset += buffer.len();
    }
    for child in array.child_data() {
        required_data_size_inner(child, next_offset);
    }
}

/// Copy the given Arrow array into the provided buffer.
///
/// If the Arrow array consists of multiple buffers, they are placed continuously in the target
/// buffer (there might be some padding for alignment)
///
/// Panics if the buffer is not large enough.
pub fn copy_array_into_sample(target_buffer: &mut [u8], arrow_array: &ArrayData) -> ArrowTypeInfo {
    let mut next_offset = 0;
    copy_array_into_sample_inner(target_buffer, &mut next_offset, arrow_array)
}

fn copy_array_into_sample_inner(
    target_buffer: &mut [u8],
    next_offset: &mut usize,
    arrow_array: &ArrayData,
) -> ArrowTypeInfo {
    let mut buffer_offsets = Vec::new();
    let layout = arrow::array::layout(arrow_array.data_type());
    for (buffer, spec) in arrow_array.buffers().iter().zip(&layout.buffers) {
        let len = buffer.len();
        assert!(
            target_buffer[*next_offset..].len() >= len,
            "target buffer too small (total_len: {}, offset: {}, required_len: {len})",
            target_buffer.len(),
            *next_offset,
        );
        // add alignment padding
        if let BufferSpec::FixedWidth { alignment, .. } = *spec {
            *next_offset = (*next_offset).div_ceil(alignment) * alignment;
        }

        target_buffer[*next_offset..][..len].copy_from_slice(buffer.as_slice());
        buffer_offsets.push(BufferOffset {
            offset: *next_offset,
            len,
        });
        *next_offset += len;
    }

    let mut child_data = Vec::new();
    for child in arrow_array.child_data() {
        let child_type_info = copy_array_into_sample_inner(target_buffer, next_offset, child);
        child_data.push(child_type_info);
    }

    ArrowTypeInfo {
        data_type: arrow_array.data_type().clone(),
        len: arrow_array.len(),
        null_count: arrow_array.null_count(),
        validity: arrow_array.nulls().map(|b| b.validity().to_owned()),
        offset: arrow_array.offset(),
        buffer_offsets,
        child_data,
    }
}

/// Tries to convert the given raw Arrow buffer into an Arrow array.
///
/// The `type_info` is required for decoding the `raw_buffer` correctly.
pub fn buffer_into_arrow_array(
    raw_buffer: &arrow::buffer::Buffer,
    type_info: &ArrowTypeInfo,
) -> eyre::Result<arrow::array::ArrayData> {
    if raw_buffer.is_empty() {
        return Ok(arrow::array::ArrayData::new_empty(&type_info.data_type));
    }

    let mut buffers = Vec::new();
    for BufferOffset { offset, len } in &type_info.buffer_offsets {
        buffers.push(raw_buffer.slice_with_length(*offset, *len));
    }

    let mut child_data = Vec::new();
    for child_type_info in &type_info.child_data {
        child_data.push(buffer_into_arrow_array(raw_buffer, child_type_info)?)
    }

    arrow::array::ArrayData::try_new(
        type_info.data_type.clone(),
        type_info.len,
        type_info
            .validity
            .clone()
            .map(arrow::buffer::Buffer::from_vec),
        type_info.offset,
        buffers,
        child_data,
    )
    .context("Error creating Arrow array")
}
