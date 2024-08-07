use arrow::array::{ArrayData, BufferSpec};
use dora_message::metadata::{ArrowTypeInfo, BufferOffset};

pub fn required_data_size(array: &ArrayData) -> usize {
    let mut next_offset = 0;
    required_data_size_inner(array, &mut next_offset);
    next_offset
}
fn required_data_size_inner(array: &ArrayData, next_offset: &mut usize) {
    let layout = arrow::array::layout(array.data_type());
    for (buffer, spec) in array.buffers().iter().zip(&layout.buffers) {
        // consider alignment padding
        if let BufferSpec::FixedWidth { alignment, .. } = spec {
            *next_offset = (*next_offset + alignment - 1) / alignment * alignment;
        }
        *next_offset += buffer.len();
    }
    for child in array.child_data() {
        required_data_size_inner(child, next_offset);
    }
}

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
        if let BufferSpec::FixedWidth { alignment, .. } = spec {
            *next_offset = (*next_offset + alignment - 1) / alignment * alignment;
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
