#[cfg(test)]
mod tests {
    use super::*;
    use dora_node_api::arrow::array::{UInt8Array};
    use dora_node_api::arrow::datatypes::DataType;
    use some_module::{DoraEvent, Event, ffi};

    #[test]
    fn test_event_as_input_uint8() {
        let array = UInt8Array::from(vec![1, 2, 3, 4]);
        let event = Box::new(DoraEvent(Event::Input {
            id: "test_id".to_string(),
            metadata: Metadata { type_info: TypeInfo { data_type: DataType::UInt8 }},
            data: Box::new(array),
        }));

        let result = event_as_input(event);
        assert!(result.is_ok());
        let input = result.unwrap();
        assert_eq!(input.data, vec![1, 2, 3, 4]);
    }
}
