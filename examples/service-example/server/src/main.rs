use adora_node_api::{
    AdoraNode, Event,
    arrow::array::{Array, Int64Array, StructArray},
};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id: _,
                metadata,
                data,
            } => {
                let struct_array = StructArray::from(data.to_data());

                let a = struct_array
                    .column_by_name("a")
                    .and_then(|c| c.as_any().downcast_ref::<Int64Array>())
                    .map(|arr| arr.value(0))
                    .ok_or_else(|| eyre::eyre!("missing field 'a' in request"))?;
                let b = struct_array
                    .column_by_name("b")
                    .and_then(|c| c.as_any().downcast_ref::<Int64Array>())
                    .map(|arr| arr.value(0))
                    .ok_or_else(|| eyre::eyre!("missing field 'b' in request"))?;

                let sum = a + b;
                println!("[server] {a} + {b} = {sum}");

                let sum_array = Int64Array::from(vec![sum]);
                let fields = vec![arrow::datatypes::Field::new(
                    "sum",
                    arrow::datatypes::DataType::Int64,
                    false,
                )];
                let result_array = StructArray::try_new(
                    fields.into(),
                    vec![std::sync::Arc::new(sum_array) as _],
                    None,
                )
                .context("failed to create result struct")?;

                // Pass through metadata.parameters (includes request_id)
                node.send_service_response("response".into(), metadata.parameters, result_array)
                    .context("failed to send response")?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}
