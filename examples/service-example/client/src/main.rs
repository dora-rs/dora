use std::collections::HashMap;

use adora_node_api::{
    AdoraNode, Event, MetadataParameters, REQUEST_ID,
    arrow::array::{Array, Int64Array, StructArray},
    get_string_param,
};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;

    let node_id = std::env::var("ADORA_NODE_ID").unwrap_or_else(|_| "client".into());
    let mut in_flight: HashMap<String, (i64, i64)> = HashMap::new();
    let mut tick_count: i64 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "tick" => {
                    let a = tick_count;
                    let b = tick_count + 10;
                    tick_count += 1;

                    let a_array = Int64Array::from(vec![a]);
                    let b_array = Int64Array::from(vec![b]);
                    let fields = vec![
                        arrow::datatypes::Field::new("a", arrow::datatypes::DataType::Int64, false),
                        arrow::datatypes::Field::new("b", arrow::datatypes::DataType::Int64, false),
                    ];
                    let struct_array = StructArray::try_new(
                        fields.into(),
                        vec![
                            std::sync::Arc::new(a_array) as _,
                            std::sync::Arc::new(b_array) as _,
                        ],
                        None,
                    )
                    .context("failed to create struct array")?;

                    let request_id = node
                        .send_service_request(
                            "request".into(),
                            MetadataParameters::default(),
                            struct_array,
                        )
                        .context("failed to send request")?;

                    println!("[{node_id}] sent request {request_id}: {a} + {b}");
                    in_flight.insert(request_id, (a, b));
                }
                "response" => {
                    let rid = get_string_param(&metadata.parameters, REQUEST_ID);

                    if let Some(rid) = rid {
                        let struct_array = StructArray::from(data.to_data());
                        let sum_col = struct_array
                            .column_by_name("sum")
                            .and_then(|c| c.as_any().downcast_ref::<Int64Array>())
                            .map(|a| a.value(0));

                        if let (Some((a, b)), Some(sum)) = (in_flight.remove(rid), sum_col) {
                            println!("[{node_id}] response {rid}: {a} + {b} = {sum}");
                            assert_eq!(sum, a + b, "sum mismatch");
                        }
                    }
                }
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}
