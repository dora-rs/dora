//! Service handler node for the YAML bridge service example.
//!
//! Receives `example_interfaces/AddTwoInts` requests from the bridge,
//! computes the sum, and sends the response back through the bridge.
//! No ROS2 dependencies needed -- the bridge handles all ROS2 communication.

use std::sync::Arc;

use adora_node_api::{self, AdoraNode, Event, adora_core::config::DataId};
use arrow::array::{Array, Int64Array, StructArray};
use arrow::datatypes::{DataType, Field};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;
    let output = DataId::from("response".to_owned());

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "request" => {
                    // Parse Arrow struct matching AddTwoInts_Request: {a: i64, b: i64}
                    let struct_array = data
                        .as_any()
                        .downcast_ref::<StructArray>()
                        .expect("expected struct array for request");
                    if struct_array.len() == 0 {
                        eprintln!("Warning: received empty request array");
                        continue;
                    }
                    let a = struct_array
                        .column_by_name("a")
                        .expect("missing 'a' field")
                        .as_any()
                        .downcast_ref::<Int64Array>()
                        .expect("expected Int64Array for 'a'")
                        .value(0);
                    let b = struct_array
                        .column_by_name("b")
                        .expect("missing 'b' field")
                        .as_any()
                        .downcast_ref::<Int64Array>()
                        .expect("expected Int64Array for 'b'")
                        .value(0);

                    let sum = a + b;
                    println!("Request: {a} + {b} = {sum}");

                    // Build Arrow struct matching AddTwoInts_Response: {sum: i64}
                    let response = make_add_response(sum);
                    node.send_output(output.clone(), metadata.parameters, response)?;
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            Event::Stop(_) => {
                println!("Received stop");
                break;
            }
            _ => {}
        }
    }

    Ok(())
}

/// Create an Arrow StructArray matching `example_interfaces/AddTwoInts_Response`.
fn make_add_response(sum: i64) -> StructArray {
    let fields = vec![Arc::new(Field::new("sum", DataType::Int64, false))];
    let arrays: Vec<Arc<dyn Array>> = vec![Arc::new(Int64Array::from(vec![sum]))];
    StructArray::try_new(fields.into(), arrays, None).expect("failed to create struct array")
}
