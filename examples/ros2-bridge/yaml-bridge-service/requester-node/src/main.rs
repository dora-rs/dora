//! Service requester node for the YAML bridge service example.
//!
//! Sends `example_interfaces/AddTwoInts` requests as Arrow struct arrays
//! through the declarative ROS2 service bridge, and prints the response.
//! No ROS2 dependencies needed -- the bridge handles all ROS2 communication.

use std::sync::Arc;

use arrow::array::{Array, Int64Array, StructArray};
use arrow::datatypes::{DataType, Field};
use dora_node_api::{self, DoraNode, Event, dora_core::config::DataId};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let output = DataId::from("request".to_owned());

    let mut counter: i64 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "tick" => {
                    counter += 1;
                    let a = counter;
                    let b = counter * 10;

                    // Build Arrow struct matching AddTwoInts_Request: {a: i64, b: i64}
                    let request = make_add_request(a, b);
                    println!("Sending request: {a} + {b}");
                    node.send_output(output.clone(), metadata.parameters, request)?;
                }
                "response" => {
                    // Parse Arrow struct matching AddTwoInts_Response: {sum: i64}
                    let struct_array = data
                        .as_any()
                        .downcast_ref::<StructArray>()
                        .expect("expected struct array for response");
                    if struct_array.len() == 0 {
                        eprintln!("Warning: received empty response array");
                        continue;
                    }
                    let sum = struct_array
                        .column_by_name("sum")
                        .expect("missing 'sum' field")
                        .as_any()
                        .downcast_ref::<Int64Array>()
                        .expect("expected Int64Array for 'sum'")
                        .value(0);
                    println!("Received response: sum = {sum}");
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

/// Create an Arrow StructArray matching `example_interfaces/AddTwoInts_Request`.
fn make_add_request(a: i64, b: i64) -> StructArray {
    let fields = vec![
        Arc::new(Field::new("a", DataType::Int64, false)),
        Arc::new(Field::new("b", DataType::Int64, false)),
    ];
    let arrays: Vec<Arc<dyn Array>> = vec![
        Arc::new(Int64Array::from(vec![a])),
        Arc::new(Int64Array::from(vec![b])),
    ];
    StructArray::try_new(fields.into(), arrays, None).expect("failed to create struct array")
}
