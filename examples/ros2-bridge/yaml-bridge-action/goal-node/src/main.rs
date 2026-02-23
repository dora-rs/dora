//! Action client goal node for the YAML bridge action example.
//!
//! Sends `example_interfaces/Fibonacci` goals as Arrow struct arrays
//! through the declarative ROS2 action bridge, and prints feedback/results.
//! No ROS2 dependencies needed -- the bridge handles all ROS2 communication.

use std::sync::Arc;

use adora_node_api::{self, AdoraNode, Event, adora_core::config::DataId};
use arrow::array::{Array, Int32Array, ListArray, StructArray};
use arrow::datatypes::{DataType, Field};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;
    let output = DataId::from("goal".to_owned());

    let mut order: i32 = 5;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "tick" => {
                    // Build Arrow struct matching Fibonacci_Goal: {order: int32}
                    let goal = make_fibonacci_goal(order);
                    println!("Sending Fibonacci goal: order={order}");
                    node.send_output(output.clone(), metadata.parameters, goal)?;
                    order += 1;
                }
                "feedback" => {
                    let struct_array = data
                        .as_any()
                        .downcast_ref::<StructArray>()
                        .expect("expected struct array for feedback");
                    if struct_array.len() == 0 {
                        eprintln!("Warning: received empty feedback array");
                        continue;
                    }
                    // Feedback contains {partial_sequence: int32[]}
                    let list = struct_array
                        .column_by_name("partial_sequence")
                        .expect("missing 'partial_sequence' field")
                        .as_any()
                        .downcast_ref::<ListArray>()
                        .expect("expected ListArray for 'partial_sequence'");
                    let values = list
                        .value(0)
                        .as_any()
                        .downcast_ref::<Int32Array>()
                        .expect("expected Int32Array values")
                        .values()
                        .to_vec();
                    println!("Feedback: partial_sequence={values:?}");
                }
                "result" => {
                    let struct_array = data
                        .as_any()
                        .downcast_ref::<StructArray>()
                        .expect("expected struct array for result");
                    if struct_array.len() == 0 {
                        eprintln!("Warning: received empty result array");
                        continue;
                    }
                    // Result contains {sequence: int32[]}
                    let list = struct_array
                        .column_by_name("sequence")
                        .expect("missing 'sequence' field")
                        .as_any()
                        .downcast_ref::<ListArray>()
                        .expect("expected ListArray for 'sequence'");
                    let values = list
                        .value(0)
                        .as_any()
                        .downcast_ref::<Int32Array>()
                        .expect("expected Int32Array values")
                        .values()
                        .to_vec();
                    println!("Result: sequence={values:?}");
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

/// Create an Arrow StructArray matching `example_interfaces/Fibonacci_Goal`.
fn make_fibonacci_goal(order: i32) -> StructArray {
    let fields = vec![Arc::new(Field::new("order", DataType::Int32, false))];
    let arrays: Vec<Arc<dyn Array>> = vec![Arc::new(Int32Array::from(vec![order]))];
    StructArray::try_new(fields.into(), arrays, None).expect("failed to create struct array")
}
