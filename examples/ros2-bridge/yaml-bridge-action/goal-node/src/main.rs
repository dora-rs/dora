//! Action client goal node for the YAML bridge action example.
//!
//! Sends `example_interfaces/Fibonacci` goals as Arrow struct arrays
//! through the declarative ROS2 action bridge, and prints feedback/results.
//! No ROS2 dependencies needed -- the bridge handles all ROS2 communication.

use std::sync::Arc;

use arrow::array::{Array, Int32Array, ListArray, StructArray};
use arrow::datatypes::{DataType, Field};
use dora_node_api::{self, DoraNode, Event, dora_core::config::DataId};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
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
                        .as_array()
                        .as_any()
                        .downcast_ref::<StructArray>()
                        .expect("expected struct array for feedback");
                    if struct_array.len() == 0 {
                        eprintln!("Warning: received empty feedback array");
                        continue;
                    }
                    // example_interfaces/action/Fibonacci feedback contains {sequence: int32[]}.
                    let values = int32_list_values(struct_array, "sequence")
                        .expect("invalid Fibonacci feedback array");
                    println!("Feedback: sequence={values:?}");
                }
                "result" => {
                    let struct_array = data
                        .as_array()
                        .as_any()
                        .downcast_ref::<StructArray>()
                        .expect("expected struct array for result");
                    if struct_array.len() == 0 {
                        eprintln!("Warning: received empty result array");
                        continue;
                    }
                    // example_interfaces/action/Fibonacci result contains {sequence: int32[]}.
                    let values =
                        int32_list_values(struct_array, "sequence").expect("invalid result array");
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

/// Read the `int32[]` field `field_name` from the first row of `struct_array`.
fn int32_list_values(struct_array: &StructArray, field_name: &str) -> eyre::Result<Vec<i32>> {
    let list = struct_array
        .column_by_name(field_name)
        .ok_or_else(|| eyre::eyre!("missing '{field_name}' field"))?
        .as_any()
        .downcast_ref::<ListArray>()
        .ok_or_else(|| eyre::eyre!("expected ListArray for '{field_name}'"))?;
    let values = list
        .value(0)
        .as_any()
        .downcast_ref::<Int32Array>()
        .ok_or_else(|| eyre::eyre!("expected Int32Array values for '{field_name}'"))?
        .values()
        .to_vec();
    Ok(values)
}

/// Create an Arrow StructArray matching `example_interfaces/Fibonacci_Goal`.
fn make_fibonacci_goal(order: i32) -> StructArray {
    let fields = vec![Arc::new(Field::new("order", DataType::Int32, false))];
    let arrays: Vec<Arc<dyn Array>> = vec![Arc::new(Int32Array::from(vec![order]))];
    StructArray::try_new(fields.into(), arrays, None).expect("failed to create struct array")
}
