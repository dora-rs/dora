//! Action server handler node for the YAML bridge action server example.
//!
//! Receives Fibonacci goals from the ROS2 action bridge, computes the
//! sequence step-by-step with feedback, and sends the final result.
//! The `goal_id` is passed through metadata parameters so the bridge
//! can correlate feedback/result to the correct goal.

use std::sync::Arc;

use adora_node_api::{AdoraNode, Event, adora_core::config::DataId};
use arrow::array::{Array, Int32Array, Int32Builder, ListBuilder, StructArray};
use arrow::datatypes::Field;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;
    let feedback_output = DataId::from("feedback".to_owned());
    let result_output = DataId::from("result".to_owned());

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "goal" => {
                    // metadata.parameters contains goal_id from the bridge;
                    // pass it through on all outputs so the bridge can
                    // correlate feedback/result to the correct goal.
                    let params = metadata.parameters;

                    let struct_array = data
                        .as_any()
                        .downcast_ref::<StructArray>()
                        .expect("expected struct array for goal");
                    if struct_array.len() == 0 {
                        eprintln!("Warning: received empty goal array");
                        continue;
                    }

                    let order = struct_array
                        .column_by_name("order")
                        .expect("missing 'order' field")
                        .as_any()
                        .downcast_ref::<Int32Array>()
                        .expect("expected Int32Array for 'order'")
                        .value(0);

                    println!("Received Fibonacci goal: order={order}");

                    // Compute Fibonacci sequence with step-by-step feedback
                    let mut sequence: Vec<i32> = Vec::new();
                    for i in 0..order {
                        let val = match i {
                            0 => 0,
                            1 => 1,
                            _ => sequence[sequence.len() - 1] + sequence[sequence.len() - 2],
                        };
                        sequence.push(val);

                        let feedback = make_list_struct("partial_sequence", &sequence);
                        node.send_output(feedback_output.clone(), params.clone(), feedback)?;
                        println!("Feedback: partial_sequence={sequence:?}");
                    }

                    // Send final result
                    let result = make_list_struct("sequence", &sequence);
                    node.send_output(result_output.clone(), params, result)?;
                    println!("Result: sequence={sequence:?}");
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

/// Create an Arrow struct with a single `list<int32>` field.
fn make_list_struct(field_name: &str, values: &[i32]) -> StructArray {
    let mut builder = ListBuilder::new(Int32Builder::new());
    builder.values().append_slice(values);
    builder.append(true);
    let list_array = builder.finish();

    let fields = vec![Arc::new(Field::new(
        field_name,
        list_array.data_type().clone(),
        false,
    ))];
    let arrays: Vec<Arc<dyn Array>> = vec![Arc::new(list_array)];
    StructArray::try_new(fields.into(), arrays, None).expect("failed to create struct array")
}
