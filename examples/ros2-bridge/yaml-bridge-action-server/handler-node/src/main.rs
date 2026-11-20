//! Action server handler node for the YAML bridge action server example.
//!
//! Receives Fibonacci goals from the ROS2 action bridge and computes each
//! sequence one step per `tick`, sending feedback after every step and the
//! result at the end. A goal canceled by a ROS2 client (the bridge's `cancel`
//! output) ends with the steps computed so far and `goal_status: canceled`.
//! The `goal_id` is passed through metadata parameters so the bridge can
//! correlate feedback/result to the correct goal.

use std::{collections::BTreeMap, sync::Arc};

use arrow::array::{Array, Int32Array, Int32Builder, ListBuilder, StructArray};
use arrow::datatypes::Field;
use dora_node_api::{
    DoraNode, Event, GOAL_ID, GOAL_STATUS, GOAL_STATUS_CANCELED, MetadataParameters, Parameter,
    dora_core::config::DataId, get_string_param,
};

/// A goal being computed: its metadata (with `goal_id`), its order, and the
/// sequence so far.
struct Goal {
    params: MetadataParameters,
    order: i32,
    sequence: Vec<i32>,
}

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let feedback_output = DataId::from("feedback".to_owned());
    let result_output = DataId::from("result".to_owned());
    let mut goals: BTreeMap<String, Goal> = BTreeMap::new();

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "goal" => {
                    // metadata.parameters contains goal_id from the bridge;
                    // pass it through on all outputs so the bridge can
                    // correlate feedback/result to the correct goal.
                    let params = metadata.parameters;
                    let Some(goal_id) = get_string_param(&params, GOAL_ID).map(str::to_owned)
                    else {
                        eprintln!("Warning: goal without goal_id, skipping");
                        continue;
                    };
                    let Some(order) = goal_order(data.as_array().as_ref()) else {
                        continue;
                    };
                    println!("Received Fibonacci goal {goal_id}: order={order}");
                    goals.insert(
                        goal_id,
                        Goal {
                            params,
                            order,
                            sequence: Vec::new(),
                        },
                    );
                }
                "tick" => {
                    // One step of every goal; finished goals send their result.
                    // A goal with `order <= 0` finishes with an empty sequence.
                    let mut finished = Vec::new();
                    for (goal_id, goal) in &mut goals {
                        let order = goal.order.max(0) as usize;
                        if goal.sequence.len() < order {
                            let next = match goal.sequence.len() {
                                0 => 0,
                                1 => 1,
                                n => goal.sequence[n - 1] + goal.sequence[n - 2],
                            };
                            goal.sequence.push(next);
                            let feedback = make_list_struct("sequence", &goal.sequence)?;
                            node.send_output(
                                feedback_output.clone(),
                                goal.params.clone(),
                                feedback,
                            )?;
                        }
                        if goal.sequence.len() >= order {
                            finished.push(goal_id.clone());
                        }
                    }
                    for goal_id in finished {
                        let goal = goals.remove(&goal_id).expect("finished goal is known");
                        let result = make_list_struct("sequence", &goal.sequence)?;
                        node.send_output(result_output.clone(), goal.params, result)?;
                        println!("Result {goal_id}: sequence={:?}", goal.sequence);
                    }
                }
                "cancel" => {
                    let Some(goal_id) = get_string_param(&metadata.parameters, GOAL_ID) else {
                        continue;
                    };
                    // A goal that already finished has nothing left to cancel.
                    let Some(mut goal) = goals.remove(goal_id) else {
                        continue;
                    };
                    goal.params.insert(
                        GOAL_STATUS.to_string(),
                        Parameter::String(GOAL_STATUS_CANCELED.to_string()),
                    );
                    let result = make_list_struct("sequence", &goal.sequence)?;
                    node.send_output(result_output.clone(), goal.params, result)?;
                    println!("Canceled {goal_id} at sequence={:?}", goal.sequence);
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

/// The `order` field of a Fibonacci goal, or `None` (with a warning) if the
/// goal does not have one.
fn goal_order(goal: &dyn Array) -> Option<i32> {
    let Some(struct_array) = goal.as_any().downcast_ref::<StructArray>() else {
        eprintln!("Warning: expected struct array for goal, skipping");
        return None;
    };
    if struct_array.is_empty() {
        eprintln!("Warning: received empty goal array");
        return None;
    }
    let Some(order_col) = struct_array.column_by_name("order") else {
        eprintln!("Warning: missing 'order' field in goal");
        return None;
    };
    let Some(order_array) = order_col.as_any().downcast_ref::<Int32Array>() else {
        eprintln!("Warning: 'order' field is not Int32Array");
        return None;
    };
    Some(order_array.value(0))
}

/// Create an Arrow struct with a single `list<int32>` field.
fn make_list_struct(field_name: &str, values: &[i32]) -> eyre::Result<StructArray> {
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
    Ok(StructArray::try_new(fields.into(), arrays, None)?)
}
