//! Action client goal node for the YAML bridge action example.
//!
//! Sends `example_interfaces/Fibonacci` goals as Arrow struct arrays
//! through the declarative ROS2 action bridge, and prints feedback/results.
//! No ROS2 dependencies needed -- the bridge handles all ROS2 communication.

use std::sync::Arc;

use arrow::array::{Array, Int32Array, ListArray, StructArray};
use arrow::datatypes::{DataType, Field};
use dora_node_api::{self, DoraNode, Event, dora_core::config::DataId};

const MAX_SKIPPED_TICKS: u8 = 6;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let output = DataId::from("goal".to_owned());

    let mut order: i32 = 5;
    let mut goal_state = GoalSendState::default();

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "tick" => {
                    if !goal_state.on_tick() {
                        println!("Goal still in flight, skipping tick");
                        continue;
                    }
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
                    // example_interfaces/action/Fibonacci feedback contains {sequence: int32[]}.
                    let values = int32_list_values(struct_array, "sequence")
                        .expect("invalid Fibonacci feedback array");
                    println!("Feedback: sequence={values:?}");
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
                    let values =
                        int32_list_values(struct_array, "sequence").expect("invalid result array");
                    println!("Result: sequence={values:?}");
                    goal_state.on_result();
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

#[derive(Debug, Default)]
struct GoalSendState {
    in_flight: bool,
    skipped_ticks: u8,
}

impl GoalSendState {
    fn on_tick(&mut self) -> bool {
        if self.in_flight {
            self.skipped_ticks = self.skipped_ticks.saturating_add(1);
            if self.skipped_ticks <= MAX_SKIPPED_TICKS {
                false
            } else {
                self.skipped_ticks = 0;
                true
            }
        } else {
            self.in_flight = true;
            self.skipped_ticks = 0;
            true
        }
    }

    fn on_result(&mut self) {
        self.in_flight = false;
        self.skipped_ticks = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{ArrayRef, ListBuilder};

    #[test]
    fn fibonacci_feedback_uses_sequence_field() {
        let feedback = make_i32_list_struct("sequence", &[0, 1, 1, 2, 3]);

        assert_eq!(
            int32_list_values(&feedback, "sequence").unwrap(),
            vec![0, 1, 1, 2, 3]
        );
    }

    #[test]
    fn fibonacci_feedback_rejects_old_partial_sequence_name() {
        let feedback = make_i32_list_struct("sequence", &[0, 1, 1]);

        let error = int32_list_values(&feedback, "partial_sequence").unwrap_err();

        assert!(
            error
                .to_string()
                .contains("missing 'partial_sequence' field")
        );
    }

    #[test]
    fn goal_send_state_allows_one_in_flight_goal() {
        let mut state = GoalSendState::default();

        assert!(state.on_tick());
        assert!(!state.on_tick());

        state.on_result();

        assert!(state.on_tick());
    }

    #[test]
    fn goal_send_state_recovers_if_result_never_arrives() {
        let mut state = GoalSendState::default();

        assert!(state.on_tick());
        for _ in 0..MAX_SKIPPED_TICKS {
            assert!(!state.on_tick());
        }

        assert!(state.on_tick());
    }

    fn make_i32_list_struct(field_name: &str, values: &[i32]) -> StructArray {
        let mut builder = ListBuilder::new(Int32Array::builder(values.len()));
        builder.values().append_slice(values);
        builder.append(true);
        let array: ArrayRef = Arc::new(builder.finish());
        StructArray::try_new(
            vec![Arc::new(Field::new(
                field_name,
                array.data_type().clone(),
                false,
            ))]
            .into(),
            vec![array],
            None,
        )
        .expect("failed to create test struct")
    }
}
