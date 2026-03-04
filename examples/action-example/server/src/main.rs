use std::collections::{HashMap, HashSet};

use adora_node_api::{
    AdoraNode, Event, GOAL_ID, GOAL_STATUS, GOAL_STATUS_CANCELED, GOAL_STATUS_SUCCEEDED,
    MetadataParameters, Parameter,
    arrow::array::{Array, Int64Array},
    get_string_param,
};

const MAX_ACTIVE_GOALS: usize = 64;
const MAX_PENDING_CANCELS: usize = 128;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;

    // Active goals: goal_id -> remaining countdown value
    let mut active_goals: HashMap<String, i64> = HashMap::new();
    let mut canceled: HashSet<String> = HashSet::new();

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                _id if _id.starts_with("goal") => {
                    let goal_id =
                        get_string_param(&metadata.parameters, GOAL_ID).map(str::to_owned);
                    let start = data
                        .as_any()
                        .downcast_ref::<Int64Array>()
                        .map(|a| a.value(0));

                    if let (Some(goal_id), Some(start)) = (goal_id, start) {
                        if active_goals.len() >= MAX_ACTIVE_GOALS {
                            eprintln!("[server] max active goals reached, dropping {goal_id}");
                            continue;
                        }
                        println!("[server] new goal {goal_id}: countdown from {start}");
                        active_goals.insert(goal_id, start);
                    }
                }
                _id if _id.starts_with("cancel") => {
                    let goal_id =
                        get_string_param(&metadata.parameters, GOAL_ID).map(str::to_owned);
                    if let Some(goal_id) = goal_id {
                        if canceled.len() >= MAX_PENDING_CANCELS {
                            eprintln!("[server] cancel queue full, dropping cancel for {goal_id}");
                            continue;
                        }
                        println!("[server] cancel requested for {goal_id}");
                        canceled.insert(goal_id);
                    }
                }
                // Use timer ticks or self-scheduling to drive countdown.
                // For simplicity, we process all active goals on every input.
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }

        // Process one step for each active goal on every input event.
        // This sits outside the match so goals make progress regardless of
        // which input triggered the event loop iteration.
        if active_goals.is_empty() {
            continue;
        }
        let goal_ids: Vec<String> = active_goals.keys().cloned().collect();
        for goal_id in goal_ids {
            if canceled.remove(&goal_id) {
                active_goals.remove(&goal_id);
                send_result(&mut node, &goal_id, GOAL_STATUS_CANCELED)?;
                continue;
            }

            if let Some(remaining) = active_goals.get_mut(&goal_id) {
                *remaining -= 1;
                let val = *remaining;

                if val <= 0 {
                    active_goals.remove(&goal_id);
                    send_result(&mut node, &goal_id, GOAL_STATUS_SUCCEEDED)?;
                } else {
                    // Send feedback
                    let mut params = MetadataParameters::default();
                    params.insert(GOAL_ID.to_string(), Parameter::String(goal_id.clone()));
                    let fb = Int64Array::from(vec![val]);
                    node.send_output("feedback".into(), params, fb)?;
                    println!("[server] feedback {goal_id}: {val}");
                }
            }
        }
    }

    Ok(())
}

fn send_result(node: &mut AdoraNode, goal_id: &str, status: &str) -> eyre::Result<()> {
    let mut params = MetadataParameters::default();
    params.insert(GOAL_ID.to_string(), Parameter::String(goal_id.to_string()));
    params.insert(
        GOAL_STATUS.to_string(),
        Parameter::String(status.to_string()),
    );
    let result_data = Int64Array::from(vec![0i64]);
    node.send_output("result".into(), params, result_data)?;
    println!("[server] result {goal_id}: {status}");
    Ok(())
}
