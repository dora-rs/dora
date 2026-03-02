use std::collections::{HashMap, HashSet};

use adora_node_api::{
    AdoraNode, Event, GOAL_ID, GOAL_STATUS, GOAL_STATUS_CANCELED, GOAL_STATUS_SUCCEEDED,
    MetadataParameters, Parameter,
    arrow::array::{Array, Int64Array},
};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;

    // Active goals: goal_id -> remaining countdown value
    let mut active_goals: HashMap<String, i64> = HashMap::new();
    let mut canceled: HashSet<String> = HashSet::new();

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "goal" => {
                    let goal_id = extract_string_param(&metadata.parameters, GOAL_ID);
                    let start = data
                        .as_any()
                        .downcast_ref::<Int64Array>()
                        .map(|a| a.value(0));

                    if let (Some(goal_id), Some(start)) = (goal_id, start) {
                        println!("[server] new goal {goal_id}: countdown from {start}");
                        active_goals.insert(goal_id, start);
                    }
                }
                "cancel" => {
                    let goal_id = extract_string_param(&metadata.parameters, GOAL_ID);
                    if let Some(goal_id) = goal_id {
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

        // Process one step for each active goal
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

fn extract_string_param(params: &MetadataParameters, key: &str) -> Option<String> {
    params.get(key).and_then(|p| match p {
        Parameter::String(s) => Some(s.clone()),
        _ => None,
    })
}
