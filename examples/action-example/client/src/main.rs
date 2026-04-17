use std::collections::HashMap;

use dora_node_api::{
    DoraNode, Event, GOAL_ID, GOAL_STATUS, MetadataParameters, Parameter,
    arrow::array::{Array, Int64Array},
    get_string_param,
};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;

    let node_id = std::env::var("DORA_NODE_ID").unwrap_or_else(|_| "client".into());
    let mut active_goals: HashMap<String, i64> = HashMap::new();
    let mut tick_count: i64 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "tick" => {
                    let start_value = 3 + tick_count;
                    tick_count += 1;

                    let goal_id = DoraNode::new_goal_id();
                    let mut params = MetadataParameters::default();
                    params.insert(GOAL_ID.to_string(), Parameter::String(goal_id.clone()));

                    let data = Int64Array::from(vec![start_value]);
                    node.send_output("goal".into(), params, data)?;

                    println!("[{node_id}] sent goal {goal_id}: countdown from {start_value}");
                    active_goals.insert(goal_id, start_value);
                }
                "feedback" => {
                    let gid = get_string_param(&metadata.parameters, GOAL_ID);
                    let fb_array = data
                        .as_any()
                        .downcast_ref::<Int64Array>()
                        .map(|a| a.value(0));

                    if let (Some(gid), Some(val)) = (gid, fb_array) {
                        println!("[{node_id}] feedback {gid}: {val}");
                    }
                }
                "result" => {
                    let gid = get_string_param(&metadata.parameters, GOAL_ID);
                    let status = get_string_param(&metadata.parameters, GOAL_STATUS);

                    if let Some(gid) = gid {
                        let status = status.unwrap_or("unknown");
                        println!("[{node_id}] result {gid}: {status}");
                        active_goals.remove(gid);
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
