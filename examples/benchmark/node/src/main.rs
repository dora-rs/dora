use dora_node_api::{self, dora_core::config::DataId, DoraNode};
use rand::Rng;

fn main() -> eyre::Result<()> {
    let output = DataId::from("random".to_owned());

    let (mut node, _events) = DoraNode::init_from_env()?;
    for size in [0, 8, 64, 512, 2048, 4096, 4 * 4096, 10 * 4096] {
        for _ in 0..100 {
            let data: Vec<u8> = rand::thread_rng()
                .sample_iter(rand::distributions::Standard)
                .take(size)
                .collect();
            node.send_output(output.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(&data);
            })?;
        }
    }

    Ok(())
}
