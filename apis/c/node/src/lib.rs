use std::ptr;

use dora_node_api::DoraNode;
use futures::executor::block_on;

pub extern "C" fn init_dora_node_from_env() -> *mut () {
    let node = match block_on(DoraNode::init_from_env()) {
        Ok(n) => n,
        Err(err) => {
            eprintln!("{err:?}");
            return ptr::null_mut();
        }
    };

    Box::into_raw(Box::new(node)).cast()
}

pub extern "C" fn free_dora_node(node: *mut ()) {
    let node: Box<DoraNode> = unsafe { Box::from_raw(node.cast()) };
    let _ = node;
}
