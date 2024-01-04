pub use ros2_client;
pub use rustdds;

#[cfg(all(feature = "generate-messages", feature = "cxx-bridge"))]
dora_ros2_bridge_msg_gen_macro::msg_include_all!(cxx_bridge = true);

#[cfg(all(feature = "generate-messages", not(feature = "cxx-bridge")))]
dora_ros2_bridge_msg_gen_macro::msg_include_all!();

pub mod _core;
