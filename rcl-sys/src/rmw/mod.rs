//! Wrapper of [rmw](https://github.com/ros2/rmw/tree/master/rmw)

pub mod names_and_types;
pub use names_and_types::*;

pub mod ret_types {
    //! API in rmw/ret_types.h

    /// Return code for rmw functions
    #[must_use]
    pub type rmw_ret_t = i32;
}
pub use ret_types::*;

pub mod serialized_message {
    //! API in rmw/serialized_message.h

    use crate::rcutils::*;

    /// Serialized message as a string of bytes.
    pub type rmw_serialized_message_t = rcutils_uint8_array_t;
}
pub use serialized_message::*;

pub mod topic_endpoint_info;
pub use topic_endpoint_info::*;

pub mod topic_endpoint_info_array;
pub use topic_endpoint_info_array::*;

pub mod types;
pub use types::*;
