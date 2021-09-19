//! Wrapper of [rosidl_runtime_c](https://github.com/ros2/rosidl/tree/master/rosidl_runtime_c)

use std::os::raw::{c_char, c_void};

pub type rosidl_message_typesupport_handle_function = Option<
    unsafe extern "C" fn(
        arg1: *const rosidl_message_type_support_t,
        arg2: *const c_char,
    ) -> *const rosidl_message_type_support_t,
>;

/// Contains rosidl message type support data
#[repr(C)]
#[derive(Debug)]
pub struct rosidl_message_type_support_t {
    /// String identifier for the type_support.
    pub typesupport_identifier: *const c_char,
    /// Pointer to the message type support library
    pub data: *const c_void,
    /// Pointer to the message type support handler function
    pub func: rosidl_message_typesupport_handle_function,
}

pub type rosidl_service_typesupport_handle_function = Option<
    unsafe extern "C" fn(
        arg1: *const rosidl_service_type_support_t,
        arg2: *const c_char,
    ) -> *const rosidl_service_type_support_t,
>;

/// Contains rosidl service type support data
#[repr(C)]
#[derive(Debug)]
pub struct rosidl_service_type_support_t {
    /// String identifier for the type_support.
    pub typesupport_identifier: *const c_char,
    /// Pointer to the service type support library
    pub data: *const c_void,
    /// Pointer to the service type support handler function
    pub func: rosidl_service_typesupport_handle_function,
}
