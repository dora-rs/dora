//! Wrapper of [rcl_yaml_param_parser](https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser)

use std::os::raw::c_char;

use crate::rcutils::{rcutils_allocator_t, rcutils_string_array_t};

extern "C" {
    /// Free parameter structure
    pub fn rcl_yaml_node_struct_fini(params_st: *mut rcl_params_t);
}

/// Array of bool values
#[repr(C)]
#[derive(Debug)]
pub struct rcl_bool_array_t {
    /// Array with bool values
    pub values: *mut bool,
    /// Number of values in the array
    pub size: usize,
}

/// Array of int64_t values
#[repr(C)]
#[derive(Debug)]
pub struct rcl_int64_array_t {
    /// Array with int64 values
    pub values: *mut i64,
    /// Number of values in the array
    pub size: usize,
}

/// Array of double values
#[repr(C)]
#[derive(Debug)]
pub struct rcl_double_array_t {
    /// Array with double values
    pub values: *mut f64,
    /// Number of values in the array
    pub size: usize,
}

/// Array of byte values
#[repr(C)]
#[derive(Debug)]
pub struct rcl_byte_array_t {
    /// Array with uint8_t values
    pub values: *mut u8,
    /// Number of values in the array
    pub size: usize,
}

/// variant_t stores the value of a parameter
#[repr(C)]
#[derive(Debug)]
pub struct rcl_variant_t {
    /// If bool, gets stored here
    pub bool_value: *mut bool,
    /// If integer, gets stored here
    pub integer_value: *mut i64,
    /// If double, gets stored here
    pub double_value: *mut f64,
    /// If string, gets stored here
    pub string_value: *mut c_char,
    /// If array of bytes
    pub byte_array_value: *mut rcl_byte_array_t,
    /// If array of bool's
    pub bool_array_value: *mut rcl_bool_array_t,
    /// If array of integers
    pub integer_array_value: *mut rcl_int64_array_t,
    /// If array of doubles
    pub double_array_value: *mut rcl_double_array_t,
    /// If array of strings
    pub string_array_value: *mut rcutils_string_array_t,
}

/// node_params_t stores all the parameters(key:value) of a single node
#[repr(C)]
#[derive(Debug)]
pub struct rcl_node_params_t {
    /// Array of parameter names (keys)
    pub parameter_names: *mut *mut c_char,
    /// Array of coressponding parameter values
    pub parameter_values: *mut rcl_variant_t,
    /// Number of parameters in the node
    pub num_params: usize,
}

/// stores all the parameters of all nodes of a process
#[repr(C)]
#[derive(Debug)]
pub struct rcl_params_t {
    /// List of names of the node
    pub node_names: *mut *mut c_char,
    ///  Array of parameters
    pub params: *mut rcl_node_params_t,
    /// Number of nodes
    pub num_nodes: usize,
    /// Allocator used
    pub allocator: rcutils_allocator_t,
}
