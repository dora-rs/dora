#![warn(clippy::all)]
#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]

pub type va_list = [__va_list_tag; 1usize];

#[repr(C)]
#[derive(Debug)]
pub struct __va_list_tag {
    pub gp_offset: std::os::raw::c_uint,
    pub fp_offset: std::os::raw::c_uint,
    pub overflow_arg_area: *mut std::os::raw::c_void,
    pub reg_save_area: *mut std::os::raw::c_void,
}

pub mod rcl;
pub use rcl::*;

pub mod rcl_yaml_param_parser;
pub use rcl_yaml_param_parser::*;

pub mod rcutils;
pub use rcutils::*;

pub mod rmw;
pub use rmw::*;

pub mod rosidl_runtime_c;
pub use rosidl_runtime_c::*;
