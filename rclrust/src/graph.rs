use std::collections::HashMap;

use crate::{error::ToRclRustResult, internal::ffi::*, log::Logger, rclrust_error};

#[derive(Debug)]
pub(super) struct RclStringArray(rcl_sys::rcutils_string_array_t);

impl RclStringArray {
    pub fn new() -> Self {
        Self(unsafe { rcl_sys::rcutils_get_zero_initialized_string_array() })
    }

    #[inline]
    const fn raw(&self) -> &rcl_sys::rcutils_string_array_t {
        &self.0
    }

    #[inline]
    pub unsafe fn raw_mut(&mut self) -> &mut rcl_sys::rcutils_string_array_t {
        &mut self.0
    }

    pub unsafe fn iter(&self) -> impl Iterator<Item = String> {
        std::slice::from_raw_parts(self.raw().data, self.raw().size)
            .iter()
            .map(|c_str| String::from_c_char(*c_str).unwrap())
    }
}

impl Drop for RclStringArray {
    fn drop(&mut self) {
        if let Err(e) = unsafe { rcl_sys::rcutils_string_array_fini(&mut self.0).to_result() } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl string array: {}",
                e
            )
        }
    }
}

#[derive(Debug)]
pub(super) struct RclNamesAndTypes(rcl_sys::rcl_names_and_types_t);

impl RclNamesAndTypes {
    pub fn new() -> Self {
        Self(unsafe { rcl_sys::rmw_get_zero_initialized_names_and_types() })
    }

    #[inline]
    const fn raw(&self) -> &rcl_sys::rcl_names_and_types_t {
        &self.0
    }

    #[inline]
    pub unsafe fn raw_mut(&mut self) -> &mut rcl_sys::rcl_names_and_types_t {
        &mut self.0
    }

    pub unsafe fn to_hash_map(&self) -> HashMap<String, Vec<String>> {
        let names = std::slice::from_raw_parts(self.raw().names.data, self.raw().names.size);
        let types_vec = std::slice::from_raw_parts(self.raw().types, self.raw().names.size);

        types_vec
            .iter()
            .map(|types| {
                std::slice::from_raw_parts(types.data, types.size)
                    .iter()
                    .map(|type_| String::from_c_char(*type_).unwrap())
                    .collect()
            })
            .zip(names.iter().map(|name| String::from_c_char(*name).unwrap()))
            .map(|(types, name)| (name, types))
            .collect()
    }
}

impl Drop for RclNamesAndTypes {
    fn drop(&mut self) {
        if let Err(e) = unsafe { rcl_sys::rmw_names_and_types_fini(&mut self.0).to_result() } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl names and types: {}",
                e
            )
        }
    }
}
