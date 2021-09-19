//! API in rcl/context.h

use crate::*;

pub type rcl_context_instance_id_t = u64;

#[repr(C)]
#[derive(Debug)]
struct rcl_context_impl_t {
    _unused: [u8; 0],
}

/// Encapsulates the non-global state of an init/shutdown cycle.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_context_t {
    /// Global arguments for all nodes which share this context.
    pub global_arguments: rcl_arguments_t,
    /// Implementation specific pointer.
    impl_: *mut rcl_context_impl_t,
    /// Private storage for instance ID atomic.
    pub instance_id_storage: [u8; 8usize],
}

extern "C" {
    /// Return a zero initialization context object.
    pub fn rcl_get_zero_initialized_context() -> rcl_context_t;

    /// Finalize a context.
    pub fn rcl_context_fini(context: *mut rcl_context_t) -> rcl_ret_t;

    /// Return the init options used during initialization for this context.
    #[cfg(feature = "foxy")]
    pub fn rcl_context_get_init_options(context: *mut rcl_context_t) -> *const rcl_init_options_t;
    #[cfg(feature = "galactic+")]
    pub fn rcl_context_get_init_options(context: *const rcl_context_t)
        -> *const rcl_init_options_t;

    /// Returns an unsigned integer that is unique to the given context, or `0` if invalid.
    #[cfg(feature = "foxy")]
    pub fn rcl_context_get_instance_id(context: *mut rcl_context_t) -> rcl_context_instance_id_t;
    #[cfg(feature = "galactic+")]
    pub fn rcl_context_get_instance_id(context: *const rcl_context_t) -> rcl_context_instance_id_t;

    /// Return `true` if the given context is currently valid, otherwise `false`.
    #[cfg(feature = "foxy")]
    pub fn rcl_context_is_valid(context: *mut rcl_context_t) -> bool;
    #[cfg(feature = "galactic+")]
    pub fn rcl_context_is_valid(context: *const rcl_context_t) -> bool;
}
