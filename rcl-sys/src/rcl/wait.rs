//! API in rcl/wait.h

use crate::*;

#[repr(C)]
#[derive(Debug)]
struct rcl_wait_set_impl_t {
    _unused: [u8; 0],
}
/// Container for subscription's, guard condition's, etc to be waited on.
#[repr(C)]
#[derive(Debug)]
pub struct rcl_wait_set_t {
    /// Storage for subscription pointers.
    pub subscriptions: *mut *const rcl_subscription_t,
    /// Number of subscriptions
    pub size_of_subscriptions: usize,
    /// Storage for guard condition pointers.
    pub guard_conditions: *mut *const rcl_guard_condition_t,
    /// Number of guard_conditions
    pub size_of_guard_conditions: usize,
    /// Storage for timer pointers.
    pub timers: *mut *const rcl_timer_t,
    /// Number of timers
    pub size_of_timers: usize,
    /// Storage for client pointers.
    pub clients: *mut *const rcl_client_t,
    /// Number of clients
    pub size_of_clients: usize,
    /// Storage for service pointers.
    pub services: *mut *const rcl_service_t,
    /// Number of services
    pub size_of_services: usize,
    /// Storage for event pointers.
    pub events: *mut *const rcl_event_t,
    /// Number of events
    pub size_of_events: usize,
    /// Implementation specific storage.
    impl_: *mut rcl_wait_set_impl_t,
}

extern "C" {
    /// Return a [rcl_wait_set_t] struct with members set to `NULL`.
    pub fn rcl_get_zero_initialized_wait_set() -> rcl_wait_set_t;

    /// Initialize a rcl wait set with space for items to be waited on.
    pub fn rcl_wait_set_init(
        wait_set: *mut rcl_wait_set_t,
        number_of_subscriptions: usize,
        number_of_guard_conditions: usize,
        number_of_timers: usize,
        number_of_clients: usize,
        number_of_services: usize,
        number_of_events: usize,
        context: *mut rcl_context_t,
        allocator: rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Finalize a rcl wait set.
    pub fn rcl_wait_set_fini(wait_set: *mut rcl_wait_set_t) -> rcl_ret_t;

    /// Retrieve the wait set's allocator.
    pub fn rcl_wait_set_get_allocator(
        wait_set: *const rcl_wait_set_t,
        allocator: *mut rcl_allocator_t,
    ) -> rcl_ret_t;

    /// Store a pointer to the given subscription in the next empty spot in the set.
    pub fn rcl_wait_set_add_subscription(
        wait_set: *mut rcl_wait_set_t,
        subscription: *const rcl_subscription_t,
        index: *mut usize,
    ) -> rcl_ret_t;

    /// Remove (sets to `NULL`) all entities in the wait set.
    pub fn rcl_wait_set_clear(wait_set: *mut rcl_wait_set_t) -> rcl_ret_t;

    /// Reallocate space for entities in the wait set.
    pub fn rcl_wait_set_resize(
        wait_set: *mut rcl_wait_set_t,
        subscriptions_size: usize,
        guard_conditions_size: usize,
        timers_size: usize,
        clients_size: usize,
        services_size: usize,
        events_size: usize,
    ) -> rcl_ret_t;

    /// Store a pointer to the guard condition in the next empty spot in the set.
    pub fn rcl_wait_set_add_guard_condition(
        wait_set: *mut rcl_wait_set_t,
        guard_condition: *const rcl_guard_condition_t,
        index: *mut usize,
    ) -> rcl_ret_t;

    /// Store a pointer to the timer in the next empty spot in the set.
    pub fn rcl_wait_set_add_timer(
        wait_set: *mut rcl_wait_set_t,
        timer: *const rcl_timer_t,
        index: *mut usize,
    ) -> rcl_ret_t;

    /// Store a pointer to the client in the next empty spot in the set.
    pub fn rcl_wait_set_add_client(
        wait_set: *mut rcl_wait_set_t,
        client: *const rcl_client_t,
        index: *mut usize,
    ) -> rcl_ret_t;

    /// Store a pointer to the service in the next empty spot in the set.
    pub fn rcl_wait_set_add_service(
        wait_set: *mut rcl_wait_set_t,
        service: *const rcl_service_t,
        index: *mut usize,
    ) -> rcl_ret_t;

    /// Store a pointer to the event in the next empty spot in the set.
    pub fn rcl_wait_set_add_event(
        wait_set: *mut rcl_wait_set_t,
        event: *const rcl_event_t,
        index: *mut usize,
    ) -> rcl_ret_t;

    /// Block until the wait set is ready or until the timeout has been exceeded.
    pub fn rcl_wait(wait_set: *mut rcl_wait_set_t, timeout: i64) -> rcl_ret_t;

    /// Return `true` if the wait set is valid, else `false`.
    pub fn rcl_wait_set_is_valid(wait_set: *const rcl_wait_set_t) -> bool;
}
