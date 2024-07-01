#![deny(unsafe_op_in_unsafe_fn)]

use arrow_array::UInt8Array;
use dora_node_api::{arrow::array::AsArray, DoraNode, Event, EventStream};
use eyre::Context;
use std::{ffi::c_void, ptr, slice};

pub const HEADER_NODE_API: &str = include_str!("../node_api.h");

struct DoraContext {
    node: &'static mut DoraNode,
    events: EventStream,
}

/// Initializes a dora context from the environment variables that were set by
/// the dora-coordinator.
///
/// Returns a pointer to the dora context on success. This pointer can be
/// used to call dora API functions that expect a `context` argument. Any
/// other use is prohibited. To free the dora context when it is no longer
/// needed, use the [`free_dora_context`] function.
///
/// On error, a null pointer is returned.
#[no_mangle]
pub extern "C" fn init_dora_context_from_env() -> *mut c_void {
    let context = || {
        let (node, events) = DoraNode::init_from_env()?;
        let node = Box::leak(Box::new(node));
        Result::<_, eyre::Report>::Ok(DoraContext { node, events })
    };
    let context = match context().context("failed to initialize node") {
        Ok(n) => n,
        Err(err) => {
            let err: eyre::Error = err;
            tracing::error!("{err:?}");
            return ptr::null_mut();
        }
    };

    Box::into_raw(Box::new(context)).cast()
}

/// Frees the given dora context.
///
/// ## Safety
///
/// Only pointers created through [`init_dora_context_from_env`] are allowed
/// as arguments. Each context pointer must be freed exactly once. After
/// freeing, the pointer must not be used anymore.
#[no_mangle]
pub unsafe extern "C" fn free_dora_context(context: *mut c_void) {
    let context: Box<DoraContext> = unsafe { Box::from_raw(context.cast()) };
    // drop all fields except for `node`
    let DoraContext { node, .. } = *context;
    // convert the `'static` reference back to a Box, then drop it
    let _ = unsafe { Box::from_raw(node as *const DoraNode as *mut DoraNode) };
}

/// Waits for the next incoming event for the node.
///
/// Returns a pointer to the event on success. This pointer must not be used
/// directly. Instead, use the `read_dora_event_*` functions to read out the
/// type and payload of the event. When the event is not needed anymore, use
/// [`free_dora_event`] to free it again.
///
/// Returns a null pointer when all event streams were closed. This means that
/// no more event will be available. Nodes typically react by stopping.
///
/// ## Safety
///
/// The `context` argument must be a dora context created through
/// [`init_dora_context_from_env`]. The context must be still valid, i.e., not
/// freed yet.
#[no_mangle]
pub unsafe extern "C" fn dora_next_event(context: *mut c_void) -> *mut c_void {
    let context: &mut DoraContext = unsafe { &mut *context.cast() };
    match context.events.recv() {
        Some(event) => Box::into_raw(Box::new(event)).cast(),
        None => ptr::null_mut(),
    }
}

/// Reads out the type of the given event.
///
/// ## Safety
///
/// The `event` argument must be a dora event received through
/// [`dora_next_event`]. The event must be still valid, i.e., not
/// freed yet.
#[no_mangle]
pub unsafe extern "C" fn read_dora_event_type(event: *const ()) -> EventType {
    let event: &Event = unsafe { &*event.cast() };
    match event {
        Event::Stop => EventType::Stop,
        Event::Input { .. } => EventType::Input,
        Event::InputClosed { .. } => EventType::InputClosed,
        Event::Error(_) => EventType::Error,
        _ => EventType::Unknown,
    }
}

#[repr(C)]
pub enum EventType {
    Stop,
    Input,
    InputClosed,
    Error,
    Unknown,
}

/// Reads out the ID of the given input event.
///
/// Writes the `out_ptr` and `out_len` with the start pointer and length of the
/// ID string of the input. The ID is guaranteed to be valid UTF-8.
///
/// Writes a null pointer and length `0` if the given event is not an input event.
///
/// ## Safety
///
/// The `event` argument must be a dora event received through
/// [`dora_next_event`]. The event must be still valid, i.e., not
/// freed yet. The returned `out_ptr` must not be used after
/// freeing the `event`, since it points directly into the event's
/// memory.
#[no_mangle]
pub unsafe extern "C" fn read_dora_input_id(
    event: *const (),
    out_ptr: *mut *const u8,
    out_len: *mut usize,
) {
    let event: &Event = unsafe { &*event.cast() };
    match event {
        Event::Input { id, .. } => {
            let id = id.as_str().as_bytes();
            let ptr = id.as_ptr();
            let len = id.len();
            unsafe {
                *out_ptr = ptr;
                *out_len = len;
            }
        }
        _ => unsafe {
            *out_ptr = ptr::null();
            *out_len = 0;
        },
    }
}

/// Reads out the data of the given input event.
///
/// Writes the `out_ptr` and `out_len` with the start pointer and length of the
/// input's data array. The data array is a raw byte array, whose format
/// depends on the source operator/node.
///
/// Writes a null pointer and length `0` if the given event is not an input event
/// or when an input event has no associated data.
///
/// ## Safety
///
/// The `event` argument must be a dora event received through
/// [`dora_next_event`]. The event must be still valid, i.e., not
/// freed yet. The returned `out_ptr` must not be used after
/// freeing the `event`, since it points directly into the event's
/// memory.
#[no_mangle]
pub unsafe extern "C" fn read_dora_input_data(
    event: *const (),
    out_ptr: *mut *const u8,
    out_len: *mut usize,
) {
    let event: &Event = unsafe { &*event.cast() };
    match event {
        Event::Input { data, metadata, .. } => match metadata.type_info.data_type {
            dora_node_api::arrow::datatypes::DataType::UInt8 => {
                let array: &UInt8Array = data.as_primitive();
                let ptr = array.values().as_ptr();
                unsafe {
                    *out_ptr = ptr;
                    *out_len = metadata.type_info.len;
                }
            }
            dora_node_api::arrow::datatypes::DataType::Null => unsafe {
                *out_ptr = ptr::null();
                *out_len = 0;
            },
            _ => {
                todo!("dora C++ Node does not yet support higher level type of arrow. Only UInt8. 
                The ultimate solution should be based on arrow FFI interface. Feel free to contribute :)")
            }
        },
        _ => unsafe {
            *out_ptr = ptr::null();
            *out_len = 0;
        },
    }
}

/// Reads out the timestamp of the given input event from metadata.
///
/// Return `0` if the given event is not an input event.
#[no_mangle]
pub unsafe extern "C" fn read_dora_input_timestamp(event: *const ()) -> core::ffi::c_ulonglong {
    let event: &Event = unsafe { &*event.cast() };
    match event {
        Event::Input { metadata, .. } => metadata.timestamp().get_time().as_u64(),
        _ => 0,
    }
}

/// Frees the given dora event.
///
/// ## Safety
///
/// Only pointers created through [`dora_next_event`] are allowed
/// as arguments. Each context pointer must be freed exactly once. After
/// freeing, the pointer and all derived pointers must not be used anymore.
/// This also applies to the `read_dora_event_*` functions, which return
/// pointers into the original event structure.
#[no_mangle]
pub unsafe extern "C" fn free_dora_event(event: *mut c_void) {
    let _: Box<Event> = unsafe { Box::from_raw(event.cast()) };
}

/// Sends the given output to subscribed dora nodes/operators.
///
/// The `id_ptr` and `id_len` fields must be the start pointer and length of an
/// UTF8-encoded string. The ID string must correspond to one of the node's
/// outputs specified in the dataflow YAML file.
///
/// The `data_ptr` and `data_len` fields must be the start pointer and length
/// a byte array. The dora API sends this data as-is, without any processing.
///
/// ## Safety
///
/// - The `id_ptr` and `id_len` fields must be the start pointer and length of an
///   UTF8-encoded string.
/// - The `data_ptr` and `data_len` fields must be the start pointer and length
///   a byte array.
#[no_mangle]
pub unsafe extern "C" fn dora_send_output(
    context: *mut c_void,
    id_ptr: *const u8,
    id_len: usize,
    data_ptr: *const u8,
    data_len: usize,
) -> isize {
    match unsafe { try_send_output(context, id_ptr, id_len, data_ptr, data_len) } {
        Ok(()) => 0,
        Err(err) => {
            tracing::error!("{err:?}");
            -1
        }
    }
}

unsafe fn try_send_output(
    context: *mut c_void,
    id_ptr: *const u8,
    id_len: usize,
    data_ptr: *const u8,
    data_len: usize,
) -> eyre::Result<()> {
    let context: &mut DoraContext = unsafe { &mut *context.cast() };
    let id = std::str::from_utf8(unsafe { slice::from_raw_parts(id_ptr, id_len) })?;
    let output_id = id.to_owned().into();
    let data = unsafe { slice::from_raw_parts(data_ptr, data_len) };
    context
        .node
        .send_output_raw(output_id, Default::default(), data.len(), |out| {
            out.copy_from_slice(data);
        })
}
