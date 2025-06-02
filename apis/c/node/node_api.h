#ifndef __RUST_DORA_NODE_API_C_WRAPPER__
#define __RUST_DORA_NODE_API_C_WRAPPER__
#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>

/// Initializes a dora context from the environment variables that were set by
/// the dora-coordinator.
///
/// Returns a pointer to the dora context on success. This pointer can be
/// used to call dora API functions that expect a `context` argument. Any
/// other use is prohibited. To free the dora context when it is no longer
/// needed, use the `free_dora_context` function.
///
/// On error, a null pointer is returned.
///
/// ## Thread Safety
///
/// The returned context pointer is thread-safe and can be used concurrently
/// from multiple threads. All functions that take this context pointer as
/// an argument are thread-safe.
void *init_dora_context_from_env();

/// Frees the given dora context.
///
/// ## Safety
///
/// Only pointers created through `init_dora_context_from_env` are allowed
/// as arguments. Each context pointer must be freed exactly once. After
/// freeing, the pointer must not be used anymore.
///
/// ## Thread Safety
///
/// This function is not thread-safe. It must not be called concurrently
/// with any other function that uses the same context pointer.
void free_dora_context(void *dora_context);

/// Waits for the next incoming event for the node.
///
/// Returns a pointer to the event on success. This pointer must not be used
/// directly. Instead, use the `read_dora_event_*` functions to read out the
/// type and payload of the event. When the event is not needed anymore, use
/// `free_dora_event` to free it again.
///
/// Returns a null pointer when all event streams were closed. This means that
/// no more event will be available. Nodes typically react by stopping.
///
/// ## Safety
///
/// The `context` argument must be a dora context created through
/// `init_dora_context_from_env`. The context must be still valid, i.e., not
/// freed yet.
///
/// ## Thread Safety
///
/// This function is thread-safe. Multiple threads can call this function
/// concurrently with the same context pointer. The underlying event stream
/// is protected by a mutex to ensure thread-safe access.
void *dora_next_event(void *dora_context);

/// Frees the given dora event.
///
/// ## Safety
///
/// Only pointers created through `dora_next_event` are allowed
/// as arguments. Each context pointer must be freed exactly once. After
/// freeing, the pointer and all derived pointers must not be used anymore.
/// This also applies to the `read_dora_event_*` functions, which return
/// pointers into the original event structure.
///
/// ## Thread Safety
///
/// This function is not thread-safe. It must not be called concurrently
/// with any other function that uses the same event pointer.
void free_dora_event(void *dora_event);

/// Reads out the type of the given event.
///
/// ## Safety
///
/// The `event` argument must be a dora event received through
/// `dora_next_event`. The event must be still valid, i.e., not
/// freed yet.
///
/// ## Thread Safety
///
/// This function is thread-safe. Multiple threads can call this function
/// concurrently with the same event pointer.
enum DoraEventType read_dora_event_type(void *dora_event);

/// Reads out the ID of the given input event.
///
/// Writes the `out_ptr` and `out_len` with the start pointer and length of the
/// ID string of the input. The ID is guaranteed to be valid UTF-8.
///
/// Writes a null pointer and length `0` if the given event is not an input event.
///
/// ## Safety
///
/// - The `event` argument must be a dora event received through
/// `dora_next_event`. The event must be still valid, i.e., not
/// freed yet. The returned `out_ptr` must not be used after
/// freeing the `event`, since it points directly into the event's
/// memory.
///
/// - Note: `Out_ptr` is not a null-terminated string. The length of the string
/// is given by `out_len`.
///
/// ## Thread Safety
///
/// This function is thread-safe. Multiple threads can call this function
/// concurrently with the same event pointer.
void read_dora_input_id(void *dora_event, char **out_ptr, size_t *out_len);

/// Reads out the data of the given input event.
///
/// Writes the `out_ptr` and `out_len` with the start pointer and length of the
/// data array of the input.
///
/// Writes a null pointer and length `0` if the given event is not an input event.
///
/// ## Safety
///
/// - The `event` argument must be a dora event received through
/// `dora_next_event`. The event must be still valid, i.e., not
/// freed yet. The returned `out_ptr` must not be used after
/// freeing the `event`, since it points directly into the event's
/// memory.
///
/// ## Thread Safety
///
/// This function is thread-safe. Multiple threads can call this function
/// concurrently with the same event pointer.
void read_dora_input_data(void *dora_event, char **out_ptr, size_t *out_len);

/// Reads out the timestamp of the given input event.
///
/// Returns the timestamp as a 64-bit unsigned integer. Returns `0` if the given
/// event is not an input event.
///
/// ## Safety
///
/// The `event` argument must be a dora event received through
/// `dora_next_event`. The event must be still valid, i.e., not
/// freed yet.
///
/// ## Thread Safety
///
/// This function is thread-safe. Multiple threads can call this function
/// concurrently with the same event pointer.
unsigned long long read_dora_input_timestamp(void *dora_event);

/// Sends the given output to subscribed dora nodes/operators.
///
/// The `id_ptr` and `id_len` fields must be the start pointer and length of an
/// UTF8-encoded string. The ID string must correspond to one of the node's
/// outputs specified in the dataflow YAML file.
///
/// The `data_ptr` and `data_len` fields must be the start pointer and length
/// a byte array. The dora API sends this data as-is, without any processing.
///
/// Returns `0` on success and `-1` on error.
///
/// ## Safety
///
/// - The `id_ptr` and `id_len` fields must be the start pointer and length of an
///   UTF8-encoded string.
/// - The `data_ptr` and `data_len` fields must be the start pointer and length
///   a byte array.
///
/// ## Thread Safety
///
/// This function is thread-safe. Multiple threads can call this function
/// concurrently with the same context pointer. The underlying node is protected
/// by a mutex to ensure thread-safe access.
int dora_send_output(void *dora_context, char *id_ptr, size_t id_len, char *data_ptr, size_t data_len);

#ifdef __cplusplus
} /* extern \"C\" */
#endif

#endif /* __RUST_DORA_NODE_API_C_WRAPPER__ */
