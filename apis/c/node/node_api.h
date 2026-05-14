#include <stddef.h>

/*
 * dora C node API
 *
 * ## Threading model
 *
 * The dora context (returned by init_dora_context_from_env) and any event
 * pointer (returned by dora_next_event) are NOT internally synchronized.
 * Each function below is annotated with its thread-safety contract.
 *
 * Rule of thumb:
 *   - A single context pointer must be accessed by at most one thread at a
 *     time. The dora-cli has not yet stabilized a Sync-safe context, so
 *     calling dora_next_event / dora_send_output / dora_log concurrently
 *     with the same context is undefined behavior.
 *   - Event pointers are read-only after creation, so multiple threads
 *     may read fields from the same event concurrently (each thread
 *     supplying its own out_ptr / out_len storage).
 *   - free_dora_context and free_dora_event take ownership; the caller
 *     must guarantee no other thread is touching the context / event when
 *     free is called.
 *
 * See dora-rs/dora#540 for the audit that produced these annotations.
 */

/* Thread-safe: yes. Creates a fresh dora context from env vars; no shared
 * state at entry. Typically called once at node startup. */
void *init_dora_context_from_env();

/* Thread-safe: NO. Takes ownership of the context and drops it. The caller
 * must guarantee no other thread is using `dora_context` when this is
 * called, and the pointer must not be reused afterward. */
void free_dora_context(void *dora_context);

/* Thread-safe: NO. Mutates the context's internal event-stream cursor.
 * Concurrent calls with the same `dora_context` are undefined behavior.
 * If you need to fan out events to worker threads, drain events from a
 * single thread and dispatch by event type. */
void *dora_next_event(void *dora_context);

/* Thread-safe: NO. Takes ownership of `dora_event` and drops it. After
 * freeing, neither the event pointer nor any pointer returned by the
 * read_dora_event_* / read_dora_input_* family for that event may be
 * used. */
void free_dora_event(void *dora_event);

enum DoraEventType
{
    DoraEventType_Stop,
    DoraEventType_Input,
    DoraEventType_InputClosed,
    DoraEventType_Error,
    DoraEventType_Unknown,
};
/* Thread-safe: yes (read-only). The returned enum value is by-value; no
 * state is mutated on the event. */
enum DoraEventType read_dora_event_type(void *dora_event);

/* Thread-safe: yes (read-only on `dora_event`). The caller is responsible
 * for ensuring the `out_ptr` / `out_len` destinations are not aliased by
 * other concurrent calls. */
void read_dora_input_id(void *dora_event, char **out_ptr, size_t *out_len);

/* Thread-safe: yes (read-only on `dora_event`). Same caveat as
 * read_dora_input_id about caller-owned out destinations. */
void read_dora_input_data(void *dora_event, char **out_ptr, size_t *out_len);

/* Thread-safe: yes (read-only). Returns by value. */
unsigned long long read_dora_input_timestamp(void *dora_event);

/* Thread-safe: NO. Mutates the context's internal sender state. Concurrent
 * calls with the same `dora_context` are undefined behavior. */
int dora_send_output(void *dora_context, const char *id_ptr, size_t id_len, const char *data_ptr, size_t data_len);

/* Thread-safe: NO. Mutates the context's internal logger state. Concurrent
 * calls with the same `dora_context` are undefined behavior. */
int dora_log(void *dora_context, const char *level_ptr, size_t level_len, const char *msg_ptr, size_t msg_len);

/* Backward-compatible aliases for dora-hub C nodes. */
#define init_dora_context_from_env  init_dora_context_from_env
#define free_dora_context           free_dora_context
#define dora_next_event             dora_next_event
#define free_dora_event             free_dora_event
#define read_dora_event_type        read_dora_event_type
#define read_dora_input_id          read_dora_input_id
#define read_dora_input_data        read_dora_input_data
#define read_dora_input_timestamp   read_dora_input_timestamp
#define dora_send_output            dora_send_output
#define dora_log                    dora_log

typedef enum DoraEventType DoraEventType;
#define DoraEventType_Stop        DoraEventType_Stop
#define DoraEventType_Input       DoraEventType_Input
#define DoraEventType_InputClosed DoraEventType_InputClosed
#define DoraEventType_Error       DoraEventType_Error
#define DoraEventType_Unknown     DoraEventType_Unknown
