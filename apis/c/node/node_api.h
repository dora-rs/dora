#include <stddef.h>

void *init_adora_context_from_env();
void free_adora_context(void *adora_context);

void *adora_next_event(void *adora_context);
void free_adora_event(void *adora_event);

enum AdoraEventType
{
    AdoraEventType_Stop,
    AdoraEventType_Input,
    AdoraEventType_InputClosed,
    AdoraEventType_Error,
    AdoraEventType_Unknown,
};
enum AdoraEventType read_adora_event_type(void *adora_event);

void read_adora_input_id(void *adora_event, char **out_ptr, size_t *out_len);
void read_adora_input_data(void *adora_event, char **out_ptr, size_t *out_len);
unsigned long long read_adora_input_timestamp(void *adora_event);
int adora_send_output(void *adora_context, const char *id_ptr, size_t id_len, const char *data_ptr, size_t data_len);
int adora_log(void *adora_context, const char *level_ptr, size_t level_len, const char *msg_ptr, size_t msg_len);

/* Backward-compatible aliases for dora-hub C nodes. */
#define init_dora_context_from_env  init_adora_context_from_env
#define free_dora_context           free_adora_context
#define dora_next_event             adora_next_event
#define free_dora_event             free_adora_event
#define read_dora_event_type        read_adora_event_type
#define read_dora_input_id          read_adora_input_id
#define read_dora_input_data        read_adora_input_data
#define read_dora_input_timestamp   read_adora_input_timestamp
#define dora_send_output            adora_send_output
#define dora_log                    adora_log

typedef enum AdoraEventType DoraEventType;
#define DoraEventType_Stop        AdoraEventType_Stop
#define DoraEventType_Input       AdoraEventType_Input
#define DoraEventType_InputClosed AdoraEventType_InputClosed
#define DoraEventType_Error       AdoraEventType_Error
#define DoraEventType_Unknown     AdoraEventType_Unknown
