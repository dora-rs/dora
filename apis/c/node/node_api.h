#include <stddef.h>

void *init_dora_context_from_env();
void free_dora_context(void *dora_context);

void *dora_next_event(void *dora_context);
void free_dora_event(void *dora_event);

enum DoraEventType
{
    DoraEventType_Stop,
    DoraEventType_Input,
    DoraEventType_InputClosed,
    DoraEventType_Error,
    DoraEventType_Unknown,
};
enum DoraEventType read_dora_event_type(void *dora_event);

void read_dora_input_id(void *dora_event, char **out_ptr, size_t *out_len);
void read_dora_input_data(void *dora_event, char **out_ptr, size_t *out_len);
unsigned long long read_dora_input_timestamp(void *dora_event);
int dora_send_output(void *dora_context, char *id_ptr, size_t id_len, char *data_ptr, size_t data_len);
