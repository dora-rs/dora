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
int adora_send_output(void *adora_context, char *id_ptr, size_t id_len, char *data_ptr, size_t data_len);
