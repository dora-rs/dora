#include <stddef.h>

void *init_dora_context_from_env();
void free_dora_context(void *dora_context);

void *dora_next_input(void *dora_context);
void read_dora_input_id(void *dora_input, char **out_ptr, size_t *out_len);
void read_dora_input_data(void *dora_input, char **out_ptr, size_t *out_len);
void free_dora_input(void *dora_input);

int dora_send_output(void *dora_context, char *id_ptr, size_t id_len, char *data_ptr, size_t data_len);
