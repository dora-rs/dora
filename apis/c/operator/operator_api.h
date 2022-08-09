int dora_init_operator(void **operator_context);

void dora_drop_operator(void *operator_context);

int dora_on_input(
    const char *id_start,
    unsigned int id_len,
    const char *data_start,
    unsigned int data_len,
    const int (*output_fn_raw)(const char *id_start,
                               unsigned int id_len,
                               const char *data_start,
                               unsigned int data_len,
                               const void *output_context),
    void *output_context,
    const void *operator_context);
