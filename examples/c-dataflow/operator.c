#include "build/operator_api.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

int dora_init_operator(void **operator_context)
{
    void *context = malloc(1);
    char *context_char = (char *)context;
    *context_char = 0;

    *operator_context = context;

    return 0;
}

void dora_drop_operator(void *operator_context)
{
    free(operator_context);
}

int dora_on_input(
    const char *id_start,
    size_t id_len,
    const char *data_start,
    size_t data_len,
    const int (*output_fn_raw)(const char *id_start,
                               size_t id_len,
                               const char *data_start,
                               size_t data_len,
                               const void *output_context),
    void *output_context,
    const void *operator_context)
{
    char *counter = (char *)operator_context;

    char id[id_len + 1];
    memcpy(id, id_start, id_len);
    id[id_len] = 0;

    if (strcmp(id, "tick") == 0)
    {
        char data[data_len + 1];
        memcpy(data, data_start, data_len);
        data[data_len] = 0;

        *counter += 1;
        printf("C operator received tick input with data `%s`, counter: %i\n", data, *counter);

        char *out_id = "counter";

        char out_data[100];
        int count = snprintf(out_data, sizeof(out_data), "The current counter value is %d", *counter);
        assert(count >= 0 && count < 100);

        int res = (output_fn_raw)(out_id, strlen(out_id), out_data, strlen(out_data), output_context);
        if (res != 0)
        {
            printf("C operator failed to send output\n");
        }
    }
    else
    {
        printf("C operator received unexpected input %s, context: %i\n", id, *counter);
    }

    return 0;
}
