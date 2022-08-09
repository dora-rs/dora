#include "operator_api.h"
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
    unsigned int id_len,
    const char *data_start,
    unsigned int data_len,
    const int (*output_fn_raw)(const char *id_start,
                               unsigned int id_len,
                               const char *data_start,
                               unsigned int data_len,
                               const void *output_context),
    void *output_context,
    const void *operator_context)
{
    char *context = (char *)operator_context;

    char id[id_len + 1];
    memcpy(id, id_start, id_len);
    id[id_len] = 0;

    if (strcmp(id, "time") == 0)
    {
        char time[data_len + 1];
        memcpy(time, data_start, data_len);
        time[data_len] = 0;

        printf("C operator received time input %s, context: %i\n", time, *context);
        *context += 1;

        char *out_id = "counter";

        int res = (output_fn_raw)(out_id, strlen(out_id), context, 1, output_context);
        if (res != 0)
        {
            printf("C operator failed to send output\n");
        }
    }
    else
    {
        printf("C operator received unexpected input %s, context: %i\n", id, *context);
    }

    return 0;
}
