#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "node_api.h"

// sleep
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

int main()
{
    void *dora_context = init_dora_context_from_env();
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;
    }

    while (1)
    {
        void *input = dora_next_input(dora_context);
        if (input == NULL)
        {
            // end of input
            break;
        }

        char *id;
        size_t id_len;
        read_dora_input_id(input, &id, &id_len);

        char *data;
        size_t data_len;
        read_dora_input_data(input, &data, &data_len);

        char out_id[] = "foo";
        char out_data[] = "bar";
        dora_send_output(dora_context, out_id, strlen(out_id), out_data, strlen(out_data));

        free_dora_input(input); // do not use `id` or `data` pointer after freeing
    }

    free_dora_context(dora_context);

    return 0;
}
