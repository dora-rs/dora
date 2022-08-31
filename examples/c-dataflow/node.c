#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "../../apis/c/node/node_api.h"

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

    for (char i = 0; i < 10; i++)
    {
        void *input = dora_next_input(dora_context);

        char *data;
        size_t data_len;
        read_dora_input_data(input, &data, &data_len);

        assert(data_len == 0);

        char out_id[] = "tick";
        dora_send_output(dora_context, out_id, strlen(out_id), &i, 1);

        free_dora_input(input);
    }

    free_dora_context(dora_context);

    return 0;
}
