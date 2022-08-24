#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "build/node_api.h"

// sleep
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

int main()
{
    printf("[c node] Hello World\n");

    void *dora_context = init_dora_context_from_env();
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;
    }

    printf("[c node] dora context initialized\n");

    for (char i = 0; i < 10; i++)
    {
        printf("[c node] waiting for next input\n");
        void *input = dora_next_input(dora_context);
        if (input == NULL)
        {
            printf("[c node] ERROR: unexpected end of input\n");
            return -1;
        }

        char *data;
        size_t data_len;
        read_dora_input_data(input, &data, &data_len);

        assert(data_len == 0);

        char out_id[] = "tick";
        dora_send_output(dora_context, out_id, strlen(out_id), &i, 1);

        free_dora_input(input);
    }

    printf("[c node] received 10 inputs\n");

    free_dora_context(dora_context);

    printf("[c node] finished successfully\n");

    return 0;
}
