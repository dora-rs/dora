#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "../../apis/c/node/node_api.h"

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
            break;
        }

        char *id;
        size_t id_len;
        read_dora_input_id(input, &id, &id_len);

        char *data;
        size_t data_len;
        read_dora_input_data(input, &data, &data_len);

        printf("sink received input `");
        fwrite(id, id_len, 1, stdout);
        printf("` with data: '");
        fwrite(data, data_len, 1, stdout);
        printf("'\n");

        free_dora_input(input);
    }

    free_dora_context(dora_context);

    return 0;
}
