#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "../../apis/c/node/node_api.h"

int main()
{
    printf("[c sink] Hello World\n");

    void *dora_context = init_dora_context_from_env();
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;

        printf("[c sink] dora context initialized\n");
    }

    while (1)
    {
        printf("[c sink] waiting for next input\n");
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c sink] end of event\n");
            break;
        }

        enum EventType ty = read_dora_event_type(event);

        if (ty == Input)
        {
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);

            char *data;
            size_t data_len;
            read_dora_input_data(event, &data, &data_len);

            printf("[c sink] received input `");
            fwrite(id, id_len, 1, stdout);
            printf("` with data: %d\n", *data);
        }
        else if (ty == InputClosed)
        {
            printf("[c sink] received InputClosed event\n");
            free_dora_event(event);
            break;
        }
        else if (ty == Stop)
        {
            printf("[c sink] received stop event\n");
            free_dora_event(event);
            break;
        }
        else
        {
            printf("[c sink] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }

    free_dora_context(dora_context);

    printf("[c sink] finished successfully\n");

    return 0;
}
