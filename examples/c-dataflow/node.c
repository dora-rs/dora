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
    printf("[c node] Hello World\n");

    void *dora_context = init_dora_context_from_env();
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;
    }

    printf("[c node] dora context initialized\n");

    for (char i = 0; i < 100; i++)
    {
        printf("[c node] waiting for next input\n");
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum EventType ty = read_dora_event_type(event);

        if (ty == Input)
        {
            char *data;
            size_t data_len;
            read_dora_input_data(event, &data, &data_len);

            assert(data_len == 0);

            char out_id[] = "counter";
            dora_send_output(dora_context, out_id, strlen(out_id), &i, 1);
        }
        else if (ty == Stop)
        {
            printf("[c node] received stop event\n");
            free_dora_event(event);
            break;
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }

    printf("[c node] received 10 events\n");

    free_dora_context(dora_context);

    printf("[c node] finished successfully\n");

    return 0;
}
