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
        fprintf(stderr, "[c node] init dora context failed\n");
        return -1;
    }

    printf("[c node] dora context initialized\n");

    for (char i = 0; i < 10; i++)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            read_dora_input_data(event, &data, &data_len);

            assert(data_len == 0);

            char out_id[] = "speech";
            char out_data[] = "Hello World";

            dora_send_output(dora_context, out_id, strlen(out_id), out_data, strlen(out_data));
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }

    printf("[c node] talker 10 events\n");

    free_dora_context(dora_context);

    printf("[c node] finished successfully\n");

    return 0;
}
