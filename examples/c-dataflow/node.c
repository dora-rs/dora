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

    void *adora_context = init_adora_context_from_env();
    if (adora_context == NULL)
    {
        fprintf(stderr, "failed to init adora context\n");
        return -1;
    }

    printf("[c node] adora context initialized\n");

    for (char i = 0; i < 100; i++)
    {
        void *event = adora_next_event(adora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum AdoraEventType ty = read_adora_event_type(event);

        if (ty == AdoraEventType_Input)
        {
            char *data;
            size_t data_len;
            read_adora_input_data(event, &data, &data_len);

            assert(data_len == 0);

            char out_id[] = "message";
            char out_data[50];
            int out_data_len = sprintf(out_data, "loop iteration %d", i);

            adora_send_output(adora_context, out_id, strlen(out_id), out_data, out_data_len);
        }
        else if (ty == AdoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_adora_event(event);
    }

    printf("[c node] received 10 events\n");

    free_adora_context(adora_context);

    printf("[c node] finished successfully\n");

    return 0;
}
