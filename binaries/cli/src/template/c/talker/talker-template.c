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
    void *adora_context = init_adora_context_from_env();
    if (adora_context == NULL)
    {
        fprintf(stderr, "[c node] init adora context failed\n");
        return -1;
    }

    printf("[c node] adora context initialized\n");

    for (char i = 0; i < 10; i++)
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

            char out_id[] = "speech";
            char out_data[] = "Hello World";

            adora_send_output(adora_context, out_id, strlen(out_id), out_data, strlen(out_data));
        }
        else if (ty == AdoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
            free_adora_event(event);
            break;
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
            free_adora_event(event);
            break;
        }

        free_adora_event(event);
    }

    printf("[c node] talker 10 events\n");

    free_adora_context(adora_context);

    printf("[c node] finished successfully\n");

    return 0;
}
