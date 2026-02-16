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

    for (char i = 0; i < 20; i++)
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
            char *id_ptr;
            size_t id_len;
            read_adora_input_id(event, &id_ptr, &id_len);

            char *data_ptr;
            size_t data_len;
            read_adora_input_data(event, &data_ptr, &data_len);

            unsigned long long timestamp = read_adora_input_timestamp(event);
            printf("I heard %s from %.*s at %llu\n", data_ptr, (int)id_len, id_ptr, timestamp);
        }
        else if (ty == AdoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
            free_adora_event(event);
            break;
        }
        else if (ty == AdoraEventType_InputClosed) {
            printf("[c node] received input closed event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
            free_adora_event(event);
            break;
        }

        free_adora_event(event);
    }

    free_adora_context(adora_context);

    return 0;
}
