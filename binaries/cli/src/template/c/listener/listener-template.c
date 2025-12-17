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

    for (char i = 0; i < 20; i++)
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
            char *id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);

            char *data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);

            unsigned long long timestamp = read_dora_input_timestamp(event);
            printf("I heard %s from %.*s at %llu\n", data_ptr, (int)id_len, id_ptr, timestamp);
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
            free_dora_event(event);
            break;
        }
        else if (ty == DoraEventType_InputClosed) {
            printf("[c node] received input closed event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
            free_dora_event(event);
            break;
        }

        free_dora_event(event);
    }

    free_dora_context(dora_context);

    return 0;
}
