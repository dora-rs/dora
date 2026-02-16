#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "../../apis/c/node/node_api.h"

int main()
{
    printf("[c sink] Hello World\n");

    void *adora_context = init_adora_context_from_env();
    if (adora_context == NULL)
    {
        fprintf(stderr, "failed to init adora context\n");
        return -1;
    }
    printf("[c sink] adora context initialized\n");

    while (1)
    {
        void *event = adora_next_event(adora_context);
        if (event == NULL)
        {
            printf("[c sink] end of event\n");
            break;
        }

        enum AdoraEventType ty = read_adora_event_type(event);

        if (ty == AdoraEventType_Input)
        {
            char *id;
            size_t id_len;
            read_adora_input_id(event, &id, &id_len);

            char *data;
            size_t data_len;
            read_adora_input_data(event, &data, &data_len);

            printf("[c sink] received input `");
            fwrite(id, id_len, 1, stdout);
            printf("` with data: %.*s\n", (int)data_len, data);
        }
        else if (ty == AdoraEventType_InputClosed)
        {
            printf("[c sink] received InputClosed event\n");
        }
        else if (ty == AdoraEventType_Stop)
        {
            printf("[c sink] received stop event\n");
        }
        else
        {
            printf("[c sink] received unexpected event: %d\n", ty);
        }

        free_adora_event(event);
    }

    free_adora_context(adora_context);

    printf("[c sink] finished successfully\n");

    return 0;
}
