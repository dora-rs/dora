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
        fprintf(stderr, "failed to init adora context\n");
        return -1;
    }

    while (1)
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
            char *id;
            size_t id_len;
            read_adora_input_id(event, &id, &id_len);

            char *data;
            size_t data_len;
            read_adora_input_data(event, &data, &data_len);

            char out_id[] = "foo";
            char out_data[] = "bar";
            adora_send_output(adora_context, out_id, strlen(out_id), out_data, strlen(out_data));

            free_adora_event(event); // do not use `id` or `data` pointer after freeing
        }
    }

    free_adora_context(adora_context);

    return 0;
}
