#include "operator_api.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

AdoraInitResult_t adora_init_operator(void)
{
    // allocate memory for storing context across function calls (optional)
    void *context = malloc(10);
    // TODO initialize context memory

    AdoraInitResult_t result = {.operator_context = context};
    return result;
}

AdoraResult_t adora_drop_operator(void *operator_context)
{
    free(operator_context);

    AdoraResult_t result = {};
    return result;
}

OnEventResult_t adora_on_event(
    RawEvent_t *event,
    const SendOutput_t *send_output,
    void *operator_context)
{
    if (event->input != NULL)
    {
        char id[event->input->id.len + 1];
        memcpy(id, event->input->id.ptr, event->input->id.len);
        id[event->input->id.len] = 0;

        // example for matching on input name
        if (strcmp(id, "foo") == 0)
        {
            char *out_id = "bar";
            char *out_id_heap = strdup(out_id);

            int data_alloc_size = 10;
            void *out_data = malloc(data_alloc_size);
            // TODO initialize out_data

            Output_t output = {.id = {
                                   .ptr = (uint8_t *)out_id_heap,
                                   .len = strlen(out_id_heap),
                                   .cap = strlen(out_id_heap) + 1,
                               },
                               .data = {.ptr = (uint8_t *)out_data, .len = strlen(out_data), .cap = data_alloc_size}};
            AdoraResult_t res = (send_output->send_output.call)(send_output->send_output.env_ptr, output);

            OnEventResult_t result = {.result = res, .status = ADORA_STATUS_CONTINUE};
            return result;
        }
    }
    if (event->stop)
    {
        printf("C operator received stop event\n");
    }
    OnEventResult_t result = {.status = ADORA_STATUS_CONTINUE};
    return result;
}
