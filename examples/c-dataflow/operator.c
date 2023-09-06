#include "../../apis/c/operator/operator_api.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

DoraInitResult_t dora_init_operator(void)
{
    void *context = malloc(1);
    char *context_char = (char *)context;
    *context_char = 0;

    DoraInitResult_t result = {.operator_context = context};
    return result;
}

DoraResult_t dora_drop_operator(void *operator_context)
{
    free(operator_context);

    DoraResult_t result = {};
    return result;
}

OnEventResult_t dora_on_event(
    RawEvent_t *event,
    const SendOutput_t *send_output,
    void *operator_context)
{
    char *counter = (char *)operator_context;

    if (event->input != NULL)
    {
        // input event
        Input_t *input = event->input;

        char id[input->id.len + 1];
        memcpy(id, input->id.ptr, input->id.len);
        id[input->id.len] = 0;

        if (strcmp(id, "message") == 0)
        {
            char data[input->data.len + 1];
            memcpy(data, input->data.ptr, input->data.len);
            data[input->data.len] = 0;

            *counter += 1;
            printf("C operator received message `%s`, counter: %i\n", data, *counter);

            char *out_id = "counter";
            char *out_id_heap = strdup(out_id);

            int data_alloc_size = 100;
            char *out_data = (char *)malloc(data_alloc_size);
            int count = snprintf(out_data, data_alloc_size, "The current counter value is %d", *counter);
            assert(count >= 0 && count < 100);

            Output_t output = {.id = {
                                   .ptr = (uint8_t *)out_id_heap,
                                   .len = strlen(out_id_heap),
                                   .cap = strlen(out_id_heap) + 1,
                               },
                               .data = {.ptr = (uint8_t *)out_data, .len = strlen(out_data), .cap = data_alloc_size}};
            DoraResult_t res = (send_output->send_output.call)(send_output->send_output.env_ptr, output);

            OnEventResult_t result = {.result = res, .status = DORA_STATUS_CONTINUE};
            return result;
        }
        else
        {
            printf("C operator received unexpected input %s, context: %i\n", id, *counter);
        }
    }
    if (event->stop)
    {
        printf("C operator received stop event\n");
    }

    OnEventResult_t result = {.status = DORA_STATUS_CONTINUE};
    return result;
}
