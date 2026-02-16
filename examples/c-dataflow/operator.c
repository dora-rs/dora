#include "../../apis/c/operator/operator_api.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

AdoraInitResult_t adora_init_operator(void)
{
    void *context = malloc(1);
    char *context_char = (char *)context;
    *context_char = 0;

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
    OnEventResult_t result = {.status = ADORA_STATUS_CONTINUE};

    char *counter = (char *)operator_context;

    if (event->input != NULL)
    {
        // input event
        Input_t *input = event->input;

        char *id = adora_read_input_id(input);

        if (strcmp(id, "message") == 0)
        {
            printf("message event\n");

            Vec_uint8_t data = adora_read_data(input);
            assert(data.ptr != NULL);

            *counter += 1;
            printf("C operator received message `%.*s`, counter: %i\n", (int)data.len, data.ptr, *counter);

            char *out_id = "counter";
            char *out_id_heap = strdup(out_id);

            int data_alloc_size = 100;
            char *out_data = (char *)malloc(data_alloc_size);
            int count = snprintf(out_data, data_alloc_size, "The current counter value is %d", *counter);
            assert(count >= 0 && count < 100);

            AdoraResult_t res = adora_send_operator_output(send_output, out_id_heap, (uint8_t *)out_data, strlen(out_data));
            result.result = res;

            adora_free_data(data);
        }
        else
        {
            printf("C operator received unexpected input %s, context: %i\n", id, *counter);
        }

        adora_free_input_id(id);
    }
    if (event->stop)
    {
        printf("C operator received stop event\n");
    }

    return result;
}
