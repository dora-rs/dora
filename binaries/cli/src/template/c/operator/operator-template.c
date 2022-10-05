#include "operator_api.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

DoraInitResult_t dora_init_operator(void)
{
    // allocate memory for storing context across function calls (optional)
    void *context = malloc(10);
    // TODO initialize context memory

    DoraInitResult_t result = {.operator_context = context};
    return result;
}

DoraResult_t dora_drop_operator(void *operator_context)
{
    free(operator_context);

    DoraResult_t result = {};
    return result;
}

OnInputResult_t dora_on_input(
    const Input_t *input,
    const SendOutput_t *send_output,
    void *operator_context)
{
    char id[input->id.len + 1];
    memcpy(id, input->id.ptr, input->id.len);
    id[input->id.len] = 0;

    // example for matching on input name
    if (strcmp(id, "foo") == 0)
    {
        char *out_id = "bar";
        char *out_id_heap = strdup(out_id);

        int data_alloc_size = 10;
        void *out_data = malloc(data_alloc_size);
        // TODO intialize out_data

        Output_t output = {.id = {
                               .ptr = (uint8_t *)out_id_heap,
                               .len = strlen(out_id_heap),
                               .cap = strlen(out_id_heap) + 1,
                           },
                           .data = {.ptr = (uint8_t *)out_data, .len = strlen(out_data), .cap = data_alloc_size}};
        DoraResult_t res = (send_output->send_output.call)(send_output->send_output.env_ptr, output);

        OnInputResult_t result = {.result = res, .status = DORA_STATUS_CONTINUE};
        return result;
    }
    else
    {
        OnInputResult_t result = {.status = DORA_STATUS_CONTINUE};
        return result;
    }
}
