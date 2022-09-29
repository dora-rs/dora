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

OnInputResult_t dora_on_input(
    const Input_t *input,
    const PrepareOutput_t *prepare_output,
    void *operator_context)
{
    char *counter = (char *)operator_context;

    char id[input->id.len + 1];
    memcpy(id, input->id.ptr, input->id.len);
    id[input->id.len] = 0;

    if (strcmp(id, "tick") == 0)
    {
        char data[input->data.len + 1];
        memcpy(data, input->data.ptr, input->data.len);
        data[input->data.len] = 0;

        *counter += 1;
        printf("C operator received tick input with data `%s`, counter: %i\n", data, *counter);

        char *out_id = "counter";
        char *out_id_heap = strdup(out_id);

        int data_alloc_size = 100;
        char *out_data = (char *)malloc(data_alloc_size);
        int count = snprintf(out_data, data_alloc_size, "The current counter value is %d", *counter);
        assert(count >= 0 && count < 100);

        OutputMetadata_t output_meta = {.id = {
                                            .ptr = (uint8_t *)out_id_heap,
                                            .len = strlen(out_id_heap),
                                            .cap = strlen(out_id_heap) + 1,
                                        },
                                        .data_len = strlen(out_data)};
        PrepareOutputResult_t res = (prepare_output->prepare_output.call)(prepare_output->prepare_output.env_ptr, output_meta);
        if (res.result.error.len == 0)
        {
            slice_raw_uint8_t data = res.output.data_mut.call(res.output.data_mut.env_ptr);
            memcpy(data.ptr, out_data, data.len);
            DoraResult_t send_result = res.output.send.call(res.output.send.env_ptr);

            OnInputResult_t result = {.result = send_result, .status = DORA_STATUS_CONTINUE};
            return result;
        }
        else
        {
            OnInputResult_t result = {.result = res.result, .status = DORA_STATUS_CONTINUE};
            return result;
        }
    }
    else
    {
        printf("C operator received unexpected input %s, context: %i\n", id, *counter);
        OnInputResult_t result = {.status = DORA_STATUS_CONTINUE};
        return result;
    }
}
