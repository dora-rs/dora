extern "C"
{
#include "../../../apis/c/operator/operator_api.h"
}

#include <memory>
#include <iostream>
#include <vector>
#include <string.h>

class Operator
{
public:
    Operator();
};

Operator::Operator() {}

extern "C" DoraInitResult_t dora_init_operator()
{
    Operator *op = std::make_unique<Operator>().release();

    DoraInitResult_t result = {.operator_context = (void *)op};
    return result;
}

extern "C" DoraResult_t dora_drop_operator(void *operator_context)
{
    delete (Operator *)operator_context;
    return {};
}

extern "C" OnEventResult_t dora_on_event(
    const RawEvent_t *event,
    const SendOutput_t *send_output,
    void *operator_context)
{
    if (event->input != NULL)
    {
        // input event
        Input_t *input = event->input;
        std::string id((char *)input->id.ptr, input->id.len);

        std::vector<unsigned char> data;
        for (size_t i = 0; i < input->data.len; i++)
        {
            data.push_back(*(input->data.ptr + i));
        }

        std::cout
            << "C++ Operator (C-API) received input `" << id << "` with data: [";
        for (unsigned char &v : data)
        {
            std::cout << (unsigned int)v << ", ";
        }
        std::cout << "]" << std::endl;

        const char *out_id = "half-status";
        char *out_id_heap = strdup(out_id);

        size_t out_data_len = 1;
        uint8_t *out_data_heap = (uint8_t *)malloc(out_data_len);
        *out_data_heap = data[0] / 2;

        Output_t output = {.id = {
                               .ptr = (uint8_t *)out_id_heap,
                               .len = strlen(out_id_heap),
                               .cap = strlen(out_id_heap) + 1,
                           },
                           .data = {.ptr = out_data_heap, .len = out_data_len, .cap = out_data_len}};

        DoraResult_t send_result = (send_output->send_output.call)(send_output->send_output.env_ptr, output);

        OnEventResult_t result = {.result = send_result, .status = DORA_STATUS_CONTINUE};
        return result;
    }
    if (event->stop)
    {
        printf("C operator received stop event\n");
    }

    OnEventResult_t result = {.status = DORA_STATUS_CONTINUE};
    return result;
}
