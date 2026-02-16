extern "C"
{
#include "../../../apis/c/operator/operator_api.h"
}

#include <memory>
#include <iostream>
#include <vector>
#include <string.h>
#include <cassert>

class Operator
{
public:
    Operator();
};

Operator::Operator() {}

extern "C" AdoraInitResult_t adora_init_operator()
{
    Operator *op = std::make_unique<Operator>().release();

    AdoraInitResult_t result = {.operator_context = (void *)op};
    return result;
}

extern "C" AdoraResult_t adora_drop_operator(void *operator_context)
{
    delete (Operator *)operator_context;
    return {};
}

extern "C" OnEventResult_t adora_on_event(
    RawEvent_t *event,
    const SendOutput_t *send_output,
    void *operator_context)
{
    if (event->input != NULL)
    {
        // input event
        Input_t *input = event->input;
        char *id = adora_read_input_id(input);

        Vec_uint8_t data = adora_read_data(input);
        assert(data.ptr != NULL);

        std::cout
            << "C++ Operator (C-API) received input `" << id << "` with data: [";
        for (int i = 0; i < data.len; i++)
        {
            std::cout << (unsigned int)data.ptr[i] << ", ";
        }
        std::cout << "]" << std::endl;

        const char *out_id = "half-status";
        char *out_id_heap = strdup(out_id);

        size_t out_data_len = 1;
        uint8_t *out_data_heap = (uint8_t *)malloc(out_data_len);
        *out_data_heap = *data.ptr / 2;

        AdoraResult_t send_result = adora_send_operator_output(send_output, out_id_heap, out_data_heap, out_data_len);

        OnEventResult_t result = {.result = send_result, .status = ADORA_STATUS_CONTINUE};

        adora_free_data(data);
        adora_free_input_id(id);

        return result;
    }
    if (event->stop)
    {
        printf("C operator received stop event\n");
    }

    OnEventResult_t result = {.status = ADORA_STATUS_CONTINUE};
    return result;
}
