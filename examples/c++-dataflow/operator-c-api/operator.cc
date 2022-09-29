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

extern "C" OnInputResult_t dora_on_input(
    const Input_t *input,
    const PrepareOutput_t *prepare_output,
    void *operator_context)
{

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

    OutputMetadata_t output_meta = {.id = {
                                        .ptr = (uint8_t *)out_id_heap,
                                        .len = strlen(out_id_heap),
                                        .cap = strlen(out_id_heap) + 1,
                                    },
                                    .data_len = out_data_len};

    PrepareOutputResult_t prepare_result = (prepare_output->prepare_output.call)(prepare_output->prepare_output.env_ptr, output_meta);
    DoraResult_t dora_result = prepare_result.result;
    if (dora_result.error.len == 0)
    {
        auto output = prepare_result.output;
        auto out_data = output.data_mut.call(output.data_mut.env_ptr);
        *out_data.ptr = data[0] / 2;
        dora_result = output.send.call(output.send.env_ptr);
    }

    OnInputResult_t result = {.result = dora_result, .status = DORA_STATUS_CONTINUE};
    return result;
}
