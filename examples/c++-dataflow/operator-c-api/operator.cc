extern "C"
{
#include "../../../apis/c/operator/operator_api.h"
}

#include <memory>
#include <iostream>
#include <vector>

class Operator
{
public:
    Operator();
};

Operator::Operator() {}

extern "C" int dora_init_operator(void **operator_context)
{
    Operator *op = std::make_unique<Operator>().release();
    *operator_context = (void *)op;

    return 0;
}

extern "C" void dora_drop_operator(void *operator_context)
{
    delete (Operator *)operator_context;
}

extern "C" int dora_on_input(
    const char *id_start,
    size_t id_len,
    const char *data_start,
    size_t data_len,
    const int (*output_fn_raw)(const char *id_start,
                               size_t id_len,
                               const char *data_start,
                               size_t data_len,
                               const void *output_context),
    void *output_context,
    const void *operator_context)
{

    std::string id(id_start, id_len);

    std::vector<unsigned char> data;
    for (size_t i = 0; i < data_len; i++)
    {
        data.push_back(*(data_start + i));
    }

    std::cout
        << "C++ Operator (C-API) received input `" << id << "` with data: [";
    for (unsigned char &v : data)
    {
        std::cout << (unsigned int)v << ", ";
    }
    std::cout << "]" << std::endl;

    char out = data[0] / 2;
    std::string out_id = "half-status";
    int result = output_fn_raw(&out_id[0], out_id.length(), &out, 1, output_context);
    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
        return 1;
    }

    return 0;
}
