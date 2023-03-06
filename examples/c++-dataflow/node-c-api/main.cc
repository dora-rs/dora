extern "C"
{
#include "../../../apis/c/node/node_api.h"
}

#include <iostream>
#include <vector>

int run(void *dora_context)
{
    unsigned char counter = 0;

    for (int i = 0; i < 20; i++)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            counter += 1;

            char *id_ptr;
            size_t id_len;
            read_dora_input_id(event, &id_ptr, &id_len);
            std::string id(id_ptr, id_len);

            char *data_ptr;
            size_t data_len;
            read_dora_input_data(event, &data_ptr, &data_len);
            std::vector<unsigned char> data;
            for (size_t i = 0; i < data_len; i++)
            {
                data.push_back(*(data_ptr + i));
            }

            std::cout
                << "Received input "
                << " (counter: " << (unsigned int)counter << ") data: [";
            for (unsigned char &v : data)
            {
                std::cout << (unsigned int)v << ", ";
            }
            std::cout << "]" << std::endl;

            std::vector<unsigned char> out_vec{counter};
            std::string out_id = "counter";
            int result = dora_send_output(dora_context, &out_id[0], out_id.length(), (char *)&counter, 1);
            if (result != 0)
            {
                std::cerr << "failed to send output" << std::endl;
                return 1;
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "HELLO FROM C++ (using C API)" << std::endl;

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE FROM C++ node (using C API)" << std::endl;

    return ret;
}
