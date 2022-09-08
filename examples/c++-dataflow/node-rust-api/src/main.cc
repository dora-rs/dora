#include "cxx-dataflow-example-node-rust-api/src/main.h"

#include <iostream>
#include <vector>

void cxx_main(Inputs &inputs, OutputSender &output_sender)
{
    std::cout << "HELLO FROM C++" << std::endl;
    unsigned char counter = 0;

    for (int i = 0; i < 20; i++)
    {

        auto input = next_input(inputs);
        if (input.end_of_input)
        {
            return;
        }
        counter += 1;

        std::cout << "Received input " << std::string(input.id) << " (counter: " << (unsigned int)counter << ")" << std::endl;

        std::vector<unsigned char> out_vec{counter};
        rust::Slice<const uint8_t> out_slice{out_vec.data(), out_vec.size()};
        auto result = send_output(output_sender, "counter", out_slice);
        auto error = std::string(result.error);
        if (!error.empty())
        {
            std::cerr << "Error: " << error << std::endl;
            return;
        }
    }
}
