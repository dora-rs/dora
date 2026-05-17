#include "operator.h"
#include <iostream>
#include <vector>
#include "dora-operator-api.h"

Operator::Operator() {}

std::unique_ptr<Operator> new_operator()
{
    return std::make_unique<Operator>();
}

DoraOnInputResult on_input(Operator &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
    op.counter += 1;
    std::cout << "Rust API operator received input `" << id.data() << "` with data `" << (unsigned int)data[0] << "` (internal counter: " << (unsigned int)op.counter << ")" << std::endl;

    std::vector<unsigned char> out_vec{op.counter};
    rust::Slice<const uint8_t> out_slice{out_vec.data(), out_vec.size()};
    auto send_result = send_output(output_sender, rust::Str("status"), out_slice);
    DoraOnInputResult result = {send_result.error, false};
    return result;
}

DoraOnInputResult on_input_closed(Operator &op, rust::Str id)
{
    std::cout << "Rust API operator: input `" << std::string(id) << "` closed" << std::endl;
    DoraOnInputResult result = {rust::String(), false};
    return result;
}

DoraOnInputResult on_stop(Operator &op)
{
    std::cout << "Rust API operator: stop received (counter was " << (unsigned int)op.counter << ")" << std::endl;
    DoraOnInputResult result = {rust::String(), false};
    return result;
}
