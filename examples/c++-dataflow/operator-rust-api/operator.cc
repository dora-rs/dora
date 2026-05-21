#include "operator.h"
#include <iostream>
#include <vector>
#include "../build/dora-operator-api.h"

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

DoraOnInputResult on_input_closed(Operator &op, rust::Str id, OutputSender &output_sender)
{
    (void)output_sender;  // Example doesn't need to emit on input close.
    std::cout << "Rust API operator: input `" << std::string(id) << "` closed" << std::endl;
    DoraOnInputResult result = {rust::String(), false};
    return result;
}

DoraOnInputResult on_stop(Operator &op, OutputSender &output_sender)
{
    (void)output_sender;  // Example doesn't emit a final output; a real
                          // operator might call `send_output(output_sender, ...)`
                          // here to flush buffered state before shutdown.
    std::cout << "Rust API operator: stop received (counter was " << (unsigned int)op.counter << ")" << std::endl;
    DoraOnInputResult result = {rust::String(), false};
    return result;
}

DoraOnInputResult on_input_parse_error(Operator &op, rust::Str id, rust::Str error, OutputSender &output_sender)
{
    (void)op;
    (void)output_sender;
    std::cerr << "Rust API operator: input parse error on `" << std::string(id) << "`: " << std::string(error) << std::endl;
    DoraOnInputResult result = {rust::String(), false};
    return result;
}
