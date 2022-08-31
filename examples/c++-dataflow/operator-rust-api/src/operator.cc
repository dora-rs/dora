#include "operator.h"
#include "cxx-dataflow-example-operator-rust-api/src/lib.rs.h"
#include <iostream>

Operator::Operator() {}

std::unique_ptr<Operator> new_operator()
{
    return std::make_unique<Operator>();
}

OnInputResult on_input(Operator &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
    op.counter += 1;
    std::cout << "Rust API operator received input `" << id << "` with data `" << (unsigned int)data[0] << "` (internal counter: " << (unsigned int)op.counter << ")" << std::endl;

    std::vector<unsigned char> out_vec{op.counter};
    rust::Slice<const uint8_t> out_slice{out_vec.data(), out_vec.size()};
    auto send_result = send_output(output_sender, rust::Str("status"), out_slice);
    OnInputResult result = {send_result.error, false};
    return result;
}
