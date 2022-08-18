#pragma once
#include "rust/cxx.h"
#include "cxx-dataflow-example-operator-rust-api/src/lib.rs.h"
#include <memory>

class Operator
{
public:
    Operator();
    unsigned char counter;
};

std::unique_ptr<Operator> new_operator();

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(Operator &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);
