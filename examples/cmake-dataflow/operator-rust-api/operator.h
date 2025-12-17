#pragma once
#include <memory>
#include "operator/operator_api.h"

class Operator
{
public:
    Operator();
    unsigned char counter;
};

#include "dora-operator-api.h"

std::unique_ptr<Operator> new_operator();

DoraOnInputResult on_input(Operator &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender);
