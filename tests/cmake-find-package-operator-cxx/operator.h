// Minimal `operator.h` consumed by the cxxbridge generated `.cc`
// (declared in `apis/c++/operator/src/lib.rs` via `include!("operator.h")`).
// The class declaration has to precede the cxxbridge header because
// the bridge types reference `::Operator`. The callback declarations
// have to follow the bridge header so `rust::Str` / `rust::Slice` /
// `OutputSender` / `DoraOnInputResult` resolve. Same shape as
// `examples/c++-dataflow/operator-rust-api/operator.h`.
#pragma once

#include <memory>

class Operator {
public:
    Operator() = default;
};

#include "dora-operator-api.h"

std::unique_ptr<Operator> new_operator();
DoraOnInputResult on_input(Operator &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender);
DoraOnInputResult on_input_closed(Operator &op, rust::Str id, OutputSender &output_sender);
DoraOnInputResult on_stop(Operator &op, OutputSender &output_sender);
