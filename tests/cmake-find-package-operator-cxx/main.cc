// Smoke-test consumer for `find_package(dora-operator-api-cxx)`.
// The dora operator runtime is callback-shaped: the static lib +
// cxxbridge .cc reference these symbols and the consumer supplies
// them. We provide no-op implementations so cmake configures,
// compiles the cxxbridge .cc, and links a shared library against
// `dora-operator-api-cxx::dora-operator-api-cxx` — that resolves
// the full find_package round-trip.

// operator.h already includes dora-operator-api.h, so a single include
// here pulls in both the Operator class and the callback declarations
// that the cxxbridge .cc references at global scope.
#include "operator.h"

std::unique_ptr<Operator> new_operator() {
    return std::make_unique<Operator>();
}

DoraOnInputResult on_input(Operator &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender) {
    (void)op;
    (void)id;
    (void)data;
    (void)output_sender;
    DoraOnInputResult result = {rust::String(), false};
    return result;
}

DoraOnInputResult on_input_closed(Operator &op, rust::Str id, OutputSender &output_sender) {
    (void)op;
    (void)id;
    (void)output_sender;
    DoraOnInputResult result = {rust::String(), false};
    return result;
}

DoraOnInputResult on_stop(Operator &op, OutputSender &output_sender) {
    (void)op;
    (void)output_sender;
    DoraOnInputResult result = {rust::String(), false};
    return result;
}
