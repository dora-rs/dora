#include "dora-node-api.h"
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// Arrow FFI helpers — wrap the raw pointer dance so user code stays clean.
// These mirror pyarrow's Array._import_from_c / _export_to_c pattern.
// ---------------------------------------------------------------------------

/// Receive an Arrow array from a Dora input event.
/// Returns {array, input_id} or {nullptr, ""} on error.
struct ArrowInput {
    std::shared_ptr<arrow::Array> array;
    std::string id;
};

ArrowInput receive_arrow(rust::cxxbridge1::Box<DoraEvent> event) {
    struct ArrowArray c_array;
    struct ArrowSchema c_schema;

    auto info = event_as_arrow_input_with_info(
        std::move(event),
        reinterpret_cast<uint8_t *>(&c_array),
        reinterpret_cast<uint8_t *>(&c_schema));

    if (!info.error.empty()) {
        std::cerr << "receive_arrow error: " << std::string(info.error)
                  << std::endl;
        return {nullptr, ""};
    }

    auto result = arrow::ImportArray(&c_array, &c_schema);
    if (!result.ok()) {
        std::cerr << "ImportArray failed: " << result.status().ToString()
                  << std::endl;
        return {nullptr, ""};
    }
    return {result.MoveValueUnsafe(), std::string(info.id)};
}

/// Send an Arrow array as a Dora output.
bool send_arrow(rust::cxxbridge1::Box<OutputSender> &sender,
                const std::string &output_id,
                const std::shared_ptr<arrow::Array> &array) {
    struct ArrowArray c_array;
    struct ArrowSchema c_schema;

    auto status = arrow::ExportArray(*array, &c_array, &c_schema);
    if (!status.ok()) {
        std::cerr << "ExportArray failed: " << status.ToString() << std::endl;
        return false;
    }

    auto result = send_arrow_output(
        sender, output_id,
        reinterpret_cast<uint8_t *>(&c_array),
        reinterpret_cast<uint8_t *>(&c_schema));

    if (!result.error.empty()) {
        std::cerr << "send_arrow error: " << std::string(result.error)
                  << std::endl;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Example builders for different Arrow types
// ---------------------------------------------------------------------------

std::shared_ptr<arrow::Array> build_int32_array() {
    arrow::Int32Builder builder;
    (void)builder.AppendValues({10, 20, 30, 40, 50});
    auto result = builder.Finish();
    return result.ValueOrDie();
}

std::shared_ptr<arrow::Array> build_float64_array() {
    arrow::DoubleBuilder builder;
    (void)builder.AppendValues({1.1, 2.2, 3.3, 4.4});
    auto result = builder.Finish();
    return result.ValueOrDie();
}

std::shared_ptr<arrow::Array> build_string_array() {
    arrow::StringBuilder builder;
    (void)builder.Append("hello");
    (void)builder.Append("from");
    (void)builder.Append("dora");
    (void)builder.Append("c++");
    auto result = builder.Finish();
    return result.ValueOrDie();
}

// ---------------------------------------------------------------------------

int main() {
    auto dora_node = init_dora_node();
    auto id = std::string(node_id(dora_node.send_output));
    std::cout << "[" << id << "] started" << std::endl;

    int counter = 0;

    for (int i = 0; i < 100; i++) {
        auto event = dora_node.events->next();
        auto ty = event_type(event);

        if (ty == DoraEventType::Stop ||
            ty == DoraEventType::AllInputsClosed) {
            break;
        }

        if (ty != DoraEventType::Input) {
            continue;
        }

        // ---- Receive & print the incoming Arrow array ----
        auto input = receive_arrow(std::move(event));
        if (input.array) {
            std::cout << "[" << id << "] received '" << input.id
                      << "': type=" << input.array->type()->ToString()
                      << ", length=" << input.array->length()
                      << ", data=" << input.array->ToString() << std::endl;
        }

        // ---- Send a different Arrow type each tick ----
        std::shared_ptr<arrow::Array> output;
        switch (counter % 3) {
        case 0:
            output = build_int32_array();
            break;
        case 1:
            output = build_float64_array();
            break;
        case 2:
            output = build_string_array();
            break;
        }
        counter++;

        std::cout << "[" << id << "] sending type="
                  << output->type()->ToString() << std::endl;
        send_arrow(dora_node.send_output, "typed_data", output);
    }

    std::cout << "[" << id << "] done" << std::endl;
    return 0;
}
