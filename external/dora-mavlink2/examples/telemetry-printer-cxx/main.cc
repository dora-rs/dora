// C++ consumer for the mavlink2-bridge example.
//
// Subscribes to `bridge/heartbeat`, an Apache Arrow StructArray whose
// schema matches HEARTBEAT_DATA (custom_mode u32, mavtype u32,
// autopilot u32, base_mode u8, system_status u32, mavlink_version u8).
// Prints one decoded line per frame.
//
// This is a minimal demonstration: it does not echo metadata or send
// outputs back, just consumes and prints. Pair with `mavlink-sim` and
// the bridge node to round-trip without an autopilot.

#include <dora-node-api.h>
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace {

// Pull the first row's primitive value out of a child column on a
// StructArray. Aborts with a descriptive message if the column is
// missing or the runtime type does not match.
template <typename ArrayT, typename ValueT>
ValueT field_value(const arrow::StructArray& s, const std::string& name) {
    auto field = s.GetFieldByName(name);
    if (!field) {
        throw std::runtime_error("StructArray missing field: " + name);
    }
    auto typed = std::dynamic_pointer_cast<ArrayT>(field);
    if (!typed) {
        throw std::runtime_error(
            "field " + name + " has unexpected type " + field->type()->ToString());
    }
    if (typed->length() == 0) {
        throw std::runtime_error("field " + name + " is empty");
    }
    return static_cast<ValueT>(typed->Value(0));
}

void print_heartbeat(const arrow::StructArray& s, std::uint64_t seq) {
    auto custom_mode = field_value<arrow::UInt32Array, std::uint32_t>(s, "custom_mode");
    auto mavtype = field_value<arrow::UInt32Array, std::uint32_t>(s, "mavtype");
    auto autopilot = field_value<arrow::UInt32Array, std::uint32_t>(s, "autopilot");
    auto base_mode = field_value<arrow::UInt8Array, std::uint32_t>(s, "base_mode");
    auto system_status = field_value<arrow::UInt32Array, std::uint32_t>(s, "system_status");
    auto mavlink_version =
        field_value<arrow::UInt8Array, std::uint32_t>(s, "mavlink_version");

    std::cout << "telemetry-printer-cxx: heartbeat #" << seq
              << " custom_mode=" << custom_mode << " mavtype=" << mavtype
              << " autopilot=" << autopilot << " base_mode=" << base_mode
              << " system_status=" << system_status
              << " mavlink_version=" << mavlink_version << std::endl;
}

}  // namespace

int main() {
    try {
        auto dora_node = init_dora_node();
        std::cout << "telemetry-printer-cxx: dora node initialized" << std::endl;
        std::uint64_t received = 0;

        for (;;) {
            auto event = dora_node.events->next();
            auto type = event_type(event);

            if (type == DoraEventType::Stop) {
                std::cout << "telemetry-printer-cxx: STOP after " << received
                          << " heartbeats" << std::endl;
                break;
            } else if (type == DoraEventType::AllInputsClosed) {
                std::cout << "telemetry-printer-cxx: inputs closed after " << received
                          << " heartbeats" << std::endl;
                break;
            } else if (type == DoraEventType::Input) {
                struct ArrowArray c_array;
                struct ArrowSchema c_schema;
                auto info = event_as_arrow_input_with_info(
                    std::move(event),
                    reinterpret_cast<std::uint8_t*>(&c_array),
                    reinterpret_cast<std::uint8_t*>(&c_schema));
                if (!info.error.empty()) {
                    std::cerr << "telemetry-printer-cxx: arrow import: "
                              << std::string(info.error) << std::endl;
                    continue;
                }
                auto imported = arrow::ImportArray(&c_array, &c_schema);
                if (!imported.ok()) {
                    std::cerr << "telemetry-printer-cxx: ImportArray: "
                              << imported.status().ToString() << std::endl;
                    continue;
                }
                auto array = imported.ValueOrDie();
                auto s = std::dynamic_pointer_cast<arrow::StructArray>(array);
                if (!s) {
                    std::cerr << "telemetry-printer-cxx: expected StructArray, got "
                              << array->type()->ToString() << std::endl;
                    continue;
                }
                received++;
                if (std::string(info.id) == "heartbeat") {
                    print_heartbeat(*s, received);
                } else {
                    std::cerr << "telemetry-printer-cxx: unexpected input id: "
                              << std::string(info.id) << std::endl;
                }
            }
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "telemetry-printer-cxx: fatal: " << e.what() << std::endl;
        return 1;
    }
}
