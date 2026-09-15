#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

extern "C" {
#include "node_api.h"
}

namespace {

std::array<uint8_t, 8> encode_counter(uint64_t value) {
    std::array<uint8_t, 8> bytes{};
    for (std::size_t index = 0; index < bytes.size(); ++index) {
        bytes[index] = static_cast<uint8_t>(value >> (index * 8));
    }
    return bytes;
}

}  // namespace

int main() {
    void* context = init_dora_context_from_env();
    if (context == nullptr) {
        std::cerr << "[cpp-sender] failed to initialize Dora context\n";
        return 1;
    }

    uint64_t counter = 0;
    int exit_code = 0;
    const std::string output_id = "counter";

    while (true) {
        void* event = dora_next_event(context);
        if (event == nullptr) {
            break;
        }

        const DoraEventType type = read_dora_event_type(event);
        if (type == DoraEventType_Stop) {
            free_dora_event(event);
            break;
        }

        if (type == DoraEventType_Input) {
            char* id = nullptr;
            std::size_t id_length = 0;
            read_dora_input_id(event, &id, &id_length);

            if (std::string(id, id_length) == "tick") {
                const auto payload = encode_counter(counter);
                const int result = dora_send_output(
                    context,
                    const_cast<char*>(output_id.data()), output_id.size(),
                    reinterpret_cast<char*>(
                        const_cast<uint8_t*>(payload.data())),
                    payload.size());
                if (result != 0) {
                    std::cerr << "[cpp-sender] failed to send counter\n";
                    free_dora_event(event);
                    exit_code = 1;
                    break;
                }
                ++counter;
            }
        }

        free_dora_event(event);
    }

    free_dora_context(context);
    return exit_code;
}
