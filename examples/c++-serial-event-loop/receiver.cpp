#include "serial_event_loop.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>

namespace {

bool decode_counter(
    const uint8_t* data, std::size_t size, uint64_t& value) {
    if (data == nullptr || size != 8) {
        return false;
    }
    value = 0;
    for (std::size_t index = 0; index < size; ++index) {
        value |= static_cast<uint64_t>(data[index]) << (index * 8);
    }
    return true;
}

}  // namespace

int main() {
    dora_extensions::SerialEventLoop loop("cpp-receiver");
    loop.register_handler(
        "counter", [](const dora_extensions::InputEvent& event) {
            uint64_t counter = 0;
            if (!decode_counter(
                    event.data.data(), event.data.size(), counter)) {
                std::cerr << "[cpp-receiver] invalid counter payload: "
                          << event.data.size() << " bytes\n";
                return;
            }
            std::cout << "[cpp-receiver] received " << event.id << "="
                      << counter << std::endl;
        });
    loop.run();
}
