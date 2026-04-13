#include "dora-node-api.h"
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include <arrow/gpu/cuda_api.h>
#include <cstring>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// Arrow FFI helpers
// ---------------------------------------------------------------------------

struct ArrowInput {
    std::shared_ptr<arrow::Array> array;
    std::string id;
    rust::cxxbridge1::Box<Metadata> metadata;
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
        return {nullptr, "", std::move(info.metadata)};
    }

    auto result = arrow::ImportArray(&c_array, &c_schema);
    if (!result.ok()) {
        std::cerr << "ImportArray failed: " << result.status().ToString()
                  << std::endl;
        return {nullptr, "", std::move(info.metadata)};
    }
    return {result.MoveValueUnsafe(), std::string(info.id),
            std::move(info.metadata)};
}

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

int main() {
    auto dora_node = init_dora_node();
    auto id = std::string(node_id(dora_node.send_output));
    std::cout << "[" << id << "] started" << std::endl;

    // Get CUDA context for device 0
    auto manager = arrow::cuda::CudaDeviceManager::Instance();
    if (!manager.ok()) {
        std::cerr << "Failed to get CudaDeviceManager: "
                  << manager.status().ToString() << std::endl;
        return 1;
    }
    auto ctx_result = (*manager)->GetContext(0);
    if (!ctx_result.ok()) {
        std::cerr << "Failed to get CUDA context: "
                  << ctx_result.status().ToString() << std::endl;
        return 1;
    }
    auto ctx = *ctx_result;
    std::cout << "[" << id << "] CUDA context ready on device 0" << std::endl;

    // Build an empty array for "next" output
    arrow::Int8Builder empty_builder;
    auto empty_result = empty_builder.Finish();
    auto empty_array = empty_result.ValueOrDie();

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

        auto input = receive_arrow(std::move(event));
        if (!input.array) {
            continue;
        }

        // The Arrow array is Int8 containing the 64-byte CUDA IPC handle
        auto int8_array =
            std::static_pointer_cast<arrow::Int8Array>(input.array);
        const int8_t *raw = int8_array->raw_values();
        int64_t handle_len = int8_array->length();

        // Read metadata
        int64_t size = input.metadata->get_int("size");
        auto shape = input.metadata->get_list_int("shape");
        auto dtype_str = std::string(input.metadata->get_str("dtype"));

        std::cout << "[" << id << "] received CUDA IPC handle (" << handle_len
                  << " bytes), buffer size=" << size
                  << ", dtype=" << dtype_str << ", shape=[";
        for (size_t j = 0; j < shape.size(); j++) {
            if (j > 0) std::cout << ", ";
            std::cout << shape[j];
        }
        std::cout << "]" << std::endl;

        // Reconstruct IPC handle from raw bytes
        auto ipc_handle_result = arrow::cuda::CudaIpcMemHandle::FromBuffer(
            reinterpret_cast<const void *>(raw));
        if (!ipc_handle_result.ok()) {
            std::cerr << "FromBuffer failed: "
                      << ipc_handle_result.status().ToString() << std::endl;
            continue;
        }
        auto ipc_handle = *ipc_handle_result;

        // Open the IPC buffer — zero-copy access to sender's GPU memory
        auto buffer_result = ctx->OpenIpcBuffer(*ipc_handle);
        if (!buffer_result.ok()) {
            std::cerr << "OpenIpcBuffer failed: "
                      << buffer_result.status().ToString() << std::endl;
            continue;
        }
        auto cuda_buffer = *buffer_result;

        // Copy first few values to host and print
        int64_t copy_bytes = std::min(size, (int64_t)(10 * sizeof(int64_t)));
        std::vector<uint8_t> host_buf(copy_bytes);
        auto copy_status =
            cuda_buffer->CopyToHost(0, copy_bytes, host_buf.data());
        if (!copy_status.ok()) {
            std::cerr << "CopyToHost failed: " << copy_status.ToString()
                      << std::endl;
            continue;
        }

        auto *values = reinterpret_cast<int64_t *>(host_buf.data());
        int num_values = copy_bytes / sizeof(int64_t);
        std::cout << "[" << id << "] first " << num_values << " values:";
        for (int j = 0; j < num_values; j++) {
            std::cout << " " << values[j];
        }
        std::cout << std::endl;

        // Send "next" to trigger the sender again
        send_arrow(dora_node.send_output, "next", empty_array);
    }

    std::cout << "[" << id << "] done" << std::endl;
    return 0;
}
