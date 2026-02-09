#include <dora-node-api.h>
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include <cstdint>
#include <exception>
#include <iostream>
#include <utility>
#include <string>

std::shared_ptr<arrow::Array> receive_and_print_input(rust::cxxbridge1::Box<DoraEvent> event) {
    std::cout << "Received input event" << std::endl;

    struct ArrowArray c_array;
    struct ArrowSchema c_schema;

    // Use the new function that returns input ID and metadata
    auto input_info = event_as_arrow_input_with_info(
        std::move(event),
        reinterpret_cast<uint8_t*>(&c_array),
        reinterpret_cast<uint8_t*>(&c_schema)
    );

    if (!input_info.error.empty()) {
        std::cerr << "Error getting Arrow array: " << std::string(input_info.error) << std::endl;
        return nullptr;
    }

    // Print input ID and metadata
    std::cout << "Input ID: " << std::string(input_info.id) << std::endl;
    auto metadata = std::move(input_info.metadata);
    std::cout << "Metadata timestamp: " << metadata->timestamp() << std::endl;

    auto keys = metadata->list_keys();
    std::cout << "Metadata keys: [";
    for (std::size_t i = 0; i < keys.size(); i++) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << std::string(keys[i]);
    }
    std::cout << "]" << std::endl;

    for (std::size_t i = 0; i < keys.size(); i++) {
        const std::string key = std::string(keys[i]);
        try {
            auto value_type = metadata->type(key);
            switch (value_type) {
            case MetadataValueType::Float:
                std::cout << "Metadata[" << key << "] (float) = " << metadata->get_float(key)
                          << std::endl;
                break;
            case MetadataValueType::Integer:
                std::cout << "Metadata[" << key << "] (int) = " << metadata->get_int(key)
                          << std::endl;
                break;
            case MetadataValueType::String:
                std::cout << "Metadata[" << key << "] (string) = "
                          << std::string(metadata->get_str(key)) << std::endl;
                break;
            case MetadataValueType::Bool:
                std::cout << "Metadata[" << key << "] (bool) = " << std::boolalpha
                          << metadata->get_bool(key) << std::noboolalpha << std::endl;
                break;
            case MetadataValueType::ListInt: {
                auto values = metadata->get_list_int(key);
                std::cout << "Metadata[" << key << "] (list<int>) = [";
                for (std::size_t j = 0; j < values.size(); j++) {
                    if (j > 0) {
                        std::cout << ", ";
                    }
                    std::cout << values[j];
                }
                std::cout << "]" << std::endl;
                break;
            }
            case MetadataValueType::ListFloat: {
                auto values = metadata->get_list_float(key);
                std::cout << "Metadata[" << key << "] (list<float>) = [";
                for (std::size_t j = 0; j < values.size(); j++) {
                    if (j > 0) {
                        std::cout << ", ";
                    }
                    std::cout << values[j];
                }
                std::cout << "]" << std::endl;
                break;
            }
            case MetadataValueType::ListString: {
                auto values = metadata->get_list_string(key);
                std::cout << "Metadata[" << key << "] (list<string>) = [";
                for (std::size_t j = 0; j < values.size(); j++) {
                    if (j > 0) {
                        std::cout << ", ";
                    }
                    std::cout << std::string(values[j]);
                }
                std::cout << "]" << std::endl;
                break;
            }
            case MetadataValueType::Timestamp: {
                auto nanos = metadata->get_timestamp(key);
                std::cout << "Metadata[" << key << "] (timestamp) = " << nanos << " ns" << std::endl;
                break;
            }
            default:
                std::cout << "Metadata[" << key << "] has unsupported type" << std::endl;
                break;
            }
        } catch (const std::exception& err) {
            std::cout << "Metadata[" << key << "] unavailable: " << err.what() << std::endl;
        }
    }

    std::cout << "Metadata JSON: " << std::string(metadata->to_json()) << std::endl;

    auto result2 = arrow::ImportArray(&c_array, &c_schema);
    std::shared_ptr<arrow::Array> input_array = result2.ValueOrDie();
    std::cout << "Received Arrow array: " << input_array->ToString() << std::endl;

    std::cout << "Array details: type=" << input_array->type()->ToString()
    << ", length=" << input_array->length() << std::endl;

    return input_array;
}

// To send output
bool send_output(DoraNode& dora_node, std::shared_ptr<arrow::Array> output_array, int counter) {
    if (!output_array) {
        std::cerr << "Error: Attempted to send a null Arrow array" << std::endl;
        return false;
    }
    struct ArrowArray out_c_array;
    struct ArrowSchema out_c_schema;

    auto export_status = arrow::ExportArray(*output_array, &out_c_array, &out_c_schema);
    if (!export_status.ok()) {
        std::cerr << "Failed to export Arrow array: " << export_status.ToString() << std::endl;
        return false;
    }

    auto metadata = new_metadata();
    metadata->set_string("producer", "cpp-node");
    metadata->set_int("iteration", counter);
    metadata->set_bool("successful", true);

    rust::Vec<int64_t> doubled_values;
    doubled_values.push_back(counter);
    doubled_values.push_back(counter * 2);
    doubled_values.push_back(counter * 3);
    metadata->set_list_int("multiples", std::move(doubled_values));

    rust::Vec<double> weights;
    weights.push_back(0.1);
    weights.push_back(0.2);
    metadata->set_list_float("weights", std::move(weights));

    rust::Vec<rust::String> notes;
    notes.push_back("generated");
    notes.push_back("from_cpp");
    metadata->set_list_string("notes", std::move(notes));

    auto send_result = send_arrow_output(
        dora_node.send_output,
        "counter",
        reinterpret_cast<uint8_t*>(&out_c_array),
        reinterpret_cast<uint8_t*>(&out_c_schema),
        std::move(metadata)
    );

    if (!send_result.error.empty()) {
        std::string error_message(send_result.error);
        std::cerr << "Error sending Arrow array: " << error_message << std::endl;
        return false;
    }

    return true;
}

int main() {
    try {
        auto dora_node = init_dora_node();
        std::cout << "Dora node initialized successfully" << std::endl;
        int counter=0;
        while (counter<10) {
            counter++;
            auto event = dora_node.events->next();
            auto type = event_type(event);

            if (type == DoraEventType::Stop) {
                std::cout << "Received stop event, exiting" << std::endl;
                break;
            }
            else if (type == DoraEventType::AllInputsClosed) {
                std::cout << "All inputs closed, exiting" << std::endl;
                break;
            }
            else if (type == DoraEventType::Input) {
                std::shared_ptr<arrow::Array> input_array = receive_and_print_input(std::move(event));

                std::shared_ptr<arrow::Array> output_array;

                arrow::Int32Builder builder;
                if (!builder.Append(10).ok() || !builder.Append(100).ok() ||
                    !builder.Append(1000).ok()) {
                    std::cerr << "Failed to append to builder" << std::endl;
                    return 1;
                }
                if (!builder.Finish(&output_array).ok()) {
                    std::cerr << "Failed to finish builder" << std::endl;
                    return 1;
                }

                //Printing Before sending
                auto str_array = std::static_pointer_cast<arrow::Int32Array>(output_array);

            send_output(dora_node, output_array, counter);
        }
    }

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
