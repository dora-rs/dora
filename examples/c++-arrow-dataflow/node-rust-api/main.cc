#include "../build/dora-node-api.h" 
#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

std::shared_ptr<arrow::Array> receive_and_print_input(rust::cxxbridge1::Box<DoraEvent> event) {
    std::cout << "Received input event" << std::endl;

    struct ArrowArray c_array;
    struct ArrowSchema c_schema;
    
    auto result = event_as_arrow_input(
        std::move(event),
        reinterpret_cast<uint8_t*>(&c_array),
        reinterpret_cast<uint8_t*>(&c_schema)
    );
    
    if (!result.error.empty()) {
        std::cerr << "Error getting Arrow array: " << std::endl;
        return nullptr;
    }
    
    auto result2 = arrow::ImportArray(&c_array, &c_schema);
    std::shared_ptr<arrow::Array> input_array = result2.ValueOrDie();
    std::cout << "Received Arrow array: " << input_array->ToString() << std::endl;
    
    std::cout << "Array details: type=" << input_array->type()->ToString() 
    << ", length=" << input_array->length() << std::endl;
    
    return input_array;
}

// To send output
bool send_output(DoraNode& dora_node, std::shared_ptr<arrow::Array> output_array) {
    if (!output_array) {
        std::cerr << "Error: Attempted to send a null Arrow array" << std::endl;
        return false;
    }
    struct ArrowArray out_c_array;
    struct ArrowSchema out_c_schema;
    
    arrow::ExportArray(*output_array, &out_c_array, &out_c_schema);
    
    auto send_result = send_arrow_output(
        dora_node.send_output,
        "counter",
        reinterpret_cast<uint8_t*>(&out_c_array),
        reinterpret_cast<uint8_t*>(&out_c_schema)
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
                builder.Append(10);
                builder.Append(100);
                builder.Append(1000);
                builder.Finish(&output_array);
                std::cout << "Created new string array: " << output_array->ToString() << std::endl;
                
                //Printing Before sending
                auto str_array = std::static_pointer_cast<arrow::Int32Array>(output_array);
                std::cout << "Values: [";
                for (int i = 0; i < str_array->length(); i++) {
                    if (i > 0) std::cout << ", ";
                    std::cout << str_array->Value(i);
                }
                std::cout << "]" << std::endl;

                send_output(dora_node, output_array);
            }
        }
        
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}