#ifndef SERIAL_EVENT_LOOP_HPP_
#define SERIAL_EVENT_LOOP_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <unordered_map>
#include <atomic>
#include <memory>
#include <chrono>
#include <cstring>
#include <cstdint>
extern "C" {
#include "node_api.h"
}

namespace dora_extensions {

// Simple input event representation.
struct InputEvent {
    std::string id;
    std::vector<uint8_t> data;
};

struct OutputMessage {
    std::string output_id;
    std::vector<uint8_t> data;
};

struct TimerEvent {
    std::string id;
    std::chrono::milliseconds interval;
    std::chrono::steady_clock::time_point last_trigger;
    bool repeat;
};

/**
 * @brief Dedicated worker class.
 * Each instance owns a thread and queue for processing a specific task type.
 */
class SerialWorker {
public:
    using Handler = std::function<void(const InputEvent&)>;

    SerialWorker(const std::string& name, Handler handler)
        : name_(name), handler_(handler), stop_(false) {
        worker_thread_ = std::thread(&SerialWorker::processLoop, this);
    }

    ~SerialWorker() {
        stop();
    }

    // Quickly enqueue data from the main loop and notify the worker.
    void enqueue(const InputEvent& event) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            queue_.push(event);
        }
        cv_.notify_one(); // Notify the dedicated thread that data is available.
    }

    void stop() {
        stop_ = true;
        cv_.notify_all();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

private:
    void processLoop() {
        while (!stop_) {
            InputEvent event;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait(lock, [this] { return !queue_.empty() || stop_; });
                
                if (stop_ && queue_.empty()) break;
                
                event = std::move(queue_.front());
                queue_.pop();
            }
            // Execute the potentially time-consuming application logic.
            if (handler_) {
                handler_(event);
            }
        }
    }

    std::string name_;
    Handler handler_;
    std::queue<InputEvent> queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::thread worker_thread_;
    std::atomic<bool> stop_;
};

/**
 * @brief Task-responsive event loop.
 */
class SerialEventLoop {
public:
    SerialEventLoop(const std::string& node_name) : node_name_(node_name), running_(false) {}

    // Register a dedicated handler for a specific topic.
    void register_handler(const std::string& id, SerialWorker::Handler handler) {
        workers_[id] = std::make_unique<SerialWorker>(id, handler);
    }

    /**
     * @brief Register a timer.
     *
     * @param id Timer ID.
     * @param interval Interval in milliseconds.
     * @param handler Callback function.
     * @param repeat Whether the timer repeats; defaults to true.
     */
    void register_timer(const std::string& id, std::chrono::milliseconds interval,
                        std::function<void()> handler, bool repeat = true) {
        std::lock_guard<std::mutex> lock(timer_mutex_);
        TimerEvent timer;
        timer.id = id;
        timer.interval = interval;
        timer.last_trigger = std::chrono::steady_clock::now();
        timer.repeat = repeat;
        timers_[id] = timer;
        timer_handlers_[id] = handler;
    }

    /**
     * @brief Cancel a timer.
     *
     * @param id Timer ID.
     * @return Whether the timer was successfully cancelled.
     */
    bool cancel_timer(const std::string& id) {
        std::lock_guard<std::mutex> lock(timer_mutex_);
        auto it = timers_.find(id);
        if (it != timers_.end()) {
            timers_.erase(id);
            timer_handlers_.erase(id);
            return true;
        }
        return false;
    }

    /**
     * @brief Send output. Thread-safe and callable from any thread.
     *
     * @param output_id Output ID.
     * @param data Output data.
     */
    void send_output(const std::string& output_id, const std::vector<uint8_t>& data) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Send directly when called by the main thread inside the event loop.
            if (std::this_thread::get_id() == main_thread_id_ && in_event_loop_ && dora_context_) {
                direct_send_output(dora_context_, output_id, data);
                return;
            }
        }
        queue_output(output_id, data);
    }

    void run() {
        // 1. Initialize the Dora context.
        void* dora_context = init_dora_context_from_env();
        if (!dora_context) {
            std::cerr << "[" << node_name_ << "] Failed to initialize Dora context" << std::endl;
            return;
        }
        running_ = true;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            dora_context_ = dora_context;
            in_event_loop_ = true;
            main_thread_id_ = std::this_thread::get_id();
        }
        std::cout << "[" << node_name_ << "] Dedicated-thread mode started" << std::endl;

        // Start the timer thread.
        timer_thread_ = std::make_unique<std::thread>(&SerialEventLoop::timer_thread_func, this);

        while (running_) {
            // Process messages in the output queue.
            process_output_queue();

            // 2. Block while waiting for the next Dora event.
            void* event = dora_next_event(dora_context);
            if (!event) {
                std::cerr << "[" << node_name_ << "] Unexpected end of event stream" << std::endl;
                break;
            }
            std::unique_ptr<void, decltype(&free_dora_event)> event_guard(
                event, &free_dora_event);

            enum DoraEventType ty = read_dora_event_type(event);

            if (ty == DoraEventType_Input) {
                char* id = nullptr;
                size_t id_len = 0;
                read_dora_input_id(event, &id, &id_len);

                char* data = nullptr;
                size_t data_len = 0;
                read_dora_input_data(event, &data, &data_len);

                std::string input_id(id, id_len);

                // 3. Dispatch registered topics to their queues and return immediately.
                auto it = workers_.find(input_id);
                if (it != workers_.end()) {
                    InputEvent ie;
                    ie.id = input_id;
                    ie.data = std::vector<uint8_t>(data, data + data_len);

                    it->second->enqueue(ie); // Returns quickly without blocking the main loop.
                }
            }
            else if (ty == DoraEventType_Stop) {
                std::cout << "[" << node_name_ << "] Received stop event" << std::endl;
                running_ = false;
            }
            else {
                std::cerr << "[" << node_name_ << "] Unknown event type: " << ty << std::endl;
                running_ = false;
            }
        }

        // Stop the timer thread.
        if (timer_thread_ && timer_thread_->joinable()) {
            timer_thread_->join();
        }
        timer_thread_.reset();

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            in_event_loop_ = false;
            main_thread_id_ = {};
            dora_context_ = nullptr;
        }

        // Release the Dora context.
        free_dora_context(dora_context);
    }

private:
    void queue_output(const std::string& output_id, const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(output_queue_mutex_);
        output_queue_.push({output_id, data});
    }

    void direct_send_output(void* dora_context, const std::string& output_id,
                            const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(send_lock_);
        dora_send_output(dora_context,
                         const_cast<char*>(output_id.c_str()), output_id.size(),
                         const_cast<char*>(reinterpret_cast<const char*>(data.data())), data.size());
    }

    void process_output_queue() {
        while (true) {
            OutputMessage message;
            {
                std::lock_guard<std::mutex> lock(output_queue_mutex_);
                if (output_queue_.empty()) break;
                message = output_queue_.front();
                output_queue_.pop();
            }

            try {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (dora_context_) {
                    direct_send_output(dora_context_, message.output_id, message.data);
                }
            } catch (const std::exception& e) {
                std::cerr << "[" << node_name_
                          << "] Error processing output message: "
                          << e.what() << std::endl;
            }
        }
    }

    void timer_thread_func() {
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            std::vector<std::function<void()>> due_handlers;

            {
                std::lock_guard<std::mutex> lock(timer_mutex_);
                for (auto timer_it = timers_.begin(); timer_it != timers_.end();) {
                    auto& timer = timer_it->second;
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - timer.last_trigger);

                    if (elapsed >= timer.interval) {
                        auto handler_it = timer_handlers_.find(timer.id);
                        if (handler_it != timer_handlers_.end()) {
                            due_handlers.push_back(handler_it->second);
                        }

                        if (timer.repeat) {
                            timer.last_trigger = now;
                            ++timer_it;
                        } else {
                            timer_handlers_.erase(timer.id);
                            timer_it = timers_.erase(timer_it);
                        }
                    } else {
                        ++timer_it;
                    }
                }
            }

            for (const auto& handler : due_handlers) {
                try {
                    handler();
                } catch (const std::exception& e) {
                    std::cerr << "[" << node_name_
                              << "] Timer callback error: "
                              << e.what() << std::endl;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::string node_name_;
    std::atomic<bool> running_;
    std::mutex state_mutex_;
    bool in_event_loop_ = false;
    std::thread::id main_thread_id_;
    void* dora_context_ = nullptr;

    std::unordered_map<std::string, std::unique_ptr<SerialWorker>> workers_;

    // Output queue.
    std::queue<OutputMessage> output_queue_;
    std::mutex output_queue_mutex_;
    std::mutex send_lock_;

    // Timers.
    std::unordered_map<std::string, TimerEvent> timers_;
    std::unordered_map<std::string, std::function<void()>> timer_handlers_;
    std::mutex timer_mutex_;
    std::unique_ptr<std::thread> timer_thread_;
};

} // namespace dora_extensions

#endif
