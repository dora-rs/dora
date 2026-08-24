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

// 简单的输入事件结构
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
 * @brief 专属工作者类
 * 每个实例拥有独立的线程和队列，处理特定类型的任务
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

    // 主循环通过此接口快速写入数据并通知
    void enqueue(const InputEvent& event) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            queue_.push(event);
        }
        cv_.notify_one(); // 通知专属线程有新数据
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
            // 执行具体的耗时业务逻辑
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
 * @brief 任务响应式事件循环
 */
class SerialEventLoop {
public:
    SerialEventLoop(const std::string& node_name) : node_name_(node_name), running_(false) {}

    // 注册特定话题的专属处理器
    void register_handler(const std::string& id, SerialWorker::Handler handler) {
        workers_[id] = std::make_unique<SerialWorker>(id, handler);
    }

    /**
     * @brief 注册定时器
     *        Register timer
     *
     * @param id 定时器ID
     * @param interval 时间间隔（毫秒）
     * @param handler 回调函数
     * @param repeat 是否重复，默认true
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
     * @brief 取消定时器
     *        Cancel timer
     *
     * @param id 定时器ID
     * @return 是否成功取消
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
     * @brief 发送输出（线程安全，可在任何线程调用）
     *        Send output (thread-safe, can be called from any thread)
     *
     * @param output_id 输出ID
     * @param data 输出数据
     */
    void send_output(const std::string& output_id, const std::vector<uint8_t>& data) {
        // 如果在主线程中且在事件循环内，直接发送
        if (std::this_thread::get_id() == main_thread_id_ && in_event_loop_ && dora_context_) {
            direct_send_output(output_id, data);
        } else {
            queue_output(output_id, data);
        }
    }

    void run() {
        // 1. 初始化Dora上下文
        dora_context_ = init_dora_context_from_env();
        if (!dora_context_) {
            std::cerr << "[" << node_name_ << "] 初始化Dora上下文失败 / Init dora context failed" << std::endl;
            return;
        }
        running_ = true;
        in_event_loop_ = true;
        main_thread_id_ = std::this_thread::get_id();
        std::cout << "[" << node_name_ << "] 专属线程模式启动..." << std::endl;

        // 启动定时器线程
        timer_thread_ = std::make_unique<std::thread>(&SerialEventLoop::timer_thread_func, this);

        while (running_) {
            // 处理输出队列中的消息
            process_output_queue();

            // 2. 阻塞等待Dora事件
            void* event = dora_next_event(dora_context_);
            if (!event) {
                std::cerr << "[" << node_name_ << "] 意外的事件结束 / Unexpected end of event" << std::endl;
                break;
            }

            enum DoraEventType ty = read_dora_event_type(event);

            if (ty == DoraEventType_Input) {
                char* id = nullptr;
                size_t id_len = 0;
                read_dora_input_id(event, &id, &id_len);

                char* data = nullptr;
                size_t data_len = 0;
                read_dora_input_data(event, &data, &data_len);

                std::string input_id(id, id_len);

                // 3. 核心分发逻辑：如果是已注册的话题，写入对应队列并立即返回
                auto it = workers_.find(input_id);
                if (it != workers_.end()) {
                    InputEvent ie;
                    ie.id = input_id;
                    ie.data = std::vector<uint8_t>(data, data + data_len);

                    it->second->enqueue(ie); // 瞬间完成，不会阻塞主循环
                }
            }
            else if (ty == DoraEventType_Stop) {
                std::cout << "[" << node_name_ << "] 收到停止事件 / Received stop event" << std::endl;
                free_dora_event(event);
                running_ = false;
            }
            else {
                std::cerr << "[" << node_name_ << "] 未知事件类型 / Unknown event type: " << ty << std::endl;
                free_dora_event(event);
                running_ = false;
            }

            if (ty != DoraEventType_Stop) {
                free_dora_event(event);
            }
        }

        in_event_loop_ = false;

        // 停止定时器线程
        if (timer_thread_ && timer_thread_->joinable()) {
            timer_thread_->join();
        }
        timer_thread_.reset();

        // 释放Dora上下文
        free_dora_context(dora_context_);
        dora_context_ = nullptr;
    }

private:
    void queue_output(const std::string& output_id, const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(output_queue_mutex_);
        output_queue_.push({output_id, data});
    }

    void direct_send_output(const std::string& output_id, const std::vector<uint8_t>& data) {
        if (!dora_context_) {
            std::cerr << "[" << node_name_ << "] 节点未初始化 / Node not initialized" << std::endl;
            return;
        }

        std::lock_guard<std::mutex> lock(send_lock_);
        dora_send_output(dora_context_,
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
                direct_send_output(message.output_id, message.data);
            } catch (const std::exception& e) {
                std::cerr << "[" << node_name_
                          << "] 处理输出消息出错 / Error processing output message: "
                          << e.what() << std::endl;
            }
        }
    }

    void timer_thread_func() {
        while (running_) {
            auto now = std::chrono::steady_clock::now();

            {
                std::lock_guard<std::mutex> lock(timer_mutex_);
                auto timers_copy = timers_;

                for (const auto& pair : timers_copy) {
                    const auto& timer = pair.second;
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - timer.last_trigger);

                    if (elapsed >= timer.interval) {
                        auto handler_it = timer_handlers_.find(timer.id);
                        if (handler_it != timer_handlers_.end()) {
                            try {
                                handler_it->second();
                            } catch (const std::exception& e) {
                                std::cerr << "[" << node_name_
                                          << "] 定时器回调出错 / Timer callback error: "
                                          << e.what() << std::endl;
                            }
                        }

                        auto current_it = timers_.find(timer.id);
                        if (current_it != timers_.end()) {
                            if (timer.repeat) {
                                current_it->second.last_trigger = now;
                            } else {
                                timers_.erase(timer.id);
                                timer_handlers_.erase(timer.id);
                            }
                        }
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::string node_name_;
    std::atomic<bool> running_;
    bool in_event_loop_ = false;
    std::thread::id main_thread_id_;
    void* dora_context_ = nullptr;

    std::unordered_map<std::string, std::unique_ptr<SerialWorker>> workers_;

    // 输出队列 / Output queue
    std::queue<OutputMessage> output_queue_;
    std::mutex output_queue_mutex_;
    std::mutex send_lock_;

    // 定时器 / Timers
    std::unordered_map<std::string, TimerEvent> timers_;
    std::unordered_map<std::string, std::function<void()>> timer_handlers_;
    std::mutex timer_mutex_;
    std::unique_ptr<std::thread> timer_thread_;
};

} // namespace dora_extensions

#endif
