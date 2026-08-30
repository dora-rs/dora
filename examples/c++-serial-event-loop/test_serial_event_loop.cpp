#include "serial_event_loop.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

using namespace dora_extensions;

#define REQUIRE(condition)                                                        \
    do {                                                                          \
        if (!(condition)) {                                                       \
            std::cerr << "Test failed: " #condition << " ("                    \
                      << __FILE__ << ':' << __LINE__ << ')' << std::endl;          \
            std::abort();                                                         \
        }                                                                         \
    } while (false)

namespace {

struct FakeDoraEvent {
    DoraEventType type;
};

std::mutex fake_dora_mutex;
std::queue<DoraEventType> fake_dora_events;
std::function<bool()> fake_dora_stop_condition;
std::atomic<int> freed_event_count{0};
std::atomic<bool> fake_dora_waiting{false};

void reset_fake_dora(std::function<bool()> stop_condition = {}) {
    std::lock_guard<std::mutex> lock(fake_dora_mutex);
    fake_dora_events = {};
    fake_dora_stop_condition = std::move(stop_condition);
    freed_event_count = 0;
    fake_dora_waiting = false;
}

void enqueue_fake_event(DoraEventType type) {
    std::lock_guard<std::mutex> lock(fake_dora_mutex);
    fake_dora_events.push(type);
}

} // namespace

extern "C" {

void* init_dora_context_from_env() {
    return reinterpret_cast<void*>(1);
}

void free_dora_context(void*) {}

void* dora_next_event(void*) {
    fake_dora_waiting = true;
    while (true) {
        {
            std::lock_guard<std::mutex> lock(fake_dora_mutex);
            if (!fake_dora_events.empty()) {
                const auto type = fake_dora_events.front();
                fake_dora_events.pop();
                return new FakeDoraEvent{type};
            }
            if (fake_dora_stop_condition && fake_dora_stop_condition()) {
                return new FakeDoraEvent{DoraEventType_Stop};
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void free_dora_event(void* event) {
    ++freed_event_count;
    delete static_cast<FakeDoraEvent*>(event);
}

DoraEventType read_dora_event_type(void* event) {
    return static_cast<FakeDoraEvent*>(event)->type;
}

void read_dora_input_id(void*, char**, size_t*) {}
void read_dora_input_data(void*, char**, size_t*) {}
unsigned long long read_dora_input_timestamp(void*) { return 0; }
int dora_send_output(void*, const char*, size_t, const char*, size_t) { return 0; }
int dora_log(void*, const char*, size_t, const char*, size_t) { return 0; }

} // extern "C"

// ============================================================
// Test 1: SerialWorker unit test.
// ============================================================
void test_serial_worker_basic() {
    std::cout << "[Test 1] SerialWorker basic behavior..." << std::endl;

    std::atomic<int> call_count{0};
    std::vector<std::string> received_ids;

    {
        SerialWorker worker("test_worker",
            [&](const InputEvent& e) {
                call_count++;
                received_ids.push_back(e.id);
            });

        // Enqueue several events.
        for (int i = 0; i < 5; i++) {
            InputEvent ie;
            ie.id = "input_" + std::to_string(i);
            ie.data = {uint8_t(i), uint8_t(i + 1), uint8_t(i + 2)};
            worker.enqueue(ie);
        }

        // Give the worker enough time to process the events.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } // The worker destructor stops it automatically.

    REQUIRE(call_count == 5);
    REQUIRE(received_ids.size() == 5);
    for (int i = 0; i < 5; i++) {
        REQUIRE(received_ids[i] == "input_" + std::to_string(i));
    }

    std::cout << "[Test 1] Passed ✓" << std::endl;
}

// ============================================================
// Test 2: SerialWorker ordering test.
// ============================================================
void test_serial_worker_ordering() {
    std::cout << "[Test 2] SerialWorker event ordering..." << std::endl;

    std::vector<int> processed;
    std::mutex mtx;

    SerialWorker worker("order_worker",
        [&](const InputEvent& e) {
            std::lock_guard<std::mutex> lock(mtx);
            processed.push_back(e.data[0]);
        });

    // Enqueue 100 events in order.
    for (int i = 0; i < 100; i++) {
        InputEvent ie;
        ie.id = "seq";
        ie.data = {uint8_t(i)};
        worker.enqueue(ie);
    }

    // Wait for all events to be processed.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    REQUIRE(processed.size() == 100);
    for (int i = 0; i < 100; i++) {
        REQUIRE(processed[i] == i); // Strict FIFO ordering is required.
    }

    std::cout << "[Test 2] Passed ✓ (processed " << processed.size() << " events)" << std::endl;
}

// ============================================================
// Test 3: SerialEventLoop registration and cancellation.
// ============================================================
void test_event_loop_registration() {
    std::cout << "[Test 3] EventLoop registration and cancellation..." << std::endl;

    SerialEventLoop loop("test_node");

    // Register handlers.
    loop.register_handler("camera", [](const InputEvent&) {});
    loop.register_handler("lidar", [](const InputEvent&) {});

    // Register a timer.
    std::atomic<int> tick_count{0};
    loop.register_timer("tick", std::chrono::milliseconds(50),
        [&]() { tick_count++; }, true);

    reset_fake_dora([&]() { return tick_count >= 2; });
    loop.run();
    REQUIRE(tick_count >= 2);

    // Cancel the timer.
    bool ok = loop.cancel_timer("tick");
    REQUIRE(ok);

    int snapshot = tick_count;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(tick_count == snapshot); // The timer must not fire after cancellation.

    // Cancelling a nonexistent timer must fail.
    ok = loop.cancel_timer("nonexistent");
    REQUIRE(!ok);

    std::cout << "[Test 3] Passed ✓ (tick_count=" << tick_count << ")" << std::endl;
}

// ============================================================
// Test 4: One-shot timer.
// ============================================================
void test_oneshot_timer() {
    std::cout << "[Test 4] One-shot timer..." << std::endl;

    SerialEventLoop loop("test_node");
    std::atomic<int> fired{0};

    loop.register_timer("once", std::chrono::milliseconds(30),
        [&]() { fired++; }, false); // Do not repeat.

    reset_fake_dora([&]() { return fired == 1; });
    loop.run();
    REQUIRE(fired == 1);

    std::cout << "[Test 4] Passed ✓" << std::endl;
}

// ============================================================
// Test 5: send_output thread safety.
// ============================================================
void test_send_output() {
    std::cout << "[Test 5] send_output queuing..." << std::endl;

    SerialEventLoop loop("test_node");

    std::atomic<int> sent{0};

    // Call send_output concurrently from multiple threads.
    std::thread t1([&]() {
        for (int i = 0; i < 50; i++) {
            loop.send_output("out_a", {uint8_t(i)});
            sent++;
        }
    });

    std::thread t2([&]() {
        for (int i = 0; i < 50; i++) {
            loop.send_output("out_b", {uint8_t(i)});
            sent++;
        }
    });

    t1.join();
    t2.join();

    REQUIRE(sent == 100);
    // This test does not call run(), so the messages are not sent to Dora.
    // They have only been safely added to the queue.

    std::cout << "[Test 5] Passed ✓ (queued " << sent << " messages)" << std::endl;
}

// ============================================================
// Test 6: Call send_output from a timer callback.
// ============================================================
void test_timer_send_output() {
    std::cout << "[Test 6] Sending output from a timer callback..." << std::endl;

    SerialEventLoop loop("test_node");
    std::atomic<int> output_calls{0};

    // A timer runs outside the main thread, so send_output queues the message.
    loop.register_timer("sender", std::chrono::milliseconds(30),
        [&]() {
            loop.send_output("timer_out", {0xAA, 0xBB});
            output_calls++;
        }, true);

    reset_fake_dora([&]() { return output_calls >= 2; });
    loop.run();
    REQUIRE(output_calls >= 2);

    loop.cancel_timer("sender");
    std::cout << "[Test 6] Passed ✓ (send_output called " << output_calls << " times)" << std::endl;
}

// ============================================================
// Test 7: Free an unknown event exactly once.
// ============================================================
void test_unknown_event_freed_once() {
    std::cout << "[Test 7] Freeing an unknown event exactly once..." << std::endl;

    reset_fake_dora();
    enqueue_fake_event(DoraEventType_Unknown);
    SerialEventLoop loop("test_node");
    loop.run();

    REQUIRE(freed_event_count == 1);
    std::cout << "[Test 7] Passed ✓" << std::endl;
}

// ============================================================
// Test 8: A timer callback can cancel itself.
// ============================================================
void test_timer_callback_can_cancel_itself() {
    std::cout << "[Test 8] Timer callback cancelling itself..." << std::endl;

    SerialEventLoop loop("test_node");
    std::atomic<bool> callback_completed{false};
    loop.register_timer("self-cancelling", std::chrono::milliseconds(10), [&]() {
        REQUIRE(loop.cancel_timer("self-cancelling"));
        callback_completed = true;
    });

    reset_fake_dora([&]() { return callback_completed.load(); });
    loop.run();

    REQUIRE(callback_completed);
    std::cout << "[Test 8] Passed ✓" << std::endl;
}

// ============================================================
// Test 9: Concurrent send_output calls while run shuts down.
// ============================================================
void test_send_output_during_shutdown() {
    std::cout << "[Test 9] Concurrent send_output calls during run shutdown..." << std::endl;

    SerialEventLoop loop("test_node");
    std::atomic<bool> stop_run{false};
    std::atomic<bool> keep_sending{true};
    reset_fake_dora([&]() { return stop_run.load(); });

    std::thread run_thread([&]() { loop.run(); });
    while (!fake_dora_waiting) {
        std::this_thread::yield();
    }

    std::thread sender([&]() {
        while (keep_sending) {
            loop.send_output("concurrent", {0x01});
        }
    });

    stop_run = true;
    run_thread.join();
    keep_sending = false;
    sender.join();

    std::cout << "[Test 9] Passed ✓" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  SerialEventLoop Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_serial_worker_basic();
    test_serial_worker_ordering();
    test_event_loop_registration();
    test_oneshot_timer();
    test_send_output();
    test_timer_send_output();
    test_unknown_event_freed_once();
    test_timer_callback_can_cancel_itself();
    test_send_output_during_shutdown();

    std::cout << "========================================" << std::endl;
    std::cout << "  All tests passed ✓" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
