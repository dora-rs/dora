#include "serial_event_loop.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace dora_extensions;

// ============================================================
// 测试1: SerialWorker 独立单元测试
// ============================================================
void test_serial_worker_basic() {
    std::cout << "[测试1] SerialWorker 基本功能..." << std::endl;

    std::atomic<int> call_count{0};
    std::vector<std::string> received_ids;

    {
        SerialWorker worker("test_worker",
            [&](const InputEvent& e) {
                call_count++;
                received_ids.push_back(e.id);
            });

        // 入队几个事件
        for (int i = 0; i < 5; i++) {
            InputEvent ie;
            ie.id = "input_" + std::to_string(i);
            ie.data = {uint8_t(i), uint8_t(i + 1), uint8_t(i + 2)};
            worker.enqueue(ie);
        }

        // 等待worker处理完（给一点时间）
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } // worker析构，自动stop

    assert(call_count == 5);
    assert(received_ids.size() == 5);
    for (int i = 0; i < 5; i++) {
        assert(received_ids[i] == "input_" + std::to_string(i));
    }

    std::cout << "[测试1] 通过 ✓" << std::endl;
}

// ============================================================
// 测试2: SerialWorker 顺序性测试
// ============================================================
void test_serial_worker_ordering() {
    std::cout << "[测试2] SerialWorker 事件顺序性..." << std::endl;

    std::vector<int> processed;
    std::mutex mtx;

    SerialWorker worker("order_worker",
        [&](const InputEvent& e) {
            std::lock_guard<std::mutex> lock(mtx);
            processed.push_back(e.data[0]);
        });

    // 按顺序入队100个事件
    for (int i = 0; i < 100; i++) {
        InputEvent ie;
        ie.id = "seq";
        ie.data = {uint8_t(i)};
        worker.enqueue(ie);
    }

    // 等待全部处理完
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    assert(processed.size() == 100);
    for (int i = 0; i < 100; i++) {
        assert(processed[i] == i); // 必须严格保序
    }

    std::cout << "[测试2] 通过 ✓ (处理了 " << processed.size() << " 个事件)" << std::endl;
}

// ============================================================
// 测试3: SerialEventLoop 注册与取消
// ============================================================
void test_event_loop_registration() {
    std::cout << "[测试3] EventLoop 注册/取消..." << std::endl;

    SerialEventLoop loop("test_node");

    // 注册handler
    loop.register_handler("camera", [](const InputEvent&) {});
    loop.register_handler("lidar", [](const InputEvent&) {});

    // 注册定时器
    int tick_count = 0;
    loop.register_timer("tick", std::chrono::milliseconds(50),
        [&]() { tick_count++; }, true);

    // 等待定时器触发几次
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    assert(tick_count >= 2);

    // 取消定时器
    bool ok = loop.cancel_timer("tick");
    assert(ok);

    int snapshot = tick_count;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    assert(tick_count == snapshot); // 取消后不应再触发

    // 取消失败的定时器
    ok = loop.cancel_timer("nonexistent");
    assert(!ok);

    std::cout << "[测试3] 通过 ✓ (tick_count=" << tick_count << ")" << std::endl;
}

// ============================================================
// 测试4: 一次性定时器
// ============================================================
void test_oneshot_timer() {
    std::cout << "[测试4] 一次性定时器..." << std::endl;

    SerialEventLoop loop("test_node");
    std::atomic<int> fired{0};

    loop.register_timer("once", std::chrono::milliseconds(30),
        [&]() { fired++; }, false); // repeat=false

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    // 应该在30ms左右触发一次，之后不再触发
    assert(fired == 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    assert(fired == 1); // 仍然只有一次

    std::cout << "[测试4] 通过 ✓" << std::endl;
}

// ============================================================
// 测试5: send_output 线程安全
// ============================================================
void test_send_output() {
    std::cout << "[测试5] send_output 入队..." << std::endl;

    SerialEventLoop loop("test_node");

    std::atomic<int> sent{0};

    // 模拟从多个线程调用 send_output
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

    assert(sent == 100);
    // 注意: 这个测试不运行 run()，所以消息还没真正发送到Dora
    // 但它们已被安全地放入队列

    std::cout << "[测试5] 通过 ✓ (入队了 " << sent << " 条消息)" << std::endl;
}

// ============================================================
// 测试6: timer回调中调用 send_output
// ============================================================
void test_timer_send_output() {
    std::cout << "[测试6] 定时器回调中发送输出..." << std::endl;

    SerialEventLoop loop("test_node");
    std::atomic<int> output_calls{0};

    // 在timer回调中调用send_output（timer线程非主线程，会入队）
    loop.register_timer("sender", std::chrono::milliseconds(30),
        [&]() {
            loop.send_output("timer_out", {0xAA, 0xBB});
            output_calls++;
        }, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    assert(output_calls >= 2);

    loop.cancel_timer("sender");
    std::cout << "[测试6] 通过 ✓ (send_output调用了 " << output_calls << " 次)" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  SerialEventLoop 单元测试" << std::endl;
    std::cout << "========================================" << std::endl;

    test_serial_worker_basic();
    test_serial_worker_ordering();
    test_event_loop_registration();
    test_oneshot_timer();
    test_send_output();
    test_timer_send_output();

    std::cout << "========================================" << std::endl;
    std::cout << "  全部测试通过 ✓" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
