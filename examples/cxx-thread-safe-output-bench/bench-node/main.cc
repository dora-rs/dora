#include "../build/dora-node-api.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// Benchmark contestant. BENCH_MODE selects behavior:
//   blocking : tick+image; image work runs inline (blocks the loop).
//   worker   : tick+image; image work offloaded to a thread (SafeOutputSender).
//   measure  : tick only (two-node config tick handler).
//   slow     : image only (two-node config image handler).
// All modes measure tick latency except slow.

// system_clock, not steady_clock: epoch is shared across processes.
static int64_t now_nanos()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

static int64_t decode_nanos(const rust::Vec<uint8_t> &d)
{
    int64_t t = 0;
    if (d.size() >= sizeof(t))
    {
        std::memcpy(&t, d.data(), sizeof(t));
    }
    return t;
}

static void print_stats(const std::string &mode, std::vector<double> &lat)
{
    if (lat.empty())
    {
        std::cout << "BENCH_RESULT mode=" << mode << " samples=0" << std::endl;
        return;
    }
    std::sort(lat.begin(), lat.end());
    double sum = 0;
    for (double v : lat)
        sum += v;
    double mean = sum / lat.size();
    double p50 = lat[lat.size() * 50 / 100];
    double p99 = lat[lat.size() * 99 / 100];
    double max = lat.back();

    std::cout << std::fixed << std::setprecision(2)
              << "BENCH_RESULT mode=" << mode
              << " samples=" << lat.size()
              << " mean_ms=" << mean
              << " p50_ms=" << p50
              << " p99_ms=" << p99
              << " max_ms=" << max << std::endl;
}

int main()
{
    const char *mode_env = std::getenv("BENCH_MODE");
    std::string mode = mode_env ? mode_env : "measure";
    std::cout << "[bench] starting in mode '" << mode << "'" << std::endl;

    auto dora_node = init_dora_node();

    std::optional<rust::Box<SafeOutputSender>> safe_holder;
    if (mode == "worker")
    {
        safe_holder.emplace(create_safe_output_sender(std::move(dora_node.send_output)));
    }

    std::vector<double> latencies_ms;
    std::vector<std::thread> workers;

    while (true)
    {
        auto event = dora_node.events->next();
        auto ty = event_type(event);

        if (ty == DoraEventType::AllInputsClosed || ty == DoraEventType::Stop)
        {
            break;
        }
        else if (ty == DoraEventType::Input)
        {
            auto input = event_as_input(std::move(event));
            std::string id(input.id);

            if (id == "tick")
            {
                int64_t emitted = decode_nanos(input.data);
                int64_t delta = now_nanos() - emitted;
                if (delta >= 0)
                {
                    latencies_ms.push_back(delta / 1e6);
                }
            }
            else if (id == "image")
            {
                if (mode == "worker")
                {
                    workers.emplace_back([&safe_holder]()
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        std::vector<uint8_t> out{1};
                        rust::Slice<const uint8_t> slice{out.data(), out.size()};
                        safe_send_output(**safe_holder, "result", slice, new_metadata());
                    });
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            }
        }
    }

    for (auto &w : workers)
    {
        w.join();
    }

    print_stats(mode, latencies_ms);

    return 0;
}
