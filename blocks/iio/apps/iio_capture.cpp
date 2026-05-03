// IIO capture example: open an IIO context, stream RX samples through
// IIOSource into a CountingSink, stop after N samples or T seconds.
//
// Mirrors blocks/soapysdr/apps/soapy_capture.cpp shape.

#include <atomic>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iostream>
#include <string>
#include <thread>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/iio/IIOSource.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

#include <CLI/CLI.hpp>

int main(int argc, char** argv) {
    CLI::App app{"IIO capture: open libiio context, read N samples, stop"};

    std::string uri              = "local:";
    std::string device           = "cf-ad9361-lpc";
    std::string phy_device       = "ad9361-phy";
    double      sample_rate      = 2'000'000.0;
    double      center_frequency = 868'100'000.0;
    double      bandwidth        = 200'000.0;
    double      gain             = 40.0;
    std::string gain_mode        = "slow_attack";
    std::string rf_port          = "A_BALANCED";
    std::size_t buffer_size      = 32'768;
    std::size_t timeout_ms       = 1'000;
    std::size_t max_overflow     = 10;
    std::size_t n_samples        = 1'000'000;
    double      timeout_s        = 0.0;

    app.add_option("--uri", uri, "libiio context URI (local:, ip:HOST, usb:...)");
    app.add_option("--device", device, "Streaming device name");
    app.add_option("--phy", phy_device, "Tuning device name (alias target)");
    app.add_option("--rate", sample_rate, "Sample rate (Hz, AD9361 alias)");
    app.add_option("--freq", center_frequency, "Center frequency (Hz, AD9361 alias)");
    app.add_option("--bw", bandwidth, "Bandwidth (Hz, AD9361 alias)");
    app.add_option("--gain", gain, "Gain (dB, AD9361 alias)");
    app.add_option("--gain-mode", gain_mode, "Gain control mode (manual|slow_attack|fast_attack)");
    app.add_option("--rf-port", rf_port, "AD9361 rf_port_select");
    app.add_option("--buffer", buffer_size, "DMA buffer size (samples, pow-of-2)");
    app.add_option("--timeout-ms", timeout_ms, "libiio context timeout (ms)");
    app.add_option("--max-overflow", max_overflow, "Refill overflow ceiling (0 = disable)");
    app.add_option("-n,--samples", n_samples, "Number of samples to capture before stopping");
    app.add_option("--timeout", timeout_s, "Wall-clock watchdog (s, 0 = disable)");

    CLI11_PARSE(app, argc, argv);

    using T = std::complex<float>;

    gr::Graph fg;

    auto& source = fg.emplaceBlock<gr::incubator::iio::IIOSource<T>>({
        {"uri", uri},
        {"device", device},
        {"phy_device", phy_device},
        {"sample_rate", sample_rate},
        {"center_frequency", center_frequency},
        {"bandwidth", bandwidth},
        {"gain", gain},
        {"gain_mode", gain_mode},
        {"rf_port", rf_port},
        {"buffer_size", static_cast<gr::Size_t>(buffer_size)},
        {"timeout_ms", static_cast<gr::Size_t>(timeout_ms)},
        {"max_overflow_count", static_cast<gr::Size_t>(max_overflow)},
    });

    auto& sink = fg.emplaceBlock<gr::testing::CountingSink<T>>({
        {"n_samples_max", static_cast<gr::Size_t>(n_samples)},
    });

    if (auto conn = fg.connect(source, gr::PortDefinition{"out"}, sink, gr::PortDefinition{"in"}); conn != gr::ConnectionResult::SUCCESS) {
        throw gr::exception(std::format("connect failed: {}", static_cast<int>(conn)));
    }

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreaded> sched;
    if (auto ret = sched.exchange(std::move(fg)); !ret) {
        throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
    }

    std::atomic<bool> done{false};
    std::thread       watchdog;
    if (timeout_s > 0.0) {
        watchdog = std::thread([&] {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
            while (!done.load(std::memory_order_relaxed) && std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (!done.load(std::memory_order_relaxed)) {
                sink.requestStop();
            }
        });
    }

    const auto ret = sched.runAndWait();
    done.store(true, std::memory_order_relaxed);
    if (watchdog.joinable()) {
        watchdog.join();
    }
    if (!ret.has_value()) {
        std::cerr << "scheduler error: " << ret.error().message << "\n";
        return 1;
    }

    std::cout << "Captured samples: " << sink.count << "\n";
    return 0;
}
