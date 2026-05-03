// IIO TX example: zero-pad transmit through IIOSink, stop after N samples
// or T seconds. Useful as a wiring smoke test (carrier null radiates) and
// as a starting point for replacing NullSource with a real signal source.
//
// Mirrors blocks/iio/apps/iio_capture.cpp shape.

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
#include <gnuradio-4.0/iio/IIOSink.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

#include <CLI/CLI.hpp>

int main(int argc, char** argv) {
    CLI::App app{"IIO TX: open libiio context, transmit zero samples through IIOSink"};

    std::string uri              = "local:";
    std::string device           = "cf-ad9361-dds-core-lpc";
    std::string phy_device       = "ad9361-phy";
    double      sample_rate      = 2'000'000.0;
    double      center_frequency = 868'100'000.0;
    double      bandwidth        = 200'000.0;
    double      tx_attenuation   = 10.0;
    std::string rf_port          = "A";
    std::size_t buffer_size      = 32'768;
    std::size_t timeout_ms       = 1'000;
    std::size_t tail_pad_samples = 32'768;
    std::size_t n_samples        = 1'000'000;
    double      timeout_s        = 0.0;

    app.add_option("--uri", uri, "libiio context URI (local:, ip:HOST, usb:...)");
    app.add_option("--device", device, "TX streaming device name");
    app.add_option("--phy", phy_device, "Tuning device name (alias target)");
    app.add_option("--rate", sample_rate, "Sample rate (Hz, AD9361 alias)");
    app.add_option("--freq", center_frequency, "TX center frequency (Hz, AD9361 alias)");
    app.add_option("--bw", bandwidth, "Bandwidth (Hz, AD9361 alias)");
    app.add_option("--atten", tx_attenuation, "TX attenuation (dB positive, AD9361 alias)");
    app.add_option("--rf-port", rf_port, "AD9361 rf_port_select (A, B)");
    app.add_option("--buffer", buffer_size, "DMA buffer size (samples, pow-of-2)");
    app.add_option("--timeout-ms", timeout_ms, "libiio context timeout (ms)");
    app.add_option("--tail-pad", tail_pad_samples, "Zero-pad samples on stop (clean carrier ramp-down)");
    app.add_option("-n,--samples", n_samples, "Number of samples to transmit before stopping");
    app.add_option("--timeout", timeout_s, "Wall-clock watchdog (s, 0 = disable)");

    CLI11_PARSE(app, argc, argv);

    using T = std::complex<float>;

    gr::Graph fg;

    auto& source = fg.emplaceBlock<gr::testing::NullSource<T>>({
        {"n_samples_max", static_cast<gr::Size_t>(n_samples)},
    });

    auto& sink = fg.emplaceBlock<gr::incubator::iio::IIOSink<T>>({
        {"uri", uri},
        {"device", device},
        {"phy_device", phy_device},
        {"sample_rate", sample_rate},
        {"center_frequency", center_frequency},
        {"bandwidth", bandwidth},
        {"tx_attenuation", tx_attenuation},
        {"rf_port", rf_port},
        {"buffer_size", static_cast<gr::Size_t>(buffer_size)},
        {"timeout_ms", static_cast<gr::Size_t>(timeout_ms)},
        {"tx_tail_pad_samples", static_cast<gr::Size_t>(tail_pad_samples)},
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
                source.requestStop();
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

    std::cout << "TX complete\n";
    return 0;
}
