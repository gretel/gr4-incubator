// Generic libiio v0.26 TX block. Mirror of IIOSource.
//
// AD9361/Pluto convenience aliases (`center_frequency`, `sample_rate`,
// `bandwidth`, `tx_attenuation`, `rf_port`) auto-map to AD9361 IIO
// attribute paths only when `phy_device == "ad9361-phy"`. For other
// devices, leave aliases at defaults and use the generic `attributes` map.
//
// AD9361 LO mapping note: TX LO lives at altvoltage1/frequency on the
// ad9361-phy device (RX LO is altvoltage0). TX gain is exposed as
// `tx_attenuation` (positive dB), written internally as
// `hardwaregain = -tx_attenuation`.

#pragma once

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <gnuradio-4.0/iio/IIORaiiWrapper.hpp>

namespace gr::incubator::iio {

template<typename T>
struct IIOSink;

GR_REGISTER_BLOCK("gr::incubator::iio::IIOSink", gr::incubator::iio::IIOSink, ([T]), [ std::complex<float>, std::complex<int16_t> ])

template<typename T>
struct IIOSink : Block<IIOSink<T>> {
    static_assert(std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<int16_t>>, "IIOSink<T>: T must be std::complex<float> or std::complex<int16_t>");

    using Description = Doc<"@brief Generic libiio v0.26 TX sink. Primary use case: "
                            "AD9361 / Adalm-Pluto. Device-agnostic via uri / device / "
                            "phy_device + attributes property_map. AD9361 convenience "
                            "aliases (center_frequency, sample_rate, bandwidth, "
                            "tx_attenuation, rf_port) auto-apply when "
                            "phy_device==\"ad9361-phy\".">;
    using Base        = Block<IIOSink<T>>;
    using Base::Base; // expose property_map-init constructor

    IIOSink() = default;

    PortIn<T> in;

    // Connection
    std::string uri        = "local:";
    std::string device     = "cf-ad9361-dds-core-lpc"; // AD9361 TX streaming device
    std::string phy_device = "ad9361-phy";

    // I/Q channel names (output direction). Order [I, Q].
    std::vector<std::string> channels = {"voltage0", "voltage1"};

    // Generic raw IIO attributes; key: "<channel>/<attr>" or "<attr>".
    property_map attributes;

    // AD9361 convenience aliases (only honoured when phy_device=="ad9361-phy")
    double      center_frequency = 868'100'000.0;
    float       sample_rate      = 2'083'334.0F; // AD9361 minimum; float matches PortMetaInfo auto-forward convention
    double      bandwidth        = 200'000.0;
    double      tx_attenuation   = 10.0; // positive dB, written as hardwaregain = -tx_attenuation
    std::string rf_port          = "A";

    // DMA + timing
    gr::Size_t buffer_size         = 32'768U;
    gr::Size_t timeout_ms          = 1'000U;
    gr::Size_t tx_tail_pad_samples = 32'768U; // zero-pad before cancel for clean carrier ramp-down

    GR_MAKE_REFLECTABLE(IIOSink, in, uri, device, phy_device, channels, attributes, center_frequency, sample_rate, bandwidth, tx_attenuation, rf_port, buffer_size, timeout_ms, tx_tail_pad_samples);

    // ---------- lifecycle ---------------------------------------------------

    void start() { reinitDevice(); }

    void stop() noexcept {
        try {
            tailPadAndCancel();
            _buf.reset();
            _ctx.reset();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "IIOSink::stop: %s\n", e.what());
        } catch (...) {
            std::fprintf(stderr, "IIOSink::stop: unknown exception\n");
        }
    }

    void settingsChanged(const property_map& /*old_*/, const property_map& new_) {
        if (!_ctx) {
            return;
        }

        const bool needsFullReinit = new_.contains("uri") || new_.contains("device") || new_.contains("phy_device") || new_.contains("channels") || new_.contains("buffer_size");
        if (needsFullReinit) {
            reinitDevice();
            return;
        }

        if (new_.contains("timeout_ms")) {
            _ctx.setTimeout(static_cast<unsigned int>(timeout_ms));
        }

        if (isAd9361()) {
            if (new_.contains("center_frequency")) {
                applyAd9361CenterFrequency();
            }
            if (new_.contains("sample_rate")) {
                applyAd9361SampleRate();
            }
            if (new_.contains("bandwidth")) {
                applyAd9361Bandwidth();
            }
            if (new_.contains("tx_attenuation") || new_.contains("rf_port")) {
                applyAd9361PerChannel();
            }
        }

        // Note: the `attributes` property_map is declared but its hot-update
        // walker is parked for a follow-up (needs pmtv variant dispatch).
    }

    // ---------- processBulk -------------------------------------------------

    [[nodiscard]] gr::work::Status processBulk(InputSpanLike auto& input) {
        if (!_buf || _chans[0] == nullptr) {
            (void)input.consume(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        const std::ptrdiff_t step = _buf.step();
        if (step <= 0) {
            (void)input.consume(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        auto* const       start    = static_cast<std::byte*>(_buf.first(_chans[0]));
        auto* const       end      = static_cast<std::byte*>(_buf.end());
        const std::size_t capacity = static_cast<std::size_t>((end - start) / step);
        if (capacity == 0U) {
            (void)input.consume(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        // iio_buffer_push always transmits the full DMA buffer. A partial
        // fill (input.size() < capacity) followed by push would radiate stale
        // tail bytes between every short batch. Wait for a full batch instead;
        // the scheduler will accumulate upstream input and re-enter when ready.
        if (input.size() < capacity) {
            (void)input.consume(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        for (std::size_t i = 0; i < capacity; ++i) {
            std::byte*   p = start + i * step;
            std::int16_t i_raw{};
            std::int16_t q_raw{};
            if constexpr (std::is_same_v<T, std::complex<std::int16_t>>) {
                i_raw = input[i].real();
                q_raw = input[i].imag();
            } else {
                // AD9361 12-bit DAC; full-scale magnitude 32768 (LSB-aligned).
                constexpr float scale = 32768.0f;
                const float     fi    = std::clamp(input[i].real() * scale, -32768.0f, 32767.0f);
                const float     fq    = std::clamp(input[i].imag() * scale, -32768.0f, 32767.0f);
                i_raw                 = static_cast<std::int16_t>(fi);
                q_raw                 = static_cast<std::int16_t>(fq);
            }
            std::memcpy(p, &i_raw, sizeof(std::int16_t));
            std::memcpy(p + sizeof(std::int16_t), &q_raw, sizeof(std::int16_t));
        }

        const ssize_t bytes = _buf.push();
        if (bytes < 0) {
            const int err = -static_cast<int>(bytes);
            (void)input.consume(0U);
            switch (err) {
            case ETIMEDOUT: return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
            case EBADF: return gr::work::Status::DONE;
            default: throw gr::exception(std::format("IIOSink: iio_buffer_push failed errno={} ({})", err, std::strerror(err)));
            }
        }

        (void)input.consume(capacity);
        return gr::work::Status::OK;
    }

private:
    detail::Context               _ctx;
    ::iio_device*                 _phy       = nullptr;
    ::iio_device*                 _streamDev = nullptr;
    std::array<::iio_channel*, 2> _chans{};
    detail::Buffer                _buf;

    [[nodiscard]] bool isAd9361() const noexcept { return phy_device == "ad9361-phy"; }

    void reinitDevice() {
        _buf.cancel();
        _buf.reset();
        _ctx.reset();
        _phy       = nullptr;
        _streamDev = nullptr;
        _chans     = {};

        _ctx = detail::Context(uri);
        _ctx.setTimeout(static_cast<unsigned int>(timeout_ms));

        if (!phy_device.empty()) {
            _phy = _ctx.findDevice(phy_device);
        }
        _streamDev = _ctx.findDevice(device);

        if (channels.size() != 2) {
            throw gr::exception(std::format("IIOSink: channels.size()={} but only 2 (one I/Q pair) is supported in this version", channels.size()));
        }
        for (std::size_t i = 0; i < channels.size(); ++i) {
            ::iio_channel* ch = ::iio_device_find_channel(_streamDev, channels[i].c_str(), /*output=*/true);
            if (ch == nullptr) {
                throw gr::exception(std::format("iio_device_find_channel(device='{}', channel='{}', output) returned null", device, channels[i]));
            }
            detail::enableChannel(ch);
            _chans[i] = ch;
        }

        if (isAd9361()) {
            applyAd9361CenterFrequency();
            applyAd9361SampleRate();
            applyAd9361Bandwidth();
            applyAd9361PerChannel();
        }
        // `attributes` property_map walker parked for follow-up.

        _buf = detail::Buffer(_streamDev, static_cast<std::size_t>(buffer_size),
            /*cyclic=*/false);
        _buf.setBlockingMode(true);
    }

    void applyAd9361CenterFrequency() {
        if (_phy == nullptr) {
            return;
        }
        // TX LO on AD9361 is altvoltage1 (RX LO is altvoltage0).
        detail::writeAttrLL(::iio_device_find_channel(_phy, "altvoltage1", /*output=*/true), "frequency", static_cast<long long>(center_frequency));
    }

    void applyAd9361SampleRate() {
        if (_phy == nullptr) {
            return;
        }
        detail::writeAttrLL(::iio_device_find_channel(_phy, "voltage0", /*output=*/true), "sampling_frequency", static_cast<long long>(sample_rate));
    }

    void applyAd9361Bandwidth() {
        if (_phy == nullptr) {
            return;
        }
        detail::writeAttrLL(::iio_device_find_channel(_phy, "voltage0", /*output=*/true), "rf_bandwidth", static_cast<long long>(bandwidth));
    }

    void applyAd9361PerChannel() {
        if (_phy == nullptr) {
            return;
        }
        ::iio_channel* phyCh = ::iio_device_find_channel(_phy, "voltage0", /*output=*/true);
        if (phyCh == nullptr) {
            throw gr::exception("ad9361-phy voltage0 output channel not found");
        }
        // hardwaregain on TX path is negated: positive tx_attenuation = lower TX power
        detail::writeAttrLL(phyCh, "hardwaregain", -static_cast<long long>(tx_attenuation));
        detail::writeAttr(phyCh, "rf_port_select", rf_port);
    }

    void tailPadAndCancel() {
        if (!_buf || _chans[0] == nullptr || tx_tail_pad_samples == 0U) {
            _buf.cancel();
            return;
        }
        const std::ptrdiff_t step = _buf.step();
        if (step <= 0) {
            _buf.cancel();
            return;
        }
        auto* const       start    = static_cast<std::byte*>(_buf.first(_chans[0]));
        auto* const       end      = static_cast<std::byte*>(_buf.end());
        const std::size_t capacity = static_cast<std::size_t>((end - start) / step);
        if (capacity == 0U) {
            _buf.cancel();
            return;
        }
        const std::size_t rounds = (tx_tail_pad_samples + capacity - 1U) / capacity;

        for (std::size_t r = 0; r < rounds; ++r) {
            std::memset(start, 0, static_cast<std::size_t>(end - start));
            const ssize_t bytes = _buf.push();
            if (bytes < 0) {
                break; // -EBADF or other; we're shutting down anyway
            }
        }
        _buf.cancel();
    }
};

} // namespace gr::incubator::iio
