// Generic libiio v0.26 RX block. Primary use case: AD9361 on Adalm-Pluto /
// FISH Ball running tezuka_fw, but the API is device-agnostic — any
// libiio-supported device works through `uri` + `device` + `phy_device` +
// `attributes` property_map.
//
// AD9361/Pluto convenience aliases (`center_frequency`, `sample_rate`,
// `bandwidth`, `gain`, `gain_mode`, `rf_port`) are auto-mapped to AD9361
// IIO attribute paths only when `phy_device == "ad9361-phy"`. For other
// devices, leave aliases at defaults and use the generic `attributes` map.

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
#include <gnuradio-4.0/ValueHelper.hpp>

#include <gnuradio-4.0/iio/IIORaiiWrapper.hpp>

namespace gr::incubator::iio {

template<typename T>
struct IIOSource;

GR_REGISTER_BLOCK("gr::incubator::iio::IIOSource", gr::incubator::iio::IIOSource, ([T]), [ std::complex<float>, std::complex<int16_t> ])

template<typename T>
struct IIOSource : Block<IIOSource<T>> {
    static_assert(std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<int16_t>>, "IIOSource<T>: T must be std::complex<float> or std::complex<int16_t>");

    using Description = Doc<"@brief Generic libiio v0.26 RX source. Primary use case: "
                            "AD9361 / Adalm-Pluto. Device-agnostic via uri / device / "
                            "phy_device + attributes property_map. AD9361 convenience "
                            "aliases (center_frequency, sample_rate, bandwidth, gain, "
                            "gain_mode, rf_port) auto-apply when phy_device==\"ad9361-phy\".">;
    using Base        = Block<IIOSource<T>>;
    using Base::Base; // expose property_map-init constructor

    IIOSource() = default;

    PortOut<T> out;

    // Connection
    std::string uri        = "local:";
    std::string device     = "cf-ad9361-lpc"; // streaming device
    std::string phy_device = "ad9361-phy";    // tuning device (alias target)

    // I/Q channel names (size 2 for one I/Q pair). Must be in order [I, Q].
    std::vector<std::string> channels = {"voltage0", "voltage1"};

    // Generic raw IIO attributes. Key: "<channel>/<attr>" or just "<attr>"
    // for device-level. Applied AFTER convenience aliases (so the map
    // overrides aliases). Values are written as their stringified form.
    property_map attributes;

    // AD9361 convenience aliases (only honoured when phy_device=="ad9361-phy")
    double      center_frequency = 868'100'000.0;
    float       sample_rate      = 2'083'334.0f; // AD9361 minimum
    double      bandwidth        = 200'000.0;
    double      gain             = 40.0;
    std::string gain_mode        = "slow_attack";
    std::string rf_port          = "A_BALANCED";

    // DMA + timing
    gr::Size_t buffer_size        = 32'768U;
    gr::Size_t timeout_ms         = 1'000U;
    gr::Size_t max_overflow_count = 10U;

    GR_MAKE_REFLECTABLE(IIOSource, out, uri, device, phy_device, channels, attributes, center_frequency, sample_rate, bandwidth, gain, gain_mode, rf_port, buffer_size, timeout_ms, max_overflow_count);

    // ---------- lifecycle ---------------------------------------------------

    void start() { reinitDevice(); }

    void stop() noexcept {
        try {
            _buf.cancel();
            _buf.reset();
            _ctx.reset();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "IIOSource::stop: %s\n", e.what());
        } catch (...) {
            std::fprintf(stderr, "IIOSource::stop: unknown exception\n");
        }
    }

    // GR4 calls settingsChanged on every property write. Hot-update what
    // libiio v0.26 lets us (LO frequency, gains, attributes); fall back to
    // a full reinit when topology-defining properties change.
    void settingsChanged(const property_map& /*old_*/, const property_map& new_) {
        if (!_ctx) {
            return; // not started yet — start() will pick up the new state
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
            if (new_.contains("gain") || new_.contains("gain_mode") || new_.contains("rf_port")) {
                applyAd9361PerChannel();
            }
        }

        if (new_.contains("attributes")) {
            applyAttributes(/*isOutput=*/false);
        }
    }

    // ---------- processBulk -------------------------------------------------
    [[nodiscard]] gr::work::Status processBulk(OutputSpanLike auto& output) {
        if (!_buf) {
            output.publish(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        const ssize_t bytes = _buf.refill();
        if (bytes < 0) {
            const int err = -static_cast<int>(bytes);
            output.publish(0U);
            switch (err) {
            case ETIMEDOUT:
                // Idle/timeout on a source — return OK with publish(0); mirrors
                // SoapyRx (gr::incubator::soapysdr::SoapyRx::processBulk).
                return gr::work::Status::OK;
            case EBADF:
                // Cancellation from stop()/pause() — orderly shutdown.
                return gr::work::Status::DONE;
            default: bumpOverflow(err); return gr::work::Status::OK;
            }
        }

        const std::ptrdiff_t step = _buf.step();
        if (step <= 0 || _chans[0] == nullptr) {
            output.publish(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        const auto* const start = static_cast<const std::byte*>(_buf.first(_chans[0]));
        const auto* const end   = static_cast<const std::byte*>(_buf.end());
        const std::size_t scans = static_cast<std::size_t>((end - start) / step);
        const std::size_t n     = std::min<std::size_t>(scans, output.size());

        for (std::size_t i = 0; i < n; ++i) {
            const std::byte* p = start + static_cast<std::ptrdiff_t>(i) * step;
            std::int16_t     i_raw{};
            std::int16_t     q_raw{};
            std::memcpy(&i_raw, p, sizeof(std::int16_t));
            std::memcpy(&q_raw, p + sizeof(std::int16_t), sizeof(std::int16_t));
            if constexpr (std::is_same_v<T, std::complex<std::int16_t>>) {
                output[i] = std::complex<std::int16_t>(i_raw, q_raw);
            } else {
                // AD9361 12-bit ADC, sign-extended in 16-bit container; LSB
                // alignment gives a full-scale magnitude of 2048.
                constexpr float scale = 1.0f / 2048.0f;
                output[i]             = std::complex<float>(static_cast<float>(i_raw) * scale, static_cast<float>(q_raw) * scale);
            }
        }

        output.publish(n);
        _consecutiveErrorCount = 0;
        // SoapySource pattern: signal progress so the singleThreadedBlocking
        // scheduler doesn't park us on the inactivity watchdog.
        this->progress->incrementAndGet();
        this->progress->notify_all();
        return gr::work::Status::OK;
    }

private:
    // ---------- device init helpers ----------------------------------------

    detail::Context               _ctx;
    ::iio_device*                 _phy       = nullptr;
    ::iio_device*                 _streamDev = nullptr;
    std::array<::iio_channel*, 2> _chans{};
    detail::Buffer                _buf;
    std::size_t                   _consecutiveErrorCount = 0;
    std::size_t                   _overflowCount         = 0;
    std::size_t                   _totalOverflowCount    = 0;

    [[nodiscard]] bool isAd9361() const noexcept { return phy_device == "ad9361-phy"; }

    void reinitDevice() {
        // Tear down old state first (cancel + destroy buffer + reset context).
        _buf.cancel();
        _buf.reset();
        _ctx.reset();
        _phy                   = nullptr;
        _streamDev             = nullptr;
        _chans                 = {};
        _consecutiveErrorCount = 0;

        // Open new context.
        _ctx = detail::Context(uri);
        _ctx.setTimeout(static_cast<unsigned int>(timeout_ms));

        // phy_device is optional for non-ad9361 deployments; resolve when set
        // (many IIO devices have a separate tuning device).
        if (!phy_device.empty()) {
            _phy = _ctx.findDevice(phy_device);
        }
        _streamDev = _ctx.findDevice(device);

        // Resolve + enable channels.
        if (channels.size() != 2) {
            throw gr::exception(std::format("IIOSource: channels.size()={} but only 2 (one I/Q pair) is supported in this version", channels.size()));
        }
        for (std::size_t i = 0; i < channels.size(); ++i) {
            ::iio_channel* ch = ::iio_device_find_channel(_streamDev, channels[i].c_str(), /*output=*/false);
            if (ch == nullptr) {
                throw gr::exception(std::format("iio_device_find_channel(device='{}', channel='{}', input) returned null", device, channels[i]));
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
        // Generic attributes walker: applies AFTER convenience aliases so
        // the map overrides the AD9361 convenience values.
        applyAttributes(/*isOutput=*/false);

        // Allocate DMA buffer + force blocking semantics.
        _buf = detail::Buffer(_streamDev, static_cast<std::size_t>(buffer_size),
            /*cyclic=*/false);
        _buf.setBlockingMode(true);
    }

    void applyAd9361CenterFrequency() {
        if (_phy == nullptr) {
            return;
        }
        detail::writeAttrLL(::iio_device_find_channel(_phy, "altvoltage0", /*output=*/true), "frequency", static_cast<long long>(center_frequency));
    }

    void applyAd9361SampleRate() {
        if (_phy == nullptr) {
            return;
        }
        detail::writeAttrLL(::iio_device_find_channel(_phy, "voltage0", /*output=*/false), "sampling_frequency", static_cast<long long>(sample_rate));
    }

    void applyAd9361Bandwidth() {
        if (_phy == nullptr) {
            return;
        }
        detail::writeAttrLL(::iio_device_find_channel(_phy, "voltage0", /*output=*/false), "rf_bandwidth", static_cast<long long>(bandwidth));
    }

    void applyAd9361PerChannel() {
        if (_phy == nullptr) {
            return;
        }
        ::iio_channel* phyCh = ::iio_device_find_channel(_phy, "voltage0", /*output=*/false);
        if (phyCh == nullptr) {
            throw gr::exception("ad9361-phy voltage0 input channel not found");
        }
        detail::writeAttr(phyCh, "gain_control_mode", gain_mode);
        if (gain_mode == "manual") {
            detail::writeAttrLL(phyCh, "hardwaregain", static_cast<long long>(gain));
        }
        // rf_port_select may not support all values on all firmware
        // (e.g. Pluto tezuka_fw rejects B_BALANCED). Best-effort: fall
        // back to firmware default if the write fails.
        if (::iio_channel_attr_write(phyCh, "rf_port_select", rf_port.c_str()) < 0) {
            std::fprintf(stderr, "IIOSource: rf_port_select='%s' on voltage0 ignored (firmware default used)\n", rf_port.c_str());
        }
    }

    void applyAttributes(bool isOutput) {
        if (attributes.empty()) {
            return;
        }
        ::iio_device* dev = _phy ? _phy : _streamDev;
        for (const auto& [key, value] : attributes) {
            const auto slash = key.find('/');
            std::string   attrName;
            ::iio_channel* ch = nullptr;

            if (slash != decltype(key)::npos) {
                const std::string chName(key.begin(), key.begin() + static_cast<long>(slash));
                attrName = std::string(key.begin() + static_cast<long>(slash) + 1, key.end());
                ch       = ::iio_device_find_channel(dev, chName.c_str(), isOutput);
                if (ch == nullptr && _streamDev != _phy && dev == _phy) {
                    ch = ::iio_device_find_channel(_streamDev, chName.c_str(), isOutput);
                }
            } else {
                attrName = std::string(key);
            }

            pmt::ValueVisitor([&](const auto& v) {
                using ValT = std::decay_t<decltype(v)>;
                if constexpr (std::is_integral_v<ValT> || std::is_floating_point_v<ValT>) {
                    const long long ll = static_cast<long long>(v);
                    if (ch) detail::writeAttrLL(ch, attrName, ll);
                    else    detail::writeAttrLL(dev, attrName, ll);
                } else if constexpr (std::is_same_v<ValT, std::string_view>) {
                    const std::string s(v);
                    if (ch) detail::writeAttr(ch, attrName, s);
                    else    detail::writeAttr(dev, attrName, s);
                }
            }).visit(value);
        }
    }

    void bumpOverflow(int err) {
        ++_consecutiveErrorCount;
        ++_overflowCount;
        ++_totalOverflowCount;
        // Rate-limited stderr at 1, 10, 100, 1000.
        if (_overflowCount == 1U || _overflowCount == 10U || _overflowCount == 100U || _overflowCount == 1000U) {
            std::fprintf(stderr, "IIOSource: refill error errno=%d (count=%zu, total=%zu)\n", err, _overflowCount, _totalOverflowCount);
        }
        if (max_overflow_count != 0U && _overflowCount > max_overflow_count) {
            throw gr::exception(std::format("IIOSource: refill overflow threshold exceeded (count={}, max={}, last errno={})", _overflowCount, max_overflow_count, err));
        }
    }
};

} // namespace gr::incubator::iio
