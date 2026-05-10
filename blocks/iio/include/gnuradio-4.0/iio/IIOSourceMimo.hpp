// libiio v0.26 RX block for AD9361 in 2R2T mode. Sibling of IIOSource —
// not a template specialisation: 2R2T has its own buffer layout (4xi16
// per DMA scan), its own per-physical-chain rf_port_select, and a
// distinct gr port surface (out0 / out1). Keeping the single-stream
// IIOSource synchronous-DMA model here too — no IO thread, no
// streamWriter direct-publish path — matches the existing block's
// blocking semantics and downstream LoRa decoders' processBulk
// expectations.
//
// Channel ordering must be [I0, Q0, I1, Q1]: the AD9361 DMA layout for
// a 2R2T capture is 4 interleaved int16 samples per scan. The TPC
// (cf-ad9361-lpc) device exposes voltage0..voltage3 as RX-input scan
// elements; their `index` attribute (0..3) is the position inside one
// step. We require all four enabled in that order.
//
// AD9361 mode bit: `adi,2rx-2tx-mode-enable` is a debug-attribute on
// `ad9361-phy`. tezuka_fw images already configure 2R2T at boot via
// device-tree, so the helper's default behaviour is *not* to flip the
// bit at runtime (a runtime flip would tear down the streaming chain
// and is rarely safe outside calibration windows). Override
// `set_mode_at_init` if your firmware ships in 1R1T and you must
// switch.

#pragma once

#include <algorithm>
#include <array>
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
struct IIOSourceMimo;

GR_REGISTER_BLOCK("gr::incubator::iio::IIOSourceMimo", gr::incubator::iio::IIOSourceMimo, ([T]), [ std::complex<float>, std::complex<int16_t> ])

template<typename T>
struct IIOSourceMimo : Block<IIOSourceMimo<T>> {
    static_assert(std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<int16_t>>, "IIOSourceMimo<T>: T must be std::complex<float> or std::complex<int16_t>");

    using Description = Doc<"@brief libiio v0.26 RX source for AD9361 2R2T (dual RX). "
                            "Captures voltage0..voltage3 from cf-ad9361-lpc and "
                            "demuxes the 4-channel DMA stream into two gr ports "
                            "(out0=chain A, out1=chain B). Per-chain rf_port_select "
                            "is configured via rx_chains. AD9361 mode bit is left "
                            "to firmware unless set_mode_at_init=true.">;
    using Base        = Block<IIOSourceMimo<T>>;
    using Base::Base;

    IIOSourceMimo() = default;

    PortOut<T> out0;
    PortOut<T> out1;

    // Connection
    std::string uri        = "local:";
    std::string device     = "cf-ad9361-lpc";
    std::string phy_device = "ad9361-phy";

    // Four channels, [I0, Q0, I1, Q1] order. Must match the device's
    // scan-element layout (AD9361 default).
    std::vector<std::string> channels = {"voltage0", "voltage1", "voltage2", "voltage3"};

    // Per-physical-chain rf_port_select. Length must equal the I/Q pair
    // count (=2 for 2R2T). Written to voltage0 and voltage1 input
    // attributes on the phy device.
    std::vector<std::string> rx_chains = {"A_BALANCED", "B_BALANCED"};

    // Generic raw IIO attributes. Applied AFTER convenience aliases.
    property_map attributes;

    // AD9361 convenience aliases (only honoured when phy_device=="ad9361-phy")
    double      center_frequency = 868'100'000.0;
    float       sample_rate      = 2'083'334.0f;
    double      bandwidth        = 200'000.0;
    double      gain             = 40.0;
    std::string gain_mode        = "slow_attack";
    bool        set_mode_at_init = false; // flip adi,2rx-2tx-mode-enable=1 in reinit

    // DMA + timing
    gr::Size_t buffer_size        = 32'768U;
    gr::Size_t timeout_ms         = 1'000U;
    gr::Size_t max_overflow_count = 10U;

    GR_MAKE_REFLECTABLE(IIOSourceMimo, out0, out1, uri, device, phy_device, channels, rx_chains, attributes, center_frequency, sample_rate, bandwidth, gain, gain_mode, set_mode_at_init, buffer_size, timeout_ms, max_overflow_count);

    // ---------- lifecycle ---------------------------------------------------

    void start() { reinitDevice(); }

    void stop() noexcept {
        try {
            _buf.cancel();
            _buf.reset();
            _ctx.reset();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "IIOSourceMimo::stop: %s\n", e.what());
        } catch (...) {
            std::fprintf(stderr, "IIOSourceMimo::stop: unknown exception\n");
        }
    }

    void settingsChanged(const property_map& /*old_*/, const property_map& new_) {
        if (!_ctx) {
            return;
        }
        const bool needsFullReinit = new_.contains("uri") || new_.contains("device") || new_.contains("phy_device") || new_.contains("channels") || new_.contains("buffer_size") || new_.contains("set_mode_at_init");
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
            if (new_.contains("gain") || new_.contains("gain_mode") || new_.contains("rx_chains")) {
                applyAd9361PerChannel();
            }
        }
        if (new_.contains("attributes")) {
            applyAttributes(/*isOutput=*/false);
        }
    }

    // ---------- processBulk -------------------------------------------------

    [[nodiscard]] gr::work::Status processBulk(OutputSpanLike auto& output0, OutputSpanLike auto& output1) {
        if (!_buf) {
            output0.publish(0U);
            output1.publish(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        const ssize_t bytes = _buf.refill();
        if (bytes < 0) {
            const int err = -static_cast<int>(bytes);
            output0.publish(0U);
            output1.publish(0U);
            switch (err) {
            case ETIMEDOUT: return gr::work::Status::OK;
            case EBADF: return gr::work::Status::DONE;
            default: bumpOverflow(err); return gr::work::Status::OK;
            }
        }

        const std::ptrdiff_t step = _buf.step();
        if (step <= 0 || _chans[0] == nullptr) {
            output0.publish(0U);
            output1.publish(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        const auto* const start = static_cast<const std::byte*>(_buf.first(_chans[0]));
        const auto* const end   = static_cast<const std::byte*>(_buf.end());
        const std::size_t scans = static_cast<std::size_t>((end - start) / step);
        const std::size_t n     = std::min({scans, output0.size(), output1.size()});

        // Each step is [I0, Q0, I1, Q1] = 4 * sizeof(int16_t) = 8 bytes for AD9361.
        // _buf.first() gives the first I-sample of channel 0; channel 1's I is at
        // offset +2 * sizeof(int16_t) within the same step.
        constexpr std::ptrdiff_t kPairBytes = 4; // I + Q = 2 * int16_t
        for (std::size_t i = 0; i < n; ++i) {
            const std::byte* p0 = start + static_cast<std::ptrdiff_t>(i) * step;
            const std::byte* p1 = p0 + kPairBytes;

            std::int16_t i0_raw{};
            std::int16_t q0_raw{};
            std::int16_t i1_raw{};
            std::int16_t q1_raw{};
            std::memcpy(&i0_raw, p0, sizeof(std::int16_t));
            std::memcpy(&q0_raw, p0 + sizeof(std::int16_t), sizeof(std::int16_t));
            std::memcpy(&i1_raw, p1, sizeof(std::int16_t));
            std::memcpy(&q1_raw, p1 + sizeof(std::int16_t), sizeof(std::int16_t));

            if constexpr (std::is_same_v<T, std::complex<std::int16_t>>) {
                output0[i] = std::complex<std::int16_t>(i0_raw, q0_raw);
                output1[i] = std::complex<std::int16_t>(i1_raw, q1_raw);
            } else {
                constexpr float scale = 1.0f / 2048.0f;
                output0[i]            = std::complex<float>(static_cast<float>(i0_raw) * scale, static_cast<float>(q0_raw) * scale);
                output1[i]            = std::complex<float>(static_cast<float>(i1_raw) * scale, static_cast<float>(q1_raw) * scale);
            }
        }

        output0.publish(n);
        output1.publish(n);
        _consecutiveErrorCount = 0;
        this->progress->incrementAndGet();
        this->progress->notify_all();
        return gr::work::Status::OK;
    }

private:
    detail::Context               _ctx;
    ::iio_device*                 _phy       = nullptr;
    ::iio_device*                 _streamDev = nullptr;
    std::array<::iio_channel*, 4> _chans{};
    detail::Buffer                _buf;
    std::size_t                   _consecutiveErrorCount = 0;
    std::size_t                   _overflowCount         = 0;
    std::size_t                   _totalOverflowCount    = 0;

    [[nodiscard]] bool isAd9361() const noexcept { return phy_device == "ad9361-phy"; }

    void reinitDevice() {
        _buf.cancel();
        _buf.reset();
        _ctx.reset();
        _phy                   = nullptr;
        _streamDev             = nullptr;
        _chans                 = {};
        _consecutiveErrorCount = 0;

        _ctx = detail::Context(uri);
        _ctx.setTimeout(static_cast<unsigned int>(timeout_ms));

        if (!phy_device.empty()) {
            _phy = _ctx.findDevice(phy_device);
        }
        _streamDev = _ctx.findDevice(device);

        if (channels.size() != 4) {
            throw gr::exception(std::format("IIOSourceMimo: channels.size()={} but 4 (two I/Q pairs, [I0,Q0,I1,Q1]) is required", channels.size()));
        }
        if (rx_chains.size() != 2) {
            throw gr::exception(std::format("IIOSourceMimo: rx_chains.size()={} but 2 (one per physical RX chain) is required", rx_chains.size()));
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
            if (set_mode_at_init && _phy != nullptr) {
                // Best-effort: ensm_mode is a device-level attribute; debug-attribute
                // adi,2rx-2tx-mode-enable lives at the device root too. Tolerate
                // failure (older drivers may not expose it as writable).
                try {
                    detail::writeAttr(_phy, "ensm_mode", "alert");
                    detail::writeAttr(_phy, "adi,2rx-2tx-mode-enable", "1");
                    detail::writeAttr(_phy, "ensm_mode", "fdd");
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "IIOSourceMimo: 2R2T mode set skipped: %s\n", e.what());
                }
            }
            applyAd9361CenterFrequency();
            applyAd9361SampleRate();
            applyAd9361Bandwidth();
            applyAd9361PerChannel();
        }
        applyAttributes(/*isOutput=*/false);

        _buf = detail::Buffer(_streamDev, static_cast<std::size_t>(buffer_size), /*cyclic=*/false);
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
        // Walk both physical chains: voltage0 = chain A, voltage1 = chain B.
        // gain_mode + gain are not chain-specific in tezuka_fw's exposed
        // attribute set (gain_control_mode_available is a single string per
        // input pair), so write once per chain to surface any per-chain
        // overrides cleanly.
        const std::array<const char*, 2> phyNames{"voltage0", "voltage1"};
        for (std::size_t i = 0; i < phyNames.size(); ++i) {
            ::iio_channel* phyCh = ::iio_device_find_channel(_phy, phyNames[i], /*output=*/false);
            if (phyCh == nullptr) {
                throw gr::exception(std::format("ad9361-phy {} input channel not found", phyNames[i]));
            }
            detail::writeAttr(phyCh, "gain_control_mode", gain_mode);
            if (gain_mode == "manual") {
                detail::writeAttrLL(phyCh, "hardwaregain", static_cast<long long>(gain));
            }
            detail::writeAttr(phyCh, "rf_port_select", rx_chains[i]);
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
        if (_overflowCount == 1U || _overflowCount == 10U || _overflowCount == 100U || _overflowCount == 1000U) {
            std::fprintf(stderr, "IIOSourceMimo: refill error errno=%d (count=%zu, total=%zu)\n", err, _overflowCount, _totalOverflowCount);
        }
        if (max_overflow_count != 0U && _overflowCount > max_overflow_count) {
            throw gr::exception(std::format("IIOSourceMimo: refill overflow threshold exceeded (count={}, max={}, last errno={})", _overflowCount, max_overflow_count, err));
        }
    }
};

} // namespace gr::incubator::iio
