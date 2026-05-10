// libiio v0.26 TX block for AD9361 in 2R2T mode. Sibling of IIOSink —
// not a template specialisation: 2R2T has its own buffer layout
// (4xi16 per DMA scan, [I0,Q0,I1,Q1]) and per-physical-chain
// rf_port_select. Synchronous DMA model identical to IIOSink (no
// IO thread).
//
// AD9361 LO mapping note: TX LO lives at altvoltage1/frequency on the
// ad9361-phy device. tx_gain mirrors the IIOSink convention (0..89 dB,
// higher = louder), written internally as hardwaregain = tx_gain - 89
// (always negative, 0 = full scale).

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
struct IIOSinkMimo;

GR_REGISTER_BLOCK("gr::incubator::iio::IIOSinkMimo", gr::incubator::iio::IIOSinkMimo, ([T]), [ std::complex<float>, std::complex<int16_t> ])

template<typename T>
struct IIOSinkMimo : Block<IIOSinkMimo<T>> {
    static_assert(std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<int16_t>>, "IIOSinkMimo<T>: T must be std::complex<float> or std::complex<int16_t>");

    using Description = Doc<"@brief libiio v0.26 TX sink for AD9361 2R2T (dual TX). "
                            "Consumes two gr ports (in0=chain A, in1=chain B), "
                            "interleaves into [I0,Q0,I1,Q1] DMA scans, and pushes "
                            "to cf-ad9361-dds-core-lpc. Per-chain rf_port_select "
                            "via tx_chains.">;
    using Base        = Block<IIOSinkMimo<T>>;
    using Base::Base;

    IIOSinkMimo() = default;

    PortIn<T> in0;
    PortIn<T> in1;

    std::string uri        = "local:";
    std::string device     = "cf-ad9361-dds-core-lpc";
    std::string phy_device = "ad9361-phy";

    std::vector<std::string> channels  = {"voltage0", "voltage1", "voltage2", "voltage3"};
    std::vector<std::string> tx_chains = {"A", "B"};
    property_map             attributes;

    double      center_frequency = 868'100'000.0;
    float       sample_rate      = 2'083'334.0F;
    double      bandwidth        = 200'000.0;
    double      tx_gain          = 20.0;
    bool        set_mode_at_init = false;

    gr::Size_t buffer_size         = 32'768U;
    gr::Size_t timeout_ms          = 1'000U;
    gr::Size_t tx_tail_pad_samples = 32'768U;
    bool       tx_lo_powerdown     = true;

    GR_MAKE_REFLECTABLE(IIOSinkMimo, in0, in1, uri, device, phy_device, channels, tx_chains, attributes, center_frequency, sample_rate, bandwidth, tx_gain, set_mode_at_init, buffer_size, timeout_ms, tx_tail_pad_samples, tx_lo_powerdown);

    void start() { reinitDevice(); }

    void stop() noexcept {
        try {
            tailPadAndCancel();
            _buf.reset();
            _ctx.reset();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "IIOSinkMimo::stop: %s\n", e.what());
        } catch (...) {
            std::fprintf(stderr, "IIOSinkMimo::stop: unknown exception\n");
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
            if (new_.contains("tx_gain") || new_.contains("tx_chains")) {
                applyAd9361PerChannel();
            }
        }
        if (new_.contains("attributes")) {
            applyAttributes(/*isOutput=*/true);
        }
    }

    [[nodiscard]] gr::work::Status processBulk(InputSpanLike auto& input0, InputSpanLike auto& input1) {
        if (!_buf || _chans[0] == nullptr) {
            (void)input0.consume(0U);
            (void)input1.consume(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        const std::ptrdiff_t step = _buf.step();
        if (step <= 0) {
            (void)input0.consume(0U);
            (void)input1.consume(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        auto* const       start    = static_cast<std::byte*>(_buf.first(_chans[0]));
        auto* const       end      = static_cast<std::byte*>(_buf.end());
        const std::size_t capacity = static_cast<std::size_t>((end - start) / step);
        if (capacity == 0U) {
            (void)input0.consume(0U);
            (void)input1.consume(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        const std::size_t n = std::min({input0.size(), input1.size(), capacity});

        constexpr std::ptrdiff_t kPairBytes = 4; // I + Q = 2 * int16_t
        for (std::size_t i = 0; i < n; ++i) {
            std::byte*   p0 = start + static_cast<std::ptrdiff_t>(i) * step;
            std::byte*   p1 = p0 + kPairBytes;
            std::int16_t i0_raw{};
            std::int16_t q0_raw{};
            std::int16_t i1_raw{};
            std::int16_t q1_raw{};
            if constexpr (std::is_same_v<T, std::complex<std::int16_t>>) {
                i0_raw = input0[i].real();
                q0_raw = input0[i].imag();
                i1_raw = input1[i].real();
                q1_raw = input1[i].imag();
            } else {
                constexpr float scale = 32768.0f;
                const float     fi0   = std::clamp(input0[i].real() * scale, -32768.0f, 32767.0f);
                const float     fq0   = std::clamp(input0[i].imag() * scale, -32768.0f, 32767.0f);
                const float     fi1   = std::clamp(input1[i].real() * scale, -32768.0f, 32767.0f);
                const float     fq1   = std::clamp(input1[i].imag() * scale, -32768.0f, 32767.0f);
                i0_raw                = static_cast<std::int16_t>(fi0);
                q0_raw                = static_cast<std::int16_t>(fq0);
                i1_raw                = static_cast<std::int16_t>(fi1);
                q1_raw                = static_cast<std::int16_t>(fq1);
            }
            std::memcpy(p0, &i0_raw, sizeof(std::int16_t));
            std::memcpy(p0 + sizeof(std::int16_t), &q0_raw, sizeof(std::int16_t));
            std::memcpy(p1, &i1_raw, sizeof(std::int16_t));
            std::memcpy(p1 + sizeof(std::int16_t), &q1_raw, sizeof(std::int16_t));
        }
        if (n < capacity) {
            std::memset(start + static_cast<std::ptrdiff_t>(n) * step, 0,
                static_cast<std::size_t>(capacity - n) * static_cast<std::size_t>(step));
        }

        // Ensure TX LO is powered up before pushing data.  The driver may
        // leave it powered down after init (ENSM transition through ALERT,
        // previous shutdown, etc.).  Powering it here, right before push,
        // is the most reliable point — it works regardless of init ordering.
        if (!_txLoWarmed && _phy != nullptr) {
            if (auto* txLo = ::iio_device_find_channel(_phy, "altvoltage1", /*output=*/true)) {
                try {
                    detail::writeAttrLL(txLo, "powerdown", 0LL);
                } catch (const std::exception&) {
                }
            }
            _txLoWarmed = true;
        }

        const ssize_t bytes = _buf.push();
        if (bytes < 0) {
            const int err = -static_cast<int>(bytes);
            (void)input0.consume(0U);
            (void)input1.consume(0U);
            switch (err) {
            case ETIMEDOUT: return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
            case EBADF: return gr::work::Status::DONE;
            default: throw gr::exception(std::format("IIOSinkMimo: iio_buffer_push failed errno={} ({})", err, std::strerror(err)));
            }
        }

        (void)input0.consume(n);
        (void)input1.consume(n);
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
    bool                          _txLoWarmed{false};

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

        if (channels.size() != 4) {
            throw gr::exception(std::format("IIOSinkMimo: channels.size()={} but 4 (two I/Q pairs) is required", channels.size()));
        }
        if (tx_chains.size() != 2) {
            throw gr::exception(std::format("IIOSinkMimo: tx_chains.size()={} but 2 (one per physical TX chain) is required", tx_chains.size()));
        }

        for (std::size_t i = 0; i < channels.size(); ++i) {
            ::iio_channel* ch = ::iio_device_find_channel(_streamDev, channels[i].c_str(), /*output=*/true);
            if (ch == nullptr) {
                throw gr::exception(std::format("iio_device_find_channel(device='{}', channel='{}', output) returned null", device, channels[i]));
            }
            detail::enableChannel(ch);
            _chans[i] = ch;
        }

        // Power up TX LO before any mode transition.  The firmware may
        // leave it powered down after a previous shutdown, and the ENSM
        // transition through ALERT (if set_mode_at_init is true) would
        // power it down again during mode set — but we apply this AFTER
        // the transition so the LO stays on during buffer push.
        if (_phy != nullptr) {
            if (auto* txLo = ::iio_device_find_channel(_phy, "altvoltage1", /*output=*/true)) {
                try {
                    detail::writeAttrLL(txLo, "powerdown", 0LL);
                } catch (const std::exception&) {
                }
            }
        }

        if (isAd9361()) {
            if (set_mode_at_init && _phy != nullptr) {
                try {
                    detail::writeAttr(_phy, "ensm_mode", "alert");
                    detail::writeDebugAttr(_phy, "adi,2rx-2tx-mode-enable", "1");
                    detail::writeAttr(_phy, "ensm_mode", "fdd");
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "IIOSinkMimo: 2R2T mode set skipped: %s\n", e.what());
                }
            }
            applyAd9361CenterFrequency();
            applyAd9361SampleRate();
            applyAd9361Bandwidth();
            applyAd9361PerChannel();
        }
        applyAttributes(/*isOutput=*/true);

        _buf = detail::Buffer(_streamDev, static_cast<std::size_t>(buffer_size), /*cyclic=*/false);
        _buf.setBlockingMode(true);
    }

    void applyAd9361CenterFrequency() {
        if (_phy == nullptr) {
            return;
        }
        auto* txLo = ::iio_device_find_channel(_phy, "altvoltage1", /*output=*/true);
        if (txLo == nullptr) {
            return;
        }
        detail::writeAttrLL(txLo, "frequency", static_cast<long long>(center_frequency));
        // Ensure TX LO is powered up — firmware may leave it down after
        // a previous shutdown or if the ENSM transition skipped LO enable.
        try {
            detail::writeAttrLL(txLo, "powerdown", 0LL);
        } catch (const std::exception&) {
            // non-fatal; LO may already be up
        }
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
        const std::array<const char*, 2> phyNames{"voltage0", "voltage1"};
        for (std::size_t i = 0; i < phyNames.size(); ++i) {
            ::iio_channel* phyCh = ::iio_device_find_channel(_phy, phyNames[i], /*output=*/true);
            if (phyCh == nullptr) {
                throw gr::exception(std::format("ad9361-phy {} output channel not found", phyNames[i]));
            }
            const long long hwGainDb = std::clamp<long long>(static_cast<long long>(tx_gain) - 89LL, -89LL, 0LL);
            detail::writeAttrLL(phyCh, "hardwaregain", hwGainDb);
            try {
                detail::writeAttr(phyCh, "rf_port_select", tx_chains[i]);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "IIOSinkMimo: rf_port_select='%s' on %s ignored (firmware default used): %s\n", tx_chains[i].c_str(), phyNames[i], e.what());
            }
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
                break;
            }
        }
        _buf.cancel();
        if (tx_lo_powerdown && isAd9361() && _phy != nullptr) {
            ::iio_channel* txLo = ::iio_device_find_channel(_phy, "altvoltage1", /*output=*/true);
            if (txLo != nullptr) {
                detail::writeAttrLL(txLo, "powerdown", 1LL);
            }
        }
    }
};

} // namespace gr::incubator::iio
