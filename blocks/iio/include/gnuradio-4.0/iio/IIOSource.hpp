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
#include <array>
#include <atomic>
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
    using Base::Base;

    IIOSource() = default;

    PortOut<T> out;

    // Connection
    std::string uri        = "local:";
    std::string device     = "cf-ad9361-lpc";
    std::string phy_device = "ad9361-phy";

    // I/Q channel names (size 2 for one I/Q pair). Must be in order [I, Q].
    std::vector<std::string> channels = {"voltage0", "voltage1"};

    // Generic raw IIO attributes. Key: "<channel>/<attr>" or just "<attr>".
    property_map attributes;

    // AD9361 convenience aliases (only honoured when phy_device=="ad9361-phy")
    double      center_frequency = 868'100'000.0;
    float       sample_rate      = 2'083'334.0f;
    double      bandwidth        = 200'000.0;
    double      gain             = 40.0;
    std::string gain_mode        = "slow_attack";
    std::string rf_port          = "A_BALANCED";

    // DMA + timing
    gr::Size_t buffer_size        = 32'768U;
    gr::Size_t timeout_ms         = 1'000U;
    gr::Size_t max_overflow_count = 10U;
    bool       non_blocking       = false;  ///< true = poll mode; false = blocking refill
    bool       debug              = false;  ///< log per-refill sample count

    GR_MAKE_REFLECTABLE(IIOSource, out, uri, device, phy_device, channels, attributes,
        center_frequency, sample_rate, bandwidth, gain, gain_mode, rf_port,
        buffer_size, timeout_ms, max_overflow_count, non_blocking, debug);

    // ---------- lifecycle ---------------------------------------------------

    void start() { reinitDevice(); }

    void stop() noexcept {
        try { _buf.cancel(); _buf.reset(); _ctx.reset(); }
        catch (const std::exception& e) { std::fprintf(stderr, "IIOSource::stop: %s\n", e.what()); }
        catch (...) { std::fprintf(stderr, "IIOSource::stop: unknown exception\n"); }
    }

    void settingsChanged(const property_map& /*old_*/, const property_map& new_) {
        if (!_ctx) return;
        const bool needsFullReinit = new_.contains("uri") || new_.contains("device")
            || new_.contains("phy_device") || new_.contains("channels")
            || new_.contains("buffer_size") || new_.contains("non_blocking");
        if (needsFullReinit) { reinitDevice(); return; }
        if (new_.contains("timeout_ms"))
            _ctx.setTimeout(static_cast<unsigned int>(timeout_ms));
        if (isAd9361()) {
            if (new_.contains("center_frequency")) applyAd9361CenterFrequency();
            if (new_.contains("sample_rate")) applyAd9361SampleRate();
            if (new_.contains("bandwidth")) applyAd9361Bandwidth();
            if (new_.contains("gain") || new_.contains("gain_mode") || new_.contains("rf_port"))
                applyAd9361PerChannel();
        }
        if (new_.contains("attributes")) applyAttributes(/*isOutput=*/false);
    }

    // ---------- processBulk -------------------------------------------------

    [[nodiscard]] gr::work::Status processBulk(OutputSpanLike auto& output) {
        if (!_buf) { output.publish(0U); return gr::work::Status::INSUFFICIENT_INPUT_ITEMS; }

        // 1. Refill from IIO hardware — append to ring buffer
        const ssize_t bytes = _buf.refill();
        if (bytes < 0) {
            const int err = -static_cast<int>(bytes);
            if (err == ETIMEDOUT) {
                // Timed-out refill may still have partial data. If ring has
                // samples, drain them first; otherwise return OK and try again.
                if (_ringTail == _ringHead) {
                    output.publish(0U);
                    this->progress->incrementAndGet();
                    this->progress->notify_all();
                    return gr::work::Status::OK;
                }
            } else {
                output.publish(0U);
                this->progress->incrementAndGet();
                this->progress->notify_all();
                if (err == EBADF) return gr::work::Status::DONE;
                bumpOverflow(err);
                return gr::work::Status::OK;
            }
        } else if (bytes > 0) {
            const std::ptrdiff_t step = _buf.step();
            if (step > 0 && _chans[0] != nullptr) {
                const auto* const start = static_cast<const std::byte*>(_buf.first(_chans[0]));
                const auto* const end   = static_cast<const std::byte*>(_buf.end());
                const std::size_t scans = static_cast<std::size_t>((end - start) / step);
                if (debug) {
                    std::fprintf(stderr, "IIOSource: refill bytes=%zd step=%td scans=%zu\n", bytes, step, scans);
                }
                appendRefill(start, step, scans);
            }
        }

        // 2. Drain ring buffer into output port
        const std::size_t available = _ringTail - _ringHead;
        if (available == 0U) {
            output.publish(0U);
            this->progress->incrementAndGet();
            this->progress->notify_all();
            return gr::work::Status::OK;
        }
        const std::size_t n = std::min(available, output.size());
        for (std::size_t i = 0U; i < n; ++i) {
            output[i] = _ring[_ringHead % _ring.size()];
            ++_ringHead;
        }
        output.publish(n);
        compactRing();
        this->progress->incrementAndGet();
        this->progress->notify_all();
        return gr::work::Status::OK;
    }

    // Overflow counter — exposed as atomic for telemetry (SoapySource
    // convention). lora_trx reads it via overflow_ptr in build_rx_graph().
    std::atomic<gr::Size_t> overflowCount{};

    // ---------- ring buffer: accumulate across refills for continuous output ---
    //
    // Local IIO (uri=local:) returns a fresh kernel buffer per refill. Between
    // refills there can be boundaries that drop/duplicate samples, corrupting
    // the stride decimation. The ring buffer stitches refills into a continuous
    // stream so downstream blocks see no discontinuities.
    //
    // Remote IIO (uri=ip:...) doesn't need this — iiod concatenates kernel
    // buffers into the TCP stream — but the ring buffer is harmless there too.

    void appendRefill(const std::byte* start, std::ptrdiff_t step, std::size_t scans) {
        ensureRingCapacity(scans);
        for (std::size_t i = 0U; i < scans; ++i) {
            const std::byte* p = start + static_cast<std::ptrdiff_t>(i) * step;
            std::int16_t i_raw{}, q_raw{};
            std::memcpy(&i_raw, p, 2);
            std::memcpy(&q_raw, p + 2, 2);
            if constexpr (std::is_same_v<T, std::complex<std::int16_t>>) {
                _ring[_ringTail % _ring.size()] = std::complex<std::int16_t>(i_raw, q_raw);
            } else {
                constexpr float scale = 1.0f / 2048.0f;
                _ring[_ringTail % _ring.size()] = std::complex<float>(
                    static_cast<float>(i_raw) * scale,
                    static_cast<float>(q_raw) * scale);
            }
            ++_ringTail;
        }
    }

    void ensureRingCapacity(std::size_t incoming) {
        if (_ring.empty()) {
            // 8× buffer_size gives plenty of margin for scheduling jitter
            _ring.resize(static_cast<std::size_t>(buffer_size) * 8U);
        }
        const std::size_t capacity = _ring.size();
        const std::size_t used = _ringTail - _ringHead;
        if (used + incoming > capacity) {
            // Ran out of space — grow (shouldn't happen in practice)
            _ring.resize(static_cast<std::size_t>(static_cast<double>(capacity) * 1.5 + incoming));
        }
    }

    void compactRing() {
        const std::size_t capacity = _ring.size();
        const std::size_t used = _ringTail - _ringHead;
        // Compact when read cursor is past the halfway point and used is small
        if (used < capacity / 4U && _ringHead > capacity / 2U) {
            const std::size_t headMod = _ringHead % capacity;
            if (headMod + used <= capacity) {
                // Simple case: contiguous elements, single memmove
                std::memmove(_ring.data(), &_ring[headMod], used * sizeof(T));
            } else {
                // Wrapped case: two segments
                const std::size_t first = capacity - headMod;
                std::memmove(_ring.data(), &_ring[headMod], first * sizeof(T));
                std::memmove(&_ring[first], _ring.data(), (used - first) * sizeof(T));
            }
            _ringHead = 0U;
            _ringTail = used;
        }
    }

private:
    // ---------- device init helpers ----------------------------------------

    detail::Context               _ctx;
    ::iio_device*                 _phy       = nullptr;
    ::iio_device*                 _streamDev = nullptr;
    std::array<::iio_channel*, 2> _chans{};
    detail::Buffer                _buf;
    std::vector<T>                _ring{};
    std::size_t                   _ringHead = 0U;   ///< read cursor (absolute sample index)
    std::size_t                   _ringTail = 0U;   ///< write cursor (absolute sample index)
    std::size_t                   _overflowCount      = 0;
    std::size_t                   _totalOverflowCount = 0;

    [[nodiscard]] bool isAd9361() const noexcept { return phy_device == "ad9361-phy"; }

    void reinitDevice() {
        _buf.cancel(); _buf.reset(); _ctx.reset();
        _phy = nullptr; _streamDev = nullptr; _chans = {};
        _ctx = detail::Context(uri);
        _ctx.setTimeout(static_cast<unsigned int>(timeout_ms));
        if (!phy_device.empty()) _phy = _ctx.findDevice(phy_device);
        _streamDev = _ctx.findDevice(device);
        if (channels.size() != 2)
            throw gr::exception(std::format("IIOSource: channels.size()={} but 2 required", channels.size()));
        for (std::size_t i = 0; i < channels.size(); ++i) {
            ::iio_channel* ch = ::iio_device_find_channel(_streamDev, channels[i].c_str(), /*output=*/false);
            if (ch == nullptr)
                throw gr::exception(std::format("iio_device_find_channel('{}') returned null", channels[i]));
            detail::enableChannel(ch);
            _chans[i] = ch;
        }
        if (isAd9361()) {
            applyAd9361CenterFrequency();
            applyAd9361SampleRate();
            applyAd9361Bandwidth();
            applyAd9361PerChannel();
        }
        applyAttributes(/*isOutput=*/false);
        _buf = detail::Buffer(_streamDev, static_cast<std::size_t>(buffer_size), /*cyclic=*/false);
        _buf.setBlockingMode(!non_blocking);
    }

    void applyAd9361CenterFrequency() {
        if (_phy == nullptr) return;
        detail::writeAttrLL(::iio_device_find_channel(_phy, "altvoltage0", /*output=*/true),
            "frequency", static_cast<long long>(center_frequency));
    }

    void applyAd9361SampleRate() {
        if (_phy == nullptr) return;
        detail::writeAttrLL(::iio_device_find_channel(_phy, "voltage0", /*output=*/false),
            "sampling_frequency", static_cast<long long>(sample_rate));
    }

    void applyAd9361Bandwidth() {
        if (_phy == nullptr) return;
        detail::writeAttrLL(::iio_device_find_channel(_phy, "voltage0", /*output=*/false),
            "rf_bandwidth", static_cast<long long>(bandwidth));
    }

    void applyAd9361PerChannel() {
        if (_phy == nullptr) return;
        ::iio_channel* phyCh = ::iio_device_find_channel(_phy, "voltage0", /*output=*/false);
        if (phyCh == nullptr) throw gr::exception("ad9361-phy voltage0 input channel not found");
        detail::writeAttr(phyCh, "gain_control_mode", gain_mode);
        if (gain_mode == "manual")
            detail::writeAttrLL(phyCh, "hardwaregain", static_cast<long long>(gain));
        if (::iio_channel_attr_write(phyCh, "rf_port_select", rf_port.c_str()) < 0)
            std::fprintf(stderr, "IIOSource: rf_port_select='%s' on voltage0 ignored (firmware default used)\n", rf_port.c_str());
    }

    void applyAttributes(bool isOutput) {
        if (attributes.empty()) return;
        ::iio_device* dev = _phy ? _phy : _streamDev;
        for (const auto& [key, value] : attributes) {
            const auto slash = key.find('/');
            std::string attrName;
            ::iio_channel* ch = nullptr;
            if (slash != decltype(key)::npos) {
                const std::string chName(key.begin(), key.begin() + static_cast<long>(slash));
                attrName = std::string(key.begin() + static_cast<long>(slash) + 1, key.end());
                ch = ::iio_device_find_channel(dev, chName.c_str(), isOutput);
                if (ch == nullptr && _streamDev != _phy && dev == _phy)
                    ch = ::iio_device_find_channel(_streamDev, chName.c_str(), isOutput);
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
        ++_overflowCount; ++_totalOverflowCount;
        overflowCount.fetch_add(1U, std::memory_order_relaxed);
        if (_overflowCount == 1U || _overflowCount == 10U || _overflowCount == 100U || _overflowCount == 1000U)
            std::fprintf(stderr, "IIOSource: refill error errno=%d (count=%zu, total=%zu)\n", err, _overflowCount, _totalOverflowCount);
        if (max_overflow_count != 0U && _overflowCount > max_overflow_count)
            throw gr::exception(std::format("IIOSource: refill overflow threshold exceeded (count=%zu, max=%zu, last errno=%d)", _overflowCount, max_overflow_count, err));
    }
};

} // namespace gr::incubator::iio
