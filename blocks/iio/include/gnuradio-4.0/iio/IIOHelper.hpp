// SPDX-License-Identifier: LGPL-2.1-or-later
//
// Convenience helpers for creating IIO source/sink blocks with Pluto/AD9361
// default settings.  Reduces boilerplate when wiring a PlutoSDR (or compatible)
// into a gnuradio4 graph.
//
// Usage:
//
//   auto& source = graph.emplaceBlock<gr::incubator::iio::IIOSource<cf32>>(
//       gr::incubator::iio::plutoSourceProps(uri, sample_rate, freq, bw, gain,
//                                            gain_mode, rf_port));

#pragma once

#include <gnuradio-4.0/iio/IIOSource.hpp>
#include <gnuradio-4.0/iio/IIOSink.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

namespace gr::incubator::iio {

/// Normalise a raw URI (strip "uri=" prefix if present).
inline std::string normaliseUri(const std::string& raw) {
    if (raw.starts_with("uri=")) {
        return raw.substr(4);
    }
    return raw;
}

/// Return a property_map with Pluto/AD9361 source defaults pre-filled.
///
/// The returned map is suitable for passing to emplaceBlock<IIOSource<T>>.
/// Fields set here reflect FISH Ball / Nano PlutoSDR best practices:
/// DC blocker enabled, overflow recovery on, blocking I/O, 32768-sample
/// buffers.  Pass additional overrides via extraOverrides to replace any
/// default.
///
/// @param uri        IIO context URI ("local:", "ip:1.2.3.4", etc.)
/// @param sampleRate Sample rate in Hz (e.g. 2.5e6)
/// @param freq       Centre frequency in Hz
/// @param bandwidth  AD9361 analog LPF bandwidth in Hz (set to channel
///                   bandwidth, not sample rate, to prevent aliasing on
///                   decimated paths; for scan/sweep paths set equal to
///                   l1_rate)
/// @param gain       RX gain in dB (0-71 for AD9361 manual, or AGC target)
/// @param gainMode   "manual" or "slow_attack" (AGC)
/// @param rfPort     "A_BALANCED", "B_BALANCED", "A_N", "B_N" etc.
/// @param extraOverrides  Optional additional property overrides (e.g.
///                        {{"non_blocking", true}})
inline gr::property_map plutoSourceProps(
    const std::string& uri,
    float sampleRate,
    double freq,
    double bandwidth,
    double gain,
    std::string gainMode     = "manual",
    std::string rfPort       = "A_BALANCED",
    gr::property_map extraOverrides = {}) {
    // Base property map with Pluto-validated defaults:
    //   - DC blocker always on (Pluto ignores LO offset, DC spur at bin 0)
    //   - overflow recovery on (IIO buffer can stall on Pluto)
    //   - blocking I/O (non_blocking=false) gives full buffers; safe for
    //     both local and remote IIO (setBlockingMode() returns ENOSYS on
    //     TCP backends and is silently ignored)
    //   - 32768-sample buffer (matches Pluto IIO buffer depth)
    //   - 1-second timeout (generous for TCP jitter)
    //   - max 10 overflow events before warning
    //   - IIOSource debug OFF (stdout spam causes double-free on armv7)
    gr::property_map props{
        {"uri", uri},
        {"sample_rate", sampleRate},
        {"center_frequency", freq},
        {"bandwidth", bandwidth},
        {"gain", gain},
        {"gain_mode", std::move(gainMode)},
        {"rf_port", std::move(rfPort)},
        {"buffer_size", gr::Size_t{32768U}},
        {"timeout_ms", gr::Size_t{1000U}},
        {"non_blocking", false},
        {"max_overflow_count", gr::Size_t{10U}},
        {"debug", false},
        {"dc_blocker_enabled", true},
        {"dc_blocker_cutoff", 2000.f},
        {"overflow_recovery", true},
    };
    // Apply overrides (last-write wins within a flat property_map merge).
    for (auto& [k, v] : extraOverrides) {
        props[k] = std::move(v);
    }
    return props;
}

/// Convenience wrapper: emplace an IIOSource<T> with Pluto-validated defaults
/// in a single call.
template<typename T, typename Graph>
decltype(auto) emplacePlutoSource(Graph& graph,
                                  const std::string& uri,
                                  float sampleRate,
                                  double freq,
                                  double bandwidth,
                                  double gain,
                                  std::string gainMode     = "manual",
                                  std::string rfPort       = "A_BALANCED",
                                  gr::property_map extra   = {}) {
    const auto props = plutoSourceProps(
        normaliseUri(uri), sampleRate, freq, bandwidth,
        gain, std::move(gainMode), std::move(rfPort), std::move(extra));
    return graph.template emplaceBlock<IIOSource<T>>(props);
}

} // namespace gr::incubator::iio
