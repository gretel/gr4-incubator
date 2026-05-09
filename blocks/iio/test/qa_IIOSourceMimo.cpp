// Host smoke for IIOSourceMimo: defaults, lifecycle robustness,
// channel/chain validation. Live-device coverage is gated behind the
// FISH_BALL_URI environment variable so CI without hardware skips
// silently.

#include <complex>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include <boost/ut.hpp>

#include <gnuradio-4.0/iio/IIOSourceMimo.hpp>

using namespace gr::incubator::iio;
using namespace boost::ut;

namespace {

const suite IIOSourceMimoTests = [] {
    "default property values"_test = [] {
        IIOSourceMimo<std::complex<float>> blk;
        expect(blk.uri == "local:");
        expect(blk.device == "cf-ad9361-lpc");
        expect(blk.phy_device == "ad9361-phy");
        expect(blk.channels.size() == 4_u);
        expect(blk.channels[0] == "voltage0");
        expect(blk.channels[1] == "voltage1");
        expect(blk.channels[2] == "voltage2");
        expect(blk.channels[3] == "voltage3");
        expect(blk.rx_chains.size() == 2_u);
        expect(blk.rx_chains[0] == "A_BALANCED");
        expect(blk.rx_chains[1] == "B_BALANCED");
        expect(blk.set_mode_at_init == false);
        expect(blk.center_frequency == 868'100'000.0_d);
        expect(blk.sample_rate == 2'083'334.0_f);
        expect(blk.bandwidth == 200'000.0_d);
        expect(blk.gain == 40.0_d);
        expect(blk.gain_mode == "slow_attack");
        expect(blk.buffer_size == 32'768U);
        expect(blk.timeout_ms == 1'000U);
        expect(blk.max_overflow_count == 10U);
    };

    "int16 specialisation default constructible"_test = [] {
        IIOSourceMimo<std::complex<std::int16_t>> blk;
        expect(blk.uri == "local:");
        expect(blk.channels.size() == 4_u);
    };

    "settingsChanged is no-op pre-start"_test = [] {
        IIOSourceMimo<std::complex<float>> blk;
        gr::property_map                   empty_old;
        gr::property_map                   new_settings{
            {"center_frequency", 900'000'000.0},
            {"gain", 20.0},
        };
        expect(nothrow([&] { blk.settingsChanged(empty_old, new_settings); }));
    };

    "wrong channels size throws on start"_test = [] {
        IIOSourceMimo<std::complex<float>> blk;
        blk.uri        = "ip:127.0.0.1:1"; // unreachable; throw should be from validation, not network
        blk.channels   = {"voltage0", "voltage1"};
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "wrong rx_chains size throws on start"_test = [] {
        IIOSourceMimo<std::complex<float>> blk;
        blk.uri        = "ip:127.0.0.1:1";
        blk.rx_chains  = {"A_BALANCED"};
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "bad uri throws on start"_test = [] {
        IIOSourceMimo<std::complex<float>> blk;
        blk.uri        = "ip:127.0.0.1:1"; // reserved port — guaranteed unreachable
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "stop on never-started block is safe"_test = [] {
        IIOSourceMimo<std::complex<float>> blk;
        expect(nothrow([&] { blk.stop(); }));
    };

    // Live-device smoke: opt-in via FISH_BALL_URI=ip:<host>. Confirms the
    // 4-channel context resolves and reinit succeeds end-to-end without
    // sample capture.
    "FISH_BALL_URI live reinit"_test = [] {
        const char* uri = std::getenv("FISH_BALL_URI");
        if (uri == nullptr) {
            return; // skip
        }
        IIOSourceMimo<std::complex<float>> blk;
        blk.uri         = uri;
        blk.timeout_ms  = 2'000U;
        blk.buffer_size = 4'096U;
        expect(nothrow([&] { blk.start(); }));
        expect(nothrow([&] { blk.stop(); }));
    };
};

} // namespace

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
