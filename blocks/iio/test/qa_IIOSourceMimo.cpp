// Host smoke for IIOSource<T, 2> (unified MIMO variant). Tests defaults,
// lifecycle robustness, validation, and live-device reinit.
//
// Previously tested the separate IIOSourceMimo block; now validates the
// unified IIOSource template with nPorts=2.

#include <complex>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include <boost/ut.hpp>

#include <gnuradio-4.0/iio/IIOSource.hpp>

using namespace gr::incubator::iio;
using namespace boost::ut;

namespace {

const suite IIOSourceMimoUnifiedTests = [] {
    "default property values"_test = [] {
        IIOSource<std::complex<float>, 2> blk;
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
        IIOSource<std::complex<std::int16_t>, 2> blk;
        expect(blk.uri == "local:");
        expect(blk.channels.size() == 4_u);
    };

    "settingsChanged is no-op pre-start"_test = [] {
        IIOSource<std::complex<float>, 2> blk;
        gr::property_map                   empty_old;
        gr::property_map                   new_settings{
            {"center_frequency", 900'000'000.0},
            {"gain", 20.0},
        };
        expect(nothrow([&] { blk.settingsChanged(empty_old, new_settings); }));
    };

    "wrong channels size throws on start"_test = [] {
        IIOSource<std::complex<float>, 2> blk;
        blk.uri        = "ip:127.0.0.1:1";
        blk.channels   = {"voltage0", "voltage1"};
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "wrong rx_chains size throws on start"_test = [] {
        IIOSource<std::complex<float>, 2> blk;
        blk.uri        = "ip:127.0.0.1:1";
        blk.rx_chains  = {"A_BALANCED"};
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "bad uri throws on start"_test = [] {
        IIOSource<std::complex<float>, 2> blk;
        blk.uri        = "ip:127.0.0.1:1";
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "stop on never-started block is safe"_test = [] {
        IIOSource<std::complex<float>, 2> blk;
        expect(nothrow([&] { blk.stop(); }));
    };

    // Live-device smoke: opt-in via FISH_BALL_URI=ip:<host>.
    "FISH_BALL_URI live reinit"_test = [] {
        const char* uri = std::getenv("FISH_BALL_URI");
        if (uri == nullptr) {
            return;
        }
        IIOSource<std::complex<float>, 2> blk;
        blk.uri         = uri;
        blk.timeout_ms  = 2'000U;
        blk.buffer_size = 4'096U;
        expect(nothrow([&] { blk.start(); }));
        expect(nothrow([&] { blk.stop(); }));
    };
};

} // namespace

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
