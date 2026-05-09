// Host smoke for IIOSinkMimo: defaults, lifecycle robustness,
// channel/chain validation. Live-device coverage gated on FISH_BALL_URI.

#include <complex>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include <boost/ut.hpp>

#include <gnuradio-4.0/iio/IIOSinkMimo.hpp>

using namespace gr::incubator::iio;
using namespace boost::ut;

namespace {

const suite IIOSinkMimoTests = [] {
    "default property values"_test = [] {
        IIOSinkMimo<std::complex<float>> blk;
        expect(blk.uri == "local:");
        expect(blk.device == "cf-ad9361-dds-core-lpc");
        expect(blk.phy_device == "ad9361-phy");
        expect(blk.channels.size() == 4_u);
        expect(blk.tx_chains.size() == 2_u);
        expect(blk.tx_chains[0] == "A");
        expect(blk.tx_chains[1] == "B");
        expect(blk.set_mode_at_init == false);
        expect(blk.center_frequency == 868'100'000.0_d);
        expect(blk.sample_rate == 2'083'334.0_f);
        expect(blk.bandwidth == 200'000.0_d);
        expect(blk.tx_gain == 20.0_d);
        expect(blk.buffer_size == 32'768U);
        expect(blk.timeout_ms == 1'000U);
        expect(blk.tx_tail_pad_samples == 32'768U);
    };

    "settingsChanged is no-op pre-start"_test = [] {
        IIOSinkMimo<std::complex<float>> blk;
        gr::property_map                 empty_old;
        gr::property_map                 new_settings{
            {"center_frequency", 900'000'000.0},
            {"tx_gain", 30.0},
        };
        expect(nothrow([&] { blk.settingsChanged(empty_old, new_settings); }));
    };

    "wrong channels size throws on start"_test = [] {
        IIOSinkMimo<std::complex<float>> blk;
        blk.uri        = "ip:127.0.0.1:1";
        blk.channels   = {"voltage0", "voltage1"};
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "wrong tx_chains size throws on start"_test = [] {
        IIOSinkMimo<std::complex<float>> blk;
        blk.uri        = "ip:127.0.0.1:1";
        blk.tx_chains  = {"A"};
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "bad uri throws on start"_test = [] {
        IIOSinkMimo<std::complex<float>> blk;
        blk.uri        = "ip:127.0.0.1:1";
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "stop on never-started block is safe"_test = [] {
        IIOSinkMimo<std::complex<float>> blk;
        expect(nothrow([&] { blk.stop(); }));
    };

    "FISH_BALL_URI live reinit"_test = [] {
        const char* uri = std::getenv("FISH_BALL_URI");
        if (uri == nullptr) {
            return;
        }
        IIOSinkMimo<std::complex<float>> blk;
        blk.uri         = uri;
        blk.timeout_ms  = 2'000U;
        blk.buffer_size = 4'096U;
        expect(nothrow([&] { blk.start(); }));
        expect(nothrow([&] { blk.stop(); }));
    };
};

} // namespace

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
