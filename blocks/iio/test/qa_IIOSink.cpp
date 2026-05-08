// Host smoke for IIOSink: defaults, property-map construction,
// pre-start lifecycle robustness, bad-URI failure mode.

#include <complex>
#include <cstdint>
#include <string>

#include <boost/ut.hpp>

#include <gnuradio-4.0/iio/IIOSink.hpp>

using namespace gr::incubator::iio;
using namespace boost::ut;

namespace {

const suite IIOSinkTests = [] {
    "default property values"_test = [] {
        IIOSink<std::complex<float>> blk;
        expect(blk.uri == "local:");
        expect(blk.device == "cf-ad9361-dds-core-lpc");
        expect(blk.phy_device == "ad9361-phy");
        expect(blk.channels.size() == 2_u);
        expect(blk.channels[0] == "voltage0");
        expect(blk.channels[1] == "voltage1");
        expect(blk.attributes.empty());
        expect(blk.center_frequency == 868'100'000.0_d);
        expect(blk.sample_rate == 2'083'334.0_f);
        expect(blk.bandwidth == 200'000.0_d);
        expect(blk.tx_attenuation == 10.0_d);
        expect(blk.rf_port == "A");
        expect(blk.buffer_size == 32'768U);
        expect(blk.timeout_ms == 1'000U);
        expect(blk.tx_tail_pad_samples == 32'768U);
    };

    "int16 specialisation default constructible"_test = [] {
        IIOSink<std::complex<std::int16_t>> blk;
        expect(blk.uri == "local:");
        expect(blk.device == "cf-ad9361-dds-core-lpc");
    };

    // Constructing via property_map stages settings inside GR4's settings
    // machinery; reflectable members only update after applyStagedParameters
    // (called by start() in normal flow). See qa_IIOSource for the same note.

    "settingsChanged is no-op pre-start"_test = [] {
        IIOSink<std::complex<float>> blk;
        gr::property_map             empty_old;
        gr::property_map             new_settings{
            {"center_frequency", 900'000'000.0},
            {"tx_attenuation", 25.0},
        };
        expect(nothrow([&] { blk.settingsChanged(empty_old, new_settings); }));
    };

    "bad uri throws on start"_test = [] {
        IIOSink<std::complex<float>> blk;
        blk.uri        = "ip:127.0.0.1:1";
        blk.timeout_ms = 100U;
        expect(throws([&] { blk.start(); }));
    };

    "stop on never-started block is safe"_test = [] {
        IIOSink<std::complex<float>> blk;
        expect(nothrow([&] { blk.stop(); }));
    };
};

} // namespace

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
