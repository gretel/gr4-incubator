// Internal-use RAII helpers around libiio v0.26 primitives. Not part of the
// public block API. Insulates IIOSource / IIOSink from raw libiio C calls
// and centralises error-to-gr::exception translation.
//
// libiio v0.26 reference: https://github.com/analogdevicesinc/libiio/tree/v0.26

#pragma once

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <format>
#include <memory>
#include <string>
#include <string_view>
#include <sys/types.h> // ssize_t

#include <iio.h>

#include <gnuradio-4.0/Block.hpp>

namespace gr::incubator::iio::detail {

// -- error translation -------------------------------------------------------

[[noreturn]] inline void throwIIO(std::string_view what, int errnoVal) { throw gr::exception(std::format("{}: {} (errno {})", what, std::strerror(errnoVal), errnoVal)); }

// -- Context -----------------------------------------------------------------

class Context {
public:
    Context() = default;

    explicit Context(std::string_view uri) {
        const std::string uriStr(uri);
        ::iio_context*    raw = ::iio_create_context_from_uri(uriStr.c_str());
        if (raw == nullptr) {
            throwIIO(std::format("iio_create_context_from_uri('{}')", uriStr), errno);
        }
        _ctx.reset(raw);
    }

    void reset() noexcept { _ctx.reset(); }

    [[nodiscard]] ::iio_context* get() const noexcept { return _ctx.get(); }
    [[nodiscard]] explicit       operator bool() const noexcept { return _ctx != nullptr; }

    void setTimeout(unsigned int timeoutMs) {
        if (_ctx == nullptr) {
            throw gr::exception("Context::setTimeout on null context");
        }
        if (const int rc = ::iio_context_set_timeout(_ctx.get(), timeoutMs); rc != 0) {
            throwIIO("iio_context_set_timeout", -rc);
        }
    }

    [[nodiscard]] ::iio_device* findDevice(std::string_view name) const {
        if (_ctx == nullptr) {
            throw gr::exception("Context::findDevice on null context");
        }
        const std::string nameStr(name);
        ::iio_device*     dev = ::iio_context_find_device(_ctx.get(), nameStr.c_str());
        if (dev == nullptr) {
            throw gr::exception(std::format("iio_context_find_device('{}') returned null", nameStr));
        }
        return dev;
    }

private:
    struct Deleter {
        void operator()(::iio_context* ctx) const noexcept {
            if (ctx != nullptr) {
                ::iio_context_destroy(ctx);
            }
        }
    };
    std::unique_ptr<::iio_context, Deleter> _ctx;
};

// -- Buffer ------------------------------------------------------------------

class Buffer {
public:
    Buffer() = default;

    Buffer(::iio_device* dev, std::size_t samplesCount, bool cyclic) {
        if (dev == nullptr) {
            throw gr::exception("Buffer ctor: device is null");
        }
        ::iio_buffer* raw = ::iio_device_create_buffer(dev, samplesCount, cyclic);
        if (raw == nullptr) {
            throwIIO("iio_device_create_buffer", errno);
        }
        _buf.reset(raw);
    }

    void reset() noexcept { _buf.reset(); }

    void cancel() noexcept {
        if (_buf != nullptr) {
            ::iio_buffer_cancel(_buf.get());
        }
    }

    void setBlockingMode(bool blocking) {
        if (_buf == nullptr) {
            throw gr::exception("Buffer::setBlockingMode on null buffer");
        }
        const int rc = ::iio_buffer_set_blocking_mode(_buf.get(), blocking);
        if (rc == 0 || rc == -ENOSYS) {
            // ENOSYS: TCP/IIOD backend does not implement blocking-mode toggle
            // (refill is always blocking on remote backends). Local kernel
            // buffer accepts it. Either is fine.
            return;
        }
        throwIIO("iio_buffer_set_blocking_mode", -rc);
    }

    // Negative return = -errno (-EBADF after cancel(), -ETIMEDOUT on timeout).
    // Caller dispatches based on sign; no exception on transport-level errors.
    [[nodiscard]] ssize_t refill() {
        if (_buf == nullptr) {
            throw gr::exception("Buffer::refill on null buffer");
        }
        return ::iio_buffer_refill(_buf.get());
    }

    [[nodiscard]] ssize_t push() {
        if (_buf == nullptr) {
            throw gr::exception("Buffer::push on null buffer");
        }
        return ::iio_buffer_push(_buf.get());
    }

    [[nodiscard]] void*          first(::iio_channel* ch) const noexcept { return _buf == nullptr ? nullptr : ::iio_buffer_first(_buf.get(), ch); }
    [[nodiscard]] void*          end() const noexcept { return _buf == nullptr ? nullptr : ::iio_buffer_end(_buf.get()); }
    [[nodiscard]] std::ptrdiff_t step() const noexcept { return _buf == nullptr ? std::ptrdiff_t{0} : ::iio_buffer_step(_buf.get()); }

    [[nodiscard]] ::iio_buffer* get() const noexcept { return _buf.get(); }
    [[nodiscard]] explicit      operator bool() const noexcept { return _buf != nullptr; }

private:
    struct Deleter {
        void operator()(::iio_buffer* buf) const noexcept {
            if (buf != nullptr) {
                ::iio_buffer_destroy(buf);
            }
        }
    };
    std::unique_ptr<::iio_buffer, Deleter> _buf;
};

// -- Channel enable / disable ------------------------------------------------

inline void enableChannel(::iio_channel* ch) {
    if (ch == nullptr) {
        throw gr::exception("enableChannel: channel is null");
    }
    ::iio_channel_enable(ch);
}

inline void disableChannel(::iio_channel* ch) {
    if (ch == nullptr) {
        throw gr::exception("disableChannel: channel is null");
    }
    ::iio_channel_disable(ch);
}

// -- Channel attribute helpers ----------------------------------------------

inline void writeAttr(::iio_channel* ch, std::string_view name, std::string_view value) {
    if (ch == nullptr) {
        throw gr::exception(std::format("writeAttr('{}'): channel is null", name));
    }
    const std::string n(name);
    const std::string v(value);
    const ssize_t     rc = ::iio_channel_attr_write(ch, n.c_str(), v.c_str());
    if (rc < 0) {
        throwIIO(std::format("iio_channel_attr_write('{}', '{}')", n, v), static_cast<int>(-rc));
    }
}

inline void writeAttrLL(::iio_channel* ch, std::string_view name, long long value) {
    if (ch == nullptr) {
        throw gr::exception(std::format("writeAttrLL('{}'): channel is null", name));
    }
    const std::string n(name);
    const int         rc = ::iio_channel_attr_write_longlong(ch, n.c_str(), value);
    if (rc != 0) {
        throwIIO(std::format("iio_channel_attr_write_longlong('{}', {})", n, value), -rc);
    }
}

[[nodiscard]] inline long long readAttrLL(::iio_channel* ch, std::string_view name) {
    if (ch == nullptr) {
        throw gr::exception(std::format("readAttrLL('{}'): channel is null", name));
    }
    const std::string n(name);
    long long         out = 0;
    const int         rc  = ::iio_channel_attr_read_longlong(ch, n.c_str(), &out);
    if (rc != 0) {
        throwIIO(std::format("iio_channel_attr_read_longlong('{}')", n), -rc);
    }
    return out;
}

// -- Device attribute helpers ------------------------------------------------

inline void writeAttr(::iio_device* dev, std::string_view name, std::string_view value) {
    if (dev == nullptr) {
        throw gr::exception(std::format("writeAttr('{}'): device is null", name));
    }
    const std::string n(name);
    const std::string v(value);
    const ssize_t     rc = ::iio_device_attr_write(dev, n.c_str(), v.c_str());
    if (rc < 0) {
        throwIIO(std::format("iio_device_attr_write('{}', '{}')", n, v), static_cast<int>(-rc));
    }
}

inline void writeAttrLL(::iio_device* dev, std::string_view name, long long value) {
    if (dev == nullptr) {
        throw gr::exception(std::format("writeAttrLL('{}'): device is null", name));
    }
    const std::string n(name);
    const int         rc = ::iio_device_attr_write_longlong(dev, n.c_str(), value);
    if (rc != 0) {
        throwIIO(std::format("iio_device_attr_write_longlong('{}', {})", n, value), -rc);
    }
}

[[nodiscard]] inline long long readAttrLL(::iio_device* dev, std::string_view name) {
    if (dev == nullptr) {
        throw gr::exception(std::format("readAttrLL('{}'): device is null", name));
    }
    const std::string n(name);
    long long         out = 0;
    const int         rc  = ::iio_device_attr_read_longlong(dev, n.c_str(), &out);
    if (rc != 0) {
        throwIIO(std::format("iio_device_attr_read_longlong('{}')", n), -rc);
    }
    return out;
}

} // namespace gr::incubator::iio::detail
