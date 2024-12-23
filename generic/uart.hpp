#pragma once

#include <emblib/core.hpp>

namespace mcu {
namespace uart {

#if defined(MCUDRV_C28X)

class tty {
public:
    tty() EMB_DEFAULT
    virtual ~tty() EMB_DEFAULT

    virtual int getchar() = 0;
    virtual int putchar(int ch) = 0;
};

class module : protected emb::noncopyable {
public:
    module() {}
    virtual ~module() {}

    virtual void reset() = 0;
    virtual bool has_rx_error() const = 0;

    virtual int getchar(char& ch) = 0;
    virtual int recv(char* buf, size_t buf_len) = 0;

    virtual int putchar(char ch) = 0;
    virtual int send(const char* buf, size_t len) = 0;

    virtual void register_rx_interrupt_handler(void (*handler)(void)) = 0;
    virtual void enable_rx_interrupts() = 0;
    virtual void disable_rx_interrupts() = 0;
    virtual void acknowledge_rx_interrupt() = 0;
};

#else

class module : protected emb::noncopyable {
public:
    Uart() = default;
    virtual ~Uart() = default;

    Uart(const Uart& other) = delete;
    Uart& operator=(const Uart& other) = delete;

    //virtual void reset() = 0;
    //virtual bool hasRxError() const = 0;

    virtual int getchar(char& ch) = 0;
    virtual int recv(char* buf, size_t len) = 0;

    virtual int putchar(char ch) = 0;
    virtual int send(const char* buf, size_t len) = 0;

    //virtual void registerRxInterruptHandler(void (*handler)(void)) = 0;
    //virtual void enableRxInterrupts() = 0;
    //virtual void disableRxInterrupts() = 0;
    //virtual void acknowledgeRxInterrupt() = 0;
};

#endif

} // namespace uart
} // namespace mcu
