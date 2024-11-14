#pragma once


#ifdef MCUDRV_C28X


#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib/core.h>
#include <emblib/interfaces/uart.h>


namespace mcu {

namespace c28x {

namespace sci {


SCOPED_ENUM_DECLARE_BEGIN(Peripheral) {
    scia,
    scib,
    scic,
    scid
} SCOPED_ENUM_DECLARE_END(Peripheral)


const size_t peripheral_count = 4;


SCOPED_ENUM_UT_DECLARE_BEGIN(Baudrate, uint32_t) {
    baudrate_9600 = 9600,
    baudrate_115200 = 115200,
} SCOPED_ENUM_DECLARE_END(Baudrate)


SCOPED_ENUM_DECLARE_BEGIN(WordLen) {
    word_8bit = SCI_CONFIG_WLEN_8,
    word_7bit = SCI_CONFIG_WLEN_7,
    word_6bit = SCI_CONFIG_WLEN_6,
    word_5bit = SCI_CONFIG_WLEN_5,
    word_4bit = SCI_CONFIG_WLEN_4,
    word_3bit = SCI_CONFIG_WLEN_3,
    word_2bit = SCI_CONFIG_WLEN_2,
    word_1bit = SCI_CONFIG_WLEN_1
} SCOPED_ENUM_DECLARE_END(WordLen)


SCOPED_ENUM_DECLARE_BEGIN(StopBits) {
    one = SCI_CONFIG_STOP_ONE,
    two = SCI_CONFIG_STOP_TWO
} SCOPED_ENUM_DECLARE_END(StopBits)


SCOPED_ENUM_DECLARE_BEGIN(ParityMode) {
    none = SCI_CONFIG_PAR_NONE,
    even = SCI_CONFIG_PAR_EVEN,
    odd = SCI_CONFIG_PAR_ODD
} SCOPED_ENUM_DECLARE_END(ParityMode)


SCOPED_ENUM_DECLARE_BEGIN(AutoBaudMode) {
    disabled,
    enabled
} SCOPED_ENUM_DECLARE_END(AutoBaudMode)


struct Config {
    Baudrate baudrate;
    WordLen word_len;
    StopBits stop_bits;
    ParityMode parity_mode;
    AutoBaudMode autobaud_mode;
};


namespace impl {

struct Module {
    uint32_t base;
    uint32_t pie_rx_int_num;
    uint16_t pie_int_group;
    Module(uint32_t base_, uint32_t pie_rx_int_num_, uint16_t pie_int_group_)
            : base(base_), pie_rx_int_num(pie_rx_int_num_), pie_int_group(pie_int_group_) {}
};


extern const uint32_t sci_bases[4];
extern const uint32_t sci_rx_pie_int_nums[4];
extern const uint16_t sci_pie_int_groups[4];

} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, public emb::uart::Uart, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    impl::Module _module;
public:
    Module(Peripheral peripheral, const gpio::PinConfig& rx_pin, const gpio::PinConfig& tx_pin, const Config& config);
#ifdef CPU1
    static void transfer_control_to_cpu2(Peripheral peripheral, const gpio::PinConfig& rx_pin, const gpio::PinConfig& tx_pin);
#endif
    Peripheral peripheral() const { return _peripheral; }
    uint32_t base() const { return _module.base; }
    virtual void reset() { SCI_performSoftwareReset(_module.base); }
    virtual bool has_rx_error() const {
        return SCI_getRxStatus(_module.base) & SCI_RXSTATUS_ERROR;
    }

    virtual int getchar(char& ch) {
        if (SCI_getRxFIFOStatus(_module.base) != SCI_FIFO_RX0) {
            ch = SCI_readCharNonBlocking(_module.base);
            return 1;
        }
        return 0;
    }

    virtual int recv(char* buf, size_t buf_len) {
        int count = 0;
        char ch = 0;

        while ((count < buf_len) && (getchar(ch) == 1)) {
            buf[count++] = ch;
        }

        if (has_rx_error()) {
            return -1;
        }
        return count;
    }

    virtual int putchar(char ch) {
        if (SCI_getTxFIFOStatus(_module.base) != SCI_FIFO_TX15) {
            SCI_writeCharBlockingFIFO(_module.base, ch);
            return 1;
        }
        return 0;
    }

    virtual int send(const char* buf, size_t len) {
        SCI_writeCharArray(_module.base, reinterpret_cast<const uint16_t*>(buf), len);
        return len;
    }

    virtual void register_rx_interrupt_handler(void (*handler)(void)) {
        SCI_disableModule(_module.base);
        Interrupt_register(_module.pie_rx_int_num, handler);
        SCI_enableInterrupt(_module.base, SCI_INT_RXFF);
        SCI_enableModule(_module.base);
    }

    virtual void enable_rx_interrupts() { Interrupt_enable(_module.pie_rx_int_num); }
    virtual void disable_rx_interrupts() { Interrupt_disable(_module.pie_rx_int_num); }
    virtual void acknowledge_rx_interrupt() {
        SCI_clearInterruptStatus(_module.base, SPI_INT_RXFF);
        Interrupt_clearACKGroup(_module.pie_int_group);
    }
protected:
#ifdef CPU1
    static void _init_pins(const gpio::PinConfig& rx_pin, const gpio::PinConfig& tx_pin);
#endif
};


} // namespace sci

} // namespace c28x

} // namespace mcu


#endif
