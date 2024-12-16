#pragma once


#ifdef MCUDRV_C28X


#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib/core.h>
#include <emblib/optional.h>


namespace mcu {

namespace c28x {

namespace spi {


SCOPED_ENUM_DECLARE_BEGIN(Peripheral) {
    spia,
    spib,
    spic
} SCOPED_ENUM_DECLARE_END(Peripheral)


const size_t peripheral_count = 3;


SCOPED_ENUM_DECLARE_BEGIN(Protocol) {
    pol0_pha0 = SPI_PROT_POL0PHA0,  // Mode 0. Polarity 0, phase 0. Rising edge without delay
    pol0_pha1 = SPI_PROT_POL0PHA1,  // Mode 1. Polarity 0, phase 1. Rising edge with delay.
    pol1_pha0 = SPI_PROT_POL1PHA0,  // Mode 2. Polarity 1, phase 0. Falling edge without delay.
    pol1_pha1 = SPI_PROT_POL1PHA1   // Mode 3. Polarity 1, phase 1. Falling edge with delay.
} SCOPED_ENUM_DECLARE_END(Protocol)


SCOPED_ENUM_DECLARE_BEGIN(Mode) {
    slave = SPI_MODE_SLAVE,             // SPI slave
    master = SPI_MODE_MASTER,           // SPI master
    slave_notalk = SPI_MODE_SLAVE_OD,   // SPI slave w/ output (TALK) disabled
    master_notalk = SPI_MODE_MASTER_OD  // SPI master w/ output (TALK) disabled
} SCOPED_ENUM_DECLARE_END(Mode)


SCOPED_ENUM_UT_DECLARE_BEGIN(Bitrate, uint32_t) {
    bitrate_1m = 1000000,
    bitrate_12m5 = 12500000,
} SCOPED_ENUM_DECLARE_END(Bitrate)


SCOPED_ENUM_DECLARE_BEGIN(WordLen) {
    word_8bit = 8,
    word_16bit = 16
} SCOPED_ENUM_DECLARE_END(WordLen)


struct MosiPinConfig { uint32_t pin; uint32_t mux; };
struct MisoPinConfig { uint32_t pin; uint32_t mux; };
struct ClkPinConfig { uint32_t pin; uint32_t mux; };
struct CsPinConfig { uint32_t pin; uint32_t mux; };


struct Config {
    Protocol protocol;
    Mode mode;
    Bitrate bitrate;
    WordLen word_len;
    uint16_t data_size;
};


namespace impl {


struct Module {
    uint32_t base;
    uint32_t pie_rx_int_num;
    Module(uint32_t base_, uint32_t pie_rx_int_num_)
            : base(base_), pie_rx_int_num(pie_rx_int_num_) {}
};


extern const uint32_t spi_bases[3];
extern const uint32_t spi_rx_pie_int_nums[3];


} // namespace impl


class Module : public emb::singleton_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    impl::Module _module;
    WordLen _word_len;
public:
    Module(Peripheral peripheral,
            const MosiPinConfig& mosi_pin, const MisoPinConfig& miso_pin,
            const ClkPinConfig& clk_pin, emb::optional<CsPinConfig> cs_pin,
            const Config& config);
#ifdef CPU1
    static void transfer_control_to_cpu2(Peripheral peripheral,
                                         const MosiPinConfig& mosi_pin, const MisoPinConfig& miso_pin,
                                         const ClkPinConfig& clk_pin, emb::optional<CsPinConfig> cs_pin);
#endif
    Peripheral peripheral() const { return _peripheral; }
    uint32_t base() const { return _module.base; }
    void enable_loopback() {
        SPI_disableModule(_module.base);
        SPI_enableLoopback(_module.base);
        SPI_enableModule(_module.base);
    }

    template <typename T>
    void recv(T& data) {
        switch (_word_len.native_value()) {
        case WordLen::Word8Bit:
            uint8_t byte8[sizeof(T)*2];
            for (size_t i = 0; i < sizeof(T)*2; ++i) {
                byte8[i] = SPI_readDataBlockingFIFO(_module.base) & 0x00FF;
            }
            emb::c28x::from_bytes<T>(data, byte8);
            break;
        case WordLen::Word16Bit:
            uint16_t byte16[sizeof(T)];
            for (size_t i = 0; i < sizeof(T); ++i) {
                byte16[i] = SPI_readDataBlockingFIFO(_module.base);
            }
            memcpy(&data, byte16, sizeof(T));
            break;
        }
    }

    template <typename T>
    void send(const T& data) {
        switch (_word_len.native_value()) {
        case WordLen::Word8Bit:
            uint8_t byte8[sizeof(T)*2];
            emb::c28x::to_bytes<T>(byte8, data);
            for (size_t i = 0; i < sizeof(T)*2; ++i) {
                SPI_writeDataBlockingFIFO(_module.base, byte8[i] << 8);
            }
            break;
        case WordLen::Word16Bit:
            uint16_t byte16[sizeof(T)];
            memcpy(byte16, &data, sizeof(T));
            for (size_t i = 0; i < sizeof(T); ++i) {
                SPI_writeDataBlockingFIFO(_module.base, byte16[i]);
            }
            break;
        }
    }

    void register_rx_interrupt_handler(void (*handler)(void)) {
        SPI_disableModule(_module.base);
        Interrupt_register(_module.pie_rx_int_num, handler);
        SPI_enableInterrupt(_module.base, SPI_INT_RXFF);
        SPI_enableModule(_module.base);
    }

    void enable_rx_interrupts() { Interrupt_enable(_module.pie_rx_int_num); }
    void disable_rx_interrupts() { Interrupt_disable(_module.pie_rx_int_num); }
    void acknowledge_rx_interrupt() {
        SPI_clearInterruptStatus(_module.base, SPI_INT_RXFF);
        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
    }

    void reset_rx_fifo() { SPI_resetRxFIFO(_module.base); }
    void reset_tx_fifo() { SPI_resetTxFIFO(_module.base); }
protected:
#ifdef CPU1
    static void _init_pins(const MosiPinConfig& mosi_pin, const MisoPinConfig& miso_pin,
                           const ClkPinConfig& clk_pin, emb::optional<CsPinConfig> cs_pin);
#endif
};


} // namespace spi

} // namespace c28x

} // namespace mcu


#endif
