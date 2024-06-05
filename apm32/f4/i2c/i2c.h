#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/apm32_base.h>
#include <mcudrv/apm32/f4/system/system.h>
#include <mcudrv/apm32/f4/gpio/gpio.h>
#include <apm32f4xx_i2c.h>


namespace mcu {
namespace i2c {


constexpr size_t peripheral_count = 3;
enum class Peripheral : unsigned int {
    i2c1,
    i2c2,
    i2c3
};


enum class Direction { rx, tx };


enum class Event : uint32_t {
    ev5_master_mode_select                      = 0x00030001,  /*!< BUSBSYFLG, MSFLG and STARTFLG flag */
    ev6_master_transmitter_mode_selected        = 0x00070082,  /*!< BUSBSYFLG, MSFLG, ADDRFLG, TXBEFLG and TRFLG flags */
    ev6_master_receiver_mode_selected           = 0x00030002,  /*!< BUSBSYFLG, MSFLG and ADDRFLG flags */
    ev9_master_mode_address10                   = 0x00030008,  /*!< BUSBSYFLG, MSFLG and ADDR10FLG flags */
    ev7_master_byte_received                    = 0x00030040,  /*!< BUSBSYFLG, MSFLG and RXBNEFLG flags */
    ev8_master_byte_transmitting                = 0x00070080,  /*!< TRFLG, BUSBSYFLG, MSFLG, TXBEFLG flags */
    ev8_2_master_byte_transmitted               = 0x00070084,  /*!< TRFLG, BUSBSYFLG, MSFLG, TXBEFLG and BTCFLG flags */

    ev1_slave_receiver_address_matched          = 0x00020002, /*!< BUSBSYFLG and ADDRFLG flags */
    ev1_slave_transmitter_address_matched       = 0x00060082, /*!< TRFLG, BUSBSYFLG, TXBEFLG and ADDRFLG flags */
    ev1_slave_receiver_secondaddress_matched    = 0x00820000, /*!< DUALF and BUSBSYFLG flags */
    ev1_slave_transmitter_secondaddress_matched = 0x00860080, /*!< DUALF, TRFLG, BUSBSYFLG and TXBEFLG flags */
    ev1_slave_generalcalladdress_matched        = 0x00120000, /*!< GENCALL and BUSBSYFLG flags */
    ev2_slave_byte_received                     = 0x00020040, /*!< BUSBSYFLG and RXBNEFLG flags */
    ev2_slave_byte_received1                    = 0x00820040, /*!< DUALADDRFLG, BUSBSYFLG and RXBNEFLG flags */
    ev2_slave_byte_received2                    = 0x00120040, /*!< GENCALLFLG, BUSBSYFLG and RXBNEFLG flags */
    ev4_slave_stop_detected                     = 0x00000010, /*!< STOPFLG flag */
    ev3_slave_byte_transmitted                  = 0x00060084, /*!< TRFLG, BUSBSYFLG, TXBEFLG and BTCFLG flags */
    ev3_slave_byte_transmitted1                 = 0x00860084, /*!< DUALADDRFLG, TRFLG, BUSBSYFLG, TXBEFLG and BTCFLG flags */
    ev3_slave_byte_transmitted2                 = 0x00160084, /*!< GENCALLFLG, TRFLG, BUSBSYFLG, TXBEFLG and BTCFLG flags */
    ev3_slave_byte_transmitting                 = 0x00060080, /*!< TRFLG, BUSBSYFLG and TXBEFLG flags */
    ev3_2_slave_ack_failure                     = 0x00000400, /*!< AEFLG flag */   
};


struct SdaPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T altfunc; };
struct SclPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T altfunc; };


struct Config {
    I2C_Config_T hal_config;
};


namespace impl {


inline const std::array<I2C_T*, peripheral_count> instances = {I2C1, I2C2, I2C3};


inline Peripheral to_peripheral(const I2C_T* instance) {
    return static_cast<Peripheral>(
        std::distance(instances.begin(), std::find(instances.begin(), instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> clk_enable_funcs = {
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_I2C1); },
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_I2C2); },
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_I2C3); },
};


inline constexpr std::array<IRQn_Type, peripheral_count> event_irqn = {I2C1_EV_IRQn, I2C2_EV_IRQn, I2C3_EV_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> error_irqn = {I2C1_ER_IRQn, I2C2_ER_IRQn, I2C3_ER_IRQn};


} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    I2C_T* const _reg;
    mcu::gpio::AlternatePin _sda_pin;
    mcu::gpio::AlternatePin _scl_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    Module(Peripheral peripheral,
           const SdaPinConfig& sda_pin_config, const SclPinConfig& scl_pin_config,
           const Config& config);

    Peripheral peripheral() const { return _peripheral; }
    I2C_T* reg() { return _reg; }

    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void enable() { _reg->CTRL1_B.I2CEN = 1; }
    void disable() { _reg->CTRL1_B.I2CEN = 0; }

    void toggle_start(bool v = true) { _reg->CTRL1_B.START = v; }
    void toggle_stop(bool v = true) { _reg->CTRL1_B.STOP = v; }
    void toggle_ack(bool v = true) { _reg->CTRL1_B.ACKEN = v; }
    void toggle_ackpos(bool v = true) { _reg->CTRL1_B.ACKPOS = v; }

    bool busy() const { return _reg->STS2_B.BUSBSYFLG == 1; }
    bool rx_empty() const { return _reg->STS1_B.RXBNEFLG == 0; }
    bool tx_empty() const { return _reg->STS1_B.TXBEFLG == 1; }

    uint32_t read_status_regs() const {
        uint32_t sts1 = _reg->STS1 & 0x0000FFFF;
        uint32_t sts2 = _reg->STS2 & 0x000000FF;
        return sts1 | (sts2 << 16);
    }

    static bool is_event(uint32_t sts_regs, Event event) {
        return (sts_regs & std::to_underlying(event)) == std::to_underlying(event);
    }

    void put_addr(uint8_t addr, Direction dir) {
        addr = uint8_t((addr & 0x7F) << 1);
        if (dir == Direction::rx) {
            _reg->DATA_B.DATA = addr | 0x01;
        } else {
            _reg->DATA_B.DATA = addr & 0xFE;
        }
    }

    exec_status put_data(uint8_t data) {
        if (!tx_empty()) {
            return exec_status::busy;
        }
        _reg->DATA_B.DATA = data;
        return exec_status::ok;
    }

    std::optional<uint8_t> get_data() const {
        if (rx_empty()) {
            return {};
        }
        uint8_t data = _reg->DATA_B.DATA;
        return {data};
    }

    void clear_errors() {
        _reg->STS1_B.SMBALTFLG = 0;
        _reg->STS1_B.TTEFLG = 0;
        _reg->STS1_B.PECEFLG = 0;
        _reg->STS1_B.OVRURFLG = 0;
        _reg->STS1_B.AEFLG = 0;
        _reg->STS1_B.ALFLG = 0;
        _reg->STS1_B.BERRFLG = 0;
    }
public:
    void init_event_interrupts(bool enable_buf_it, IrqPriority priority) {
        if (enable_buf_it) {
            _reg->CTRL2_B.BUFIEN = 1;
        }
        _reg->CTRL2_B.EVIEN = 1;
        set_irq_priority(impl::event_irqn[std::to_underlying(_peripheral)], priority);
    }

    void init_error_interrupts(IrqPriority priority) {
        _reg->CTRL2_B.ERRIEN = 1;
        set_irq_priority(impl::error_irqn[std::to_underlying(_peripheral)], priority);
    }

    void enable_event_interrupts() { enable_irq(impl::event_irqn[std::to_underlying(_peripheral)]); }
    void enable_error_interrupts() { enable_irq(impl::error_irqn[std::to_underlying(_peripheral)]); }
    void disable_event_interrupts() { disable_irq(impl::event_irqn[std::to_underlying(_peripheral)]); }
    void disable_error_interrupts() { disable_irq(impl::event_irqn[std::to_underlying(_peripheral)]); }
protected:
    static void _enable_clk(Peripheral peripheral);
};














} // namespace i2c
} // namespace mcu


#endif
#endif
