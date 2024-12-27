#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_can.h>

#include <mcudrv/apm32/f4/gpio/gpio.hpp>
#include <mcudrv/apm32/f4/system/system.hpp>
#include <mcudrv/generic/can.hpp>

#include <emblib/core.hpp>
#include <emblib/queue.hpp>

extern "C" {
void CAN1_RX0_IRQHandler();
void CAN1_RX1_IRQHandler();
void CAN1_TX_IRQHandler();
void CAN2_RX0_IRQHandler();
void CAN2_RX1_IRQHandler();
void CAN2_TX_IRQHandler();
}

namespace mcu {
namespace apm32 {
namespace can {

enum class Peripheral : unsigned int { can1, can2 };

constexpr size_t peripheral_count = 2;

struct RxPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_AF_T altfunc;
};

struct TxPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_AF_T altfunc;
};

struct Config {
    CAN_Config_T hal_config;
};

namespace impl {

inline const std::array<CAN_T*, peripheral_count> can_instances = {CAN1, CAN2};

inline Peripheral to_peripheral(const CAN_T* instance) {
    return static_cast<Peripheral>(std::distance(
            can_instances.begin(),
            std::find(can_instances.begin(), can_instances.end(), instance)));
}

inline std::array<void (*)(void), peripheral_count> can_clk_enable_funcs = {
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN1); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN2); }};

inline constexpr std::array<IRQn_Type, peripheral_count> can_rx0_irqn = {
    CAN1_RX0_IRQn, CAN2_RX0_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> can_rx1_irqn = {
    CAN1_RX1_IRQn, CAN2_RX1_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> can_tx_irqn = {
    CAN1_TX_IRQn, CAN2_TX_IRQn};

} // namespace impl

enum class RxFifo { fifo0, fifo1 };

struct RxMessageAttribute {
    RxFifo fifo;
    uint32_t filter_idx{0xBAAAAAAD};
    bool operator==(const RxMessageAttribute&) const = default;
};

class Module : public emb::singleton_array<Module, peripheral_count>,
               private emb::noncopyable {
private:
    const Peripheral _peripheral;
    CAN_T* _reg;

    gpio::AlternatePin _rx_pin;
    gpio::AlternatePin _tx_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled{};

    uint8_t _filter_count{0};
#ifdef CAN2
    static const uint8_t max_fitler_count{28};
#else
    static const uint8_t max_fitler_count{14};
#endif

    emb::queue<can_frame, 32> _txqueue;
public:
    Module(Peripheral peripheral,
           const RxPinConfig& rx_pin_config,
           const TxPinConfig& tx_pin_config,
           Config config);
    RxMessageAttribute register_rxmessage(CAN_FilterConfig_T& filter);

    Peripheral peripheral() const { return _peripheral; }
    CAN_T* reg() { return _reg; }
    static Module* instance(Peripheral peripheral) {
        return emb::singleton_array<Module, peripheral_count>::instance(
                std::to_underlying(peripheral));
    }

    void start();
    void stop();

    bool mailbox_full() const {
        if ((_reg->TXSTS_B.TXMEFLG0 + _reg->TXSTS_B.TXMEFLG1 +
             _reg->TXSTS_B.TXMEFLG2) == 0) {
            return true;
        }
        return false;
    }

    uint32_t rxfifo_level(RxFifo fifo) const {
        switch (fifo) {
        case RxFifo::fifo0:
            return _reg->RXF0_B.FMNUM0;
        case RxFifo::fifo1:
            return _reg->RXF1_B.FMNUM1;
        }
        return 0;
    }

    exec_status put_frame(const can_frame& frame);
    std::optional<RxMessageAttribute> get_frame(can_frame& frame,
                                                RxFifo fifo) const;
public:
    void init_interrupts(uint32_t interrupt_bitset);
    void set_interrupt_priority(IrqPriority rx0_priority,
                                IrqPriority rx1_priority,
                                IrqPriority tx_priority);
    void enable_interrupts();
    void disable_interrupts();

    void on_mailbox_empty() {
        while (!mailbox_full()) {
            if (_txqueue.empty()) {
                return;
            }
            put_frame(_txqueue.front());
            _txqueue.pop();
        }
    }
private:
    static void _enable_clk(Peripheral peripheral);
};

} // namespace can
} // namespace apm32
} // namespace mcu

#endif
#endif
