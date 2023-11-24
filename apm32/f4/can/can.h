#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "../gpio/gpio.h"
#include <apm32f4xx_can.h>
#include <emblib/core.h>
#include <emblib/interfaces/can.h>
#include <emblib/queue.h>


extern "C" {
void CAN1_RX0_IRQHandler();
void CAN1_RX1_IRQHandler();
void CAN1_TX_IRQHandler();
void CAN2_RX0_IRQHandler();
void CAN2_RX1_IRQHandler();
void CAN2_TX_IRQHandler();
}


namespace mcu {

    
namespace can {


enum class Peripheral : unsigned int {
    can1,
    can2
};


constexpr size_t peripheral_count = 2;


struct RxPinConfig {
    GPIO_T* port;
    uint16_t pin;
    GPIO_AF_T af_selection;
};


struct TxPinConfig {
    GPIO_T* port;
    uint16_t pin;
    GPIO_AF_T af_selection;
};


struct Config {
    CAN_Config_T hal_config;
};


namespace impl {


inline const std::array<CAN_T*, peripheral_count> can_instances = {CAN1, CAN2};


inline Peripheral to_peripheral(const CAN_T* instance) {
    return static_cast<Peripheral>(
        std::distance(can_instances.begin(), std::find(can_instances.begin(), can_instances.end(), instance))
    );
}


inline std::array<void(*)(void), peripheral_count> can_clk_enable_funcs = {
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN1); },
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_CAN2); }
};


inline constexpr std::array<IRQn_Type, peripheral_count> can_fifo0_irqn = {CAN1_RX0_IRQn, CAN2_RX0_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> can_fifo1_irqn = {CAN1_RX1_IRQn, CAN2_RX1_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> can_tx_irqn = {CAN1_TX_IRQn, CAN2_TX_IRQn};


} // namespace impl


enum class RxFifo {
    fifo0,
    fifo1
};


struct RxMessageAttribute {
    RxFifo fifo;
    uint32_t filter_idx{0xBAAAAAAD};
    bool operator==(const RxMessageAttribute&) const = default;
};


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
    friend void ::CAN1_RX0_IRQHandler();
    friend void ::CAN1_RX1_IRQHandler();
    friend void ::CAN1_TX_IRQHandler();
    friend void ::CAN2_RX0_IRQHandler();
    friend void ::CAN2_RX1_IRQHandler();
    friend void ::CAN2_TX_IRQHandler();
private:
    const Peripheral _peripheral;
    CAN_T* _reg;

    mcu::gpio::AlternateIO _rx_pin;
    mcu::gpio::AlternateIO _tx_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled{};

    unsigned int _filter_count{0};
    #ifdef CAN2
    static const int max_fitler_count{28};
    #else
    static const int max_fitler_count{14};
    #endif

    emb::queue<can_frame, 32> _txqueue;
public:
    Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config);
    RxMessageAttribute register_rxmessage(CAN_FilterTypeDef& filter);
    
    Peripheral peripheral() const { return _peripheral; }
    CAN_HandleTypeDef* handle() { return &_handle; }
    CAN_TypeDef* reg() { return _reg; }
    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void start();
    void stop();

    bool mailbox_empty() const {
        if (bit_is_clear<uint32_t>(_reg->TSR, CAN_TSR_TME)) {
            return false;
        }
        return true;
    }

    uint32_t rxfifo_level(RxFifo fifo) const {
        switch (fifo) {
        case RxFifo::fifo0:
            return read_bit<uint32_t>(_reg->RF0R, CAN_RF0R_FMP0);
        case RxFifo::fifo1:
            return read_bit<uint32_t>(_reg->RF1R, CAN_RF1R_FMP1);
        }
        return 0;
    }

    Error send(const can_frame& frame);
    std::optional<RxMessageAttribute> recv(can_frame& frame, RxFifo fifo) const;

public:
    void init_interrupts(uint32_t interrupt_list);
    void set_interrupt_priority(IrqPriority fifo0_priority, IrqPriority fifo1_priority, IrqPriority tx_priority);
    void enable_interrupts();
    void disable_interrupts();

private:
    void on_txmailbox_empty() {
        if (_txqueue.empty()) { return; }
        auto frame = _txqueue.front();
        _txqueue.pop();
        send(frame);   
    }

private:
    static void _enable_clk(Peripheral peripheral);
};


} // namespace can


} // namespace mcu


#endif
#endif
