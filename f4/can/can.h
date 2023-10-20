#pragma once


#ifdef STM32F4xx

#include "../mcudef.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib/core.h>
#include <emblib/interfaces/can.h>
#include <emblib/queue.h>
#include <utility>


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


enum class Peripheral {
    can1,
    can2
};


constexpr int peripheral_count = 2;


struct RxPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


struct TxPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


struct Config {
    CAN_InitTypeDef hal_init;
};


namespace impl {


inline constexpr std::array<CAN_TypeDef*, peripheral_count> can_instances = {CAN1, CAN2};


inline Peripheral to_peripheral(const CAN_TypeDef* instance) {
    return static_cast<Peripheral>(
        std::distance(can_instances.begin(), std::find(can_instances.begin(), can_instances.end(), instance))
    );
}


inline std::array<void(*)(void), peripheral_count> can_clk_enable_funcs = {
    [](){ __HAL_RCC_CAN1_CLK_ENABLE(); },
    [](){ __HAL_RCC_CAN2_CLK_ENABLE(); }
};


inline constexpr std::array<IRQn_Type, peripheral_count> can_fifo0_irqn = {CAN1_RX0_IRQn, CAN2_RX0_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> can_fifo1_irqn = {CAN1_RX1_IRQn, CAN2_RX1_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> can_tx_irqn = {CAN1_TX_IRQn, CAN2_TX_IRQn};


} // namespace impl


struct MessageAttribute {
    uint32_t location;
    uint32_t filter_index;
    bool operator==(const MessageAttribute&) const = default;
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
    CAN_HandleTypeDef _handle{};
    CAN_TypeDef* _reg;

    mcu::gpio::Input _rx_pin;
    mcu::gpio::Output _tx_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled{};

    int _filter_count{0};
    #ifdef CAN2
    static const int max_fitler_count{28};
    #else
    static const int max_fitler_count{14};
    #endif

    emb::queue<can_frame, 32> _txqueue;
public:
    Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config);
    MessageAttribute register_message(CAN_FilterTypeDef& filter);
    
    Peripheral peripheral() const { return _peripheral; }
    CAN_HandleTypeDef* handle() { return &_handle; }
    CAN_TypeDef* reg() { return _reg; }
    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void start() {
        if (HAL_CAN_Start(&_handle) != HAL_OK) {
            fatal_error("CAN module start failed");
        }
    }

    void stop() {
        if (HAL_CAN_Stop(&_handle) != HAL_OK) {
            fatal_error("CAN module stop failed");
        }
    }

    bool mailbox_empty() const {
        if (is_bit_clr(_reg->TSR, CAN_TSR_TME)) {
            return false;
        }
        return true;
    }

    Error send(const can_frame& frame);

private:
    void on_txmailbox_empty() {
        if (_txqueue.empty()) { return; }
        auto frame = _txqueue.front();
        _txqueue.pop();
        send(frame);   
    }

    /* INTERRUPTS */
public:
    void init_interrupts(uint32_t interrupt_list);
    void set_fifo_watermark(uint32_t fifo, uint32_t watermark);

    void set_interrupt_priority(IrqPriority fifo0_priority, IrqPriority fifo1_priority, IrqPriority tx_priority) {
        HAL_NVIC_SetPriority(impl::can_fifo0_irqn[std::to_underlying(_peripheral)], fifo0_priority.get(), 0);
        HAL_NVIC_SetPriority(impl::can_fifo1_irqn[std::to_underlying(_peripheral)], fifo1_priority.get(), 0);
        HAL_NVIC_SetPriority(impl::can_tx_irqn[std::to_underlying(_peripheral)], tx_priority.get(), 0);

    }

    void enable_interrupts() {
        HAL_NVIC_EnableIRQ(impl::can_fifo0_irqn[std::to_underlying(_peripheral)]);
        HAL_NVIC_EnableIRQ(impl::can_fifo1_irqn[std::to_underlying(_peripheral)]);
        HAL_NVIC_EnableIRQ(impl::can_tx_irqn[std::to_underlying(_peripheral)]);
    }

    void disable_interrupts() {
        HAL_NVIC_DisableIRQ(impl::can_fifo0_irqn[std::to_underlying(_peripheral)]);
        HAL_NVIC_DisableIRQ(impl::can_fifo1_irqn[std::to_underlying(_peripheral)]);
        HAL_NVIC_DisableIRQ(impl::can_tx_irqn[std::to_underlying(_peripheral)]);
    }

protected:
    static void _enable_clk(Peripheral peripheral);
};


} // namespace can


} // namespace mcu


#endif
