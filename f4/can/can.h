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

    mcu::gpio::Input _rx_pin;
    mcu::gpio::Output _tx_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled{};

    int _filter_count{0};

    #ifdef CAN2
    static const int max_fitler_count{28};
    #else
    static const int max_fitler_count{14};
    #endif

    emb::queue<std::pair<CAN_TxHeaderTypeDef, can_payload>, 32> _tx_queue;

    uint64_t _tx_error_counter{0};
public:
    Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config);
    MessageAttribute register_message(CAN_FilterTypeDef& filter);
    
    Peripheral peripheral() const { return _peripheral; }
    CAN_HandleTypeDef* handle() { return &_handle; }
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

    bool mailbox_ready() const {
        uint32_t tsr = READ_REG(_handle.Instance->TSR);
        if (((tsr & CAN_TSR_TME0) != 0U)
         || ((tsr & CAN_TSR_TME1) != 0U)
         || ((tsr & CAN_TSR_TME2) != 0U)) {
            return true;
        };
        return false;
    }

    HalStatus send(can_frame& frame, uint32_t* ret_mailbox = nullptr) {
        CAN_TxHeaderTypeDef header = {
            .StdId = frame.id,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = frame.len,
            .TransmitGlobalTime = DISABLE
        };

        return send(header, frame.payload, ret_mailbox);
    }

    HalStatus send(CAN_TxHeaderTypeDef& header, can_payload& payload, uint32_t* ret_mailbox = nullptr) {
        if (!mailbox_ready()) {
            if (_tx_queue.full()) {
                return HalStatus::HAL_ERROR;
            }
            _tx_queue.push({header, payload});
            return HalStatus::HAL_BUSY;
        }
        
        uint32_t mailbox = 0;

        HalStatus status = HAL_CAN_AddTxMessage(&_handle, &header, payload.data(), &mailbox);
        if (status != HAL_OK) {
            ++_tx_error_counter;
        }

        if (ret_mailbox) {
            *ret_mailbox = mailbox;
        }

        return status;		
    }

    /* INTERRUPTS */
private:
    void on_txmailbox_free() {
        if (_tx_queue.empty()) { return; }
        auto header = _tx_queue.front().first;
        auto payload = _tx_queue.front().second;

        if (send(header, payload) == HalStatus::HAL_OK) {
            _tx_queue.pop();
        }    
    }
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
