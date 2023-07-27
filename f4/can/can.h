#pragma once


#ifdef STM32F4xx

#include "../mcu_def.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib_stm32/core.h>
#include <emblib_stm32/interfaces/can.h>
#include <utility>


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

inline constexpr std::array<IRQn_Type, peripheral_count> irq_fifo0_numbers = {	
    CAN1_RX0_IRQn,
    CAN2_RX0_IRQn,
};


inline constexpr std::array<IRQn_Type, peripheral_count> irq_fifo1_numbers = {	
    CAN1_RX1_IRQn,
    CAN2_RX1_IRQn,
};


inline constexpr std::array<IRQn_Type, peripheral_count> irq_tx_numbers = {	
    CAN1_TX_IRQn,
    CAN2_TX_IRQn,
};


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


// inline constexpr std::array<uint32_t, 9> data_len_codes = {	
//     FDCAN_DLC_BYTES_0,
//     FDCAN_DLC_BYTES_1,
//     FDCAN_DLC_BYTES_2,
//     FDCAN_DLC_BYTES_3,
//     FDCAN_DLC_BYTES_4,
//     FDCAN_DLC_BYTES_5,
//     FDCAN_DLC_BYTES_6,
//     FDCAN_DLC_BYTES_7,
//     FDCAN_DLC_BYTES_8
// };

} // namespace impl


struct MessageAttribute {
    uint32_t location;
    uint32_t filter_index;
    bool operator==(const MessageAttribute&) const = default;
};


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
    friend void ::HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*);
    friend void ::HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef*);
    friend void ::HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef*);
    friend void ::HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef*);
    friend void ::HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef*);
private:
    const Peripheral _peripheral;
    CAN_HandleTypeDef _handle = {};

    mcu::gpio::Input _rx_pin;
    mcu::gpio::Output _tx_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled = {};

    static inline int _filter_count = 0;

    #ifdef CAN2
    static const int max_fitler_count = 28;
    #else
    static const int max_fitler_count = 14;
    #endif

    uint64_t _tx_error_counter = 0;
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

    HalStatus send(can_frame& frame, uint32_t* ret_mailbox = nullptr) {
        CAN_TxHeaderTypeDef header = {
            .StdId = frame.id,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = frame.len,
            .TransmitGlobalTime = DISABLE
        };

        uint32_t mailbox = 0;

        HalStatus status = HAL_CAN_AddTxMessage(&_handle, &header, frame.payload.data(), &mailbox);
        if (status != HAL_OK) {
            ++_tx_error_counter;
        }

        if (ret_mailbox) {
            *ret_mailbox = mailbox;
        }

        return status;
    }

    HalStatus send(CAN_TxHeaderTypeDef& header, can_payload& payload, uint32_t* ret_mailbox = nullptr) {
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
    void (*_on_fifo0_frame_received)(Module&, const MessageAttribute&, const can_frame&) = [](auto, auto, auto){ emb::fatal_error("uninitialized callback"); };
    void (*_on_fifo1_frame_received)(Module&, const MessageAttribute&, const can_frame&) = [](auto, auto, auto){ emb::fatal_error("uninitialized callback"); };
    void (*_on_txmailbox_free)(Module&) = [](auto){ emb::fatal_error("uninitialized callback"); };
public:
    void register_on_fifo0_frame_received_callback(void(*callback)(Module&, const MessageAttribute&, const can_frame&)) { _on_fifo0_frame_received = callback; }
    void register_on_fifo1_frame_received_callback(void(*callback)(Module&, const MessageAttribute&, const can_frame&)) { _on_fifo1_frame_received = callback; }
    void register_on_txmailbox_free_callback(void(*callback)(Module&)) { _on_txmailbox_free = callback; }

    void init_interrupts(uint32_t interrupt_list);
    void set_fifo_watermark(uint32_t fifo, uint32_t watermark);

    void set_interrupt_priority(InterruptPriority fifo0_priority, InterruptPriority fifo1_priority, InterruptPriority tx_priority) {
        HAL_NVIC_SetPriority(impl::irq_fifo0_numbers[std::to_underlying(_peripheral)], fifo0_priority.get(), 0);
        HAL_NVIC_SetPriority(impl::irq_fifo1_numbers[std::to_underlying(_peripheral)], fifo1_priority.get(), 0);
        HAL_NVIC_SetPriority(impl::irq_tx_numbers[std::to_underlying(_peripheral)], tx_priority.get(), 0);

    }

    void enable_interrupts() {
        HAL_NVIC_EnableIRQ(impl::irq_fifo0_numbers[std::to_underlying(_peripheral)]);
        HAL_NVIC_EnableIRQ(impl::irq_fifo1_numbers[std::to_underlying(_peripheral)]);
        HAL_NVIC_EnableIRQ(impl::irq_tx_numbers[std::to_underlying(_peripheral)]);
    }

    void disable_interrupts() {
        HAL_NVIC_DisableIRQ(impl::irq_fifo0_numbers[std::to_underlying(_peripheral)]);
        HAL_NVIC_DisableIRQ(impl::irq_fifo1_numbers[std::to_underlying(_peripheral)]);
        HAL_NVIC_DisableIRQ(impl::irq_tx_numbers[std::to_underlying(_peripheral)]);
    }

protected:
    void enable_clk();
};









} // namespace can

} // namespace mcu

#endif
