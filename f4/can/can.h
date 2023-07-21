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

// inline constexpr std::array<IRQn_Type, 2> irq0_numbers = {	
//     CAN1_IT0_IRQn,
//     CAN2_IT0_IRQn,
// };


// inline constexpr std::array<IRQn_Type, 2> irq1_numbers = {	
//     CAN1_IT1_IRQn,
//     CAN2_IT1_IRQn,
// };


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


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    CAN_HandleTypeDef _handle = {};

    mcu::gpio::Input _rx_pin;
    mcu::gpio::Output _tx_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled = {};
public:
    Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config);
protected:
    void enable_clk();
};









} // namespace can

} // namespace mcu

#endif
