#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include "timerdef.h"
#include <utility>


namespace mcu {


namespace timers {
    
    
namespace gp {


enum class Peripheral : unsigned int {
    tim2,
    tim3,
    tim4,
    tim5,
    tim9,
    tim10,
    tim11,
    tim12,
    tim13,
    tim14
};


constexpr size_t peripheral_count = 10;


namespace impl {


inline const std::array<TIM_TypeDef*, peripheral_count> instances = {
    TIM2, TIM3, TIM4, TIM5,
    TIM9, TIM10, TIM11, TIM12, TIM13, TIM14
};


inline Peripheral to_peripheral(const TIM_TypeDef* instance) {
    return static_cast<Peripheral>(std::distance(instances.begin(),
                                                 std::find(instances.begin(), instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> clk_enable_funcs = {
    [](){ __HAL_RCC_TIM2_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM3_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM4_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM5_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM9_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM10_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM11_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM12_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM13_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM14_CLK_ENABLE(); },
};


inline std::array<bool, peripheral_count> clk_enabled = {};


inline constexpr std::array<IRQn_Type, peripheral_count> irq_nums = {
    TIM2_IRQn, TIM3_IRQn, TIM4_IRQn, TIM5_IRQn,
    TIM1_BRK_TIM9_IRQn, TIM1_UP_TIM10_IRQn, TIM1_TRG_COM_TIM11_IRQn,
    TIM8_BRK_TIM12_IRQn, TIM8_UP_TIM13_IRQn, TIM8_TRG_COM_TIM14_IRQn
};


} // namespace impl













} // namespace gp


} // namespace timers


} // namepsace mcu


#endif
#endif
