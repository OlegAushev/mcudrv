#pragma once

#ifdef STM32F4xx

#include "../mcu_def.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include "../dma/dma.h"
#include <utility>


namespace mcu {

namespace adc {


inline constexpr bool strict_error_check = true;


enum class Peripheral {
    adc1,
    adc2,
    adc3
};


constexpr int peripheral_count = 3;


struct PinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
};


struct Config {
    ADC_InitTypeDef hal_cfg; 
};


struct RegularChannelConfig {
    ADC_ChannelConfTypeDef hal_cfg;
};


struct InjectedChannelConfig {
    ADC_InjectionConfTypeDef hal_cfg;
};

namespace impl {


inline constexpr std::array<ADC_TypeDef*, peripheral_count> adc_instances = {ADC1, ADC2, ADC3};


inline Peripheral to_peripheral(const ADC_TypeDef* instance) {
    return static_cast<Peripheral>(
        std::distance(adc_instances.begin(), std::find(adc_instances.begin(), adc_instances.end(), instance))
    );
}


inline std::array<void(*)(void), peripheral_count> adc_clk_enable_funcs = {
    [](){ __HAL_RCC_ADC1_CLK_ENABLE(); },
    [](){ __HAL_RCC_ADC2_CLK_ENABLE(); },
    [](){ __HAL_RCC_ADC3_CLK_ENABLE(); }
};


} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
    friend void ::HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef*);
    friend void ::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);
    friend void ::HAL_ADC_ErrorCallback(ADC_HandleTypeDef*);
private:
    const Peripheral _peripheral;
    ADC_HandleTypeDef _handle = {};
    static inline std::array<bool, peripheral_count> _clk_enabled = {};
public:
    Module(Peripheral peripheral, const Config& config);
    Peripheral peripheral() const { return _peripheral; }
    ADC_HandleTypeDef* handle() { return &_handle; }
    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void add_injected_channel(PinConfig pin_config, InjectedChannelConfig channel_config);
    void add_regular_channel(PinConfig pin_config, RegularChannelConfig channel_config);
    void add_injected_internal_channel(InjectedChannelConfig channel_config);
    void add_regular_internal_channel(RegularChannelConfig channel_config);

    void start_injected() {
        set_bit(_handle.Instance->CR2, ADC_CR2_JSWSTART);
    }



    HalStatus start_regular_conversion() {
        return HAL_ADC_Start(&_handle);
    }

    HalStatus start_regular_conversion_it() {
        return HAL_ADC_Start_IT(&_handle);
    }

    HalStatus poll(uint32_t timeout = 0) { return HAL_ADC_PollForConversion(&_handle, timeout); }
    uint32_t read_regular_conversion() { return HAL_ADC_GetValue(&_handle); }

    /* INTERRUPTS */
public:
    void init_interrupts();
    void enable_interrupts() { enable_interrupt(ADC_IRQn); }
    void disable_interrupts() { disable_interrupt(ADC_IRQn); }
protected:
    static void _init_adc1_interrupts();
    static void _init_adc2_interrupts();
    static void _init_adc3_interrupts();
    // static void _enable_adc1_interrupts();
    // static void _enable_adc2_interrupts();
    // static void _enable_adc3_interrupts();
    // static void _disable_adc1_interrupts();
    // static void _disable_adc2_interrupts();
    // static void _disable_adc3_interrupts();
private:
    void (*_on_half_completed)(Module&) = [](Module&){ fatal_error("uninitialized callback"); };
    void (*_on_completed)(Module&) = [](Module&){ fatal_error("uninitialized callback"); };
    void (*_on_error)(Module&) = [](Module&){ fatal_error("uninitialized callback"); };
public:
    void register_on_half_completed_callback(void(*callback)(Module&)) { _on_half_completed = callback; }
    void register_on_completed_callback(void(*callback)(Module&)) { _on_completed = callback; }
    void register_on_error_callback(void(*callback)(Module&)) { _on_error = callback; }

    static bool regular_irq_pending(Peripheral peripheral) {
        auto instance = impl::adc_instances[std::to_underlying(peripheral)];
        if (mcu::is_bit_set(instance->SR, ADC_FLAG_EOC) && mcu::is_bit_set(instance->CR1, ADC_IT_EOC)) {
            return true;
        }
        return false;
    }
protected:
    static void _enable_clk(Peripheral peripheral);
};


} // namespace adc

} // namespace mcu

#endif

