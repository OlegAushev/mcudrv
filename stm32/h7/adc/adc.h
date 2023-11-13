#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include "../stm32_h7_base.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include "../dma/dma.h"
#include <utility>


namespace mcu {


namespace adc {


inline constexpr bool strict_error_check = true;


enum class Peripheral : unsigned int {
    adc1,
    adc2,
    adc3
};


constexpr size_t peripheral_count = 3;


struct PinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
};


struct Config {
    ADC_InitTypeDef hal_init;
    //ADC_ChannelConfTypeDef channel; 
};


struct ChannelConfig {
    ADC_ChannelConfTypeDef hal_init;
};


namespace impl {


inline const std::array<ADC_TypeDef*, peripheral_count> adc_instances = {ADC1, ADC2, ADC3};


inline Peripheral to_peripheral(const ADC_TypeDef* instance) {
    return static_cast<Peripheral>(
        std::distance(adc_instances.begin(), std::find(adc_instances.begin(), adc_instances.end(), instance))
    );
}

} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    ADC_HandleTypeDef _handle = {};
    static inline std::array<bool, 2> _clk_enabled = {false, false};
public:
    Module(Peripheral peripheral, const Config& config);
    Peripheral peripheral() const { return _peripheral; }
    ADC_HandleTypeDef* handle() { return &_handle; }
    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void calibrate();
    void add_regular_channel(PinConfig pin_config, ChannelConfig channel_config);
    void add_internal_channel(ChannelConfig channel_config);

    void enable_dma(dma::StreamController& dma_stream) {
        _handle.DMA_Handle = dma_stream.handle();
        dma_stream.handle()->Parent = &_handle;
    }

    HalStatus start_regular_conversion() {
        HalStatus status = HAL_ADC_Start(&_handle);
        if constexpr (strict_error_check) {
            if (status != HAL_OK) {
                fatal_error("regular ADC conversion start failed");
            }
        }
        return status;
    }

    template <uint32_t DmaBufSize>
    HalStatus start_regular_conversion_with_dma(mcu::dma::Buffer<uint16_t, DmaBufSize>& buf) {
        HalStatus status = HAL_ADC_Start_DMA(&_handle, reinterpret_cast<uint32_t*>(buf.data()), buf.size());
        if constexpr (strict_error_check) {
            if (status != HAL_OK) {
                fatal_error("regular ADC conversion with DMA start failed");
            }
        }
        return status;
    }

    HalStatus poll() { return HAL_ADC_PollForConversion(&_handle, 0); }
    uint32_t read_regular_conversion() { return HAL_ADC_GetValue(&_handle); }
    uint32_t read_injected_conversion(uint32_t injected_rank) { return HAL_ADCEx_InjectedGetValue(&_handle, injected_rank); }

    /* INTERRUPTS */
public:
    void (*on_half_completed)() = [](){ fatal_error("uninitialized callback"); };
    void (*on_completed)() = [](){ fatal_error("uninitialized callback"); };
    void (*on_error)() = [](){ fatal_error("uninitialized callback"); };
protected:
    void enable_clk() {
        switch (_peripheral) {
        case Peripheral::adc1:
        case Peripheral::adc2:
            if (_clk_enabled[0]) return;
            __HAL_RCC_ADC12_CLK_ENABLE();
            _clk_enabled[0] = true;
            break;
        case Peripheral::adc3:
            if (_clk_enabled[1]) return;
            __HAL_RCC_ADC3_CLK_ENABLE();
            _clk_enabled[1] = true;
            break;
        default:
            fatal_error("invalid ADC module");
            break;
        }
    }
};


} // namespace adc


} // namespace mcu


#endif
#endif
