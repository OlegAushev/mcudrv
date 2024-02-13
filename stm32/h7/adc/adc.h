#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include "../stm32_h7_base.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include "../dma/dma.h"
#include <initializer_list>
#include <utility>


namespace mcu {


namespace adc {


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
    ADC_InitTypeDef hal_config;
};


struct RegularChannelConfig {
    ADC_ChannelConfTypeDef hal_config;
    std::initializer_list<uint32_t> ranks;
};


struct InjectedChannelConfig {
    ADC_InjectionConfTypeDef hal_config;
};


enum class InjectedChannelRank {
    rank1 = ADC_INJECTED_RANK_1,
    rank2 = ADC_INJECTED_RANK_2,
    rank3 = ADC_INJECTED_RANK_3,
    rank4 = ADC_INJECTED_RANK_4
};


namespace impl {


inline const std::array<ADC_TypeDef*, peripheral_count> adc_instances = {ADC1, ADC2, ADC3};


inline const std::array<ADC_Common_TypeDef*, peripheral_count> adc_common_instances = {ADC12_COMMON, ADC12_COMMON, ADC3_COMMON};


inline Peripheral to_peripheral(const ADC_TypeDef* instance) {
    return static_cast<Peripheral>(
        std::distance(adc_instances.begin(), std::find(adc_instances.begin(), adc_instances.end(), instance))
    );
}


} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    ADC_HandleTypeDef _handle{};
    ADC_TypeDef* const _reg;
    ADC_Common_TypeDef* const _reg_common;
    static inline std::array<bool, 2> _clk_enabled{false, false};
public:
    Module(Peripheral peripheral, const Config& config, dma::Stream* dma = nullptr);
    
    Peripheral peripheral() const { return _peripheral; }
    ADC_HandleTypeDef* handle() { return &_handle; }
    ADC_TypeDef* reg() { return _reg; }
    ADC_Common_TypeDef* reg_common() { return _reg_common; }

    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void calibrate();
    void initialize_injected_channel(const PinConfig& pin_config, InjectedChannelConfig channel_config);
    void initialize_regular_channel(const PinConfig& pin_config, const RegularChannelConfig& channel_config);
    void initialize_injected_internal_channel(InjectedChannelConfig channel_config);
    void initialize_regular_internal_channel(RegularChannelConfig channel_config);

    void start_injected() {
        if (bit_is_set<uint32_t>(_reg->CR, ADC_CR_JADSTART)) {
            return; // there is ongoing injected channel conversion
        }
        set_bit<uint32_t>(_reg->CR, ADC_CR_JADSTART);
    }

    uint32_t read_injected(InjectedChannelRank rank) {
        switch (rank) {
        case InjectedChannelRank::rank1:
            return _reg->JDR1;
        case InjectedChannelRank::rank2:
            return _reg->JDR2;
        case InjectedChannelRank::rank3:
            return _reg->JDR3;
        case InjectedChannelRank::rank4:
            return _reg->JDR4;
        }
        return 0xFFFFFFFF;
    } 

    void acknowledge_injected_conversion() {
        set_bit<uint32_t>(_reg->ISR, ADC_ISR_JEOC);
    }

    void acknowledge_injected_sequence() {
        set_bit<uint32_t>(_reg->ISR, ADC_ISR_JEOS);
    }

    void start_regular() {
        if (bit_is_set<uint32_t>(_reg->CR, ADC_CR_ADSTART)) {
            return; // there is ongoing regular channel conversion
        }
        set_bit<uint32_t>(_reg->CR, ADC_CR_ADSTART);
    }

    bool busy() const {
        return bit_is_set<uint32_t>(_reg->CR, ADC_CR_ADSTART);
    }

    bool regular_ready() const {
        return bit_is_set<uint32_t>(_reg->ISR, ADC_ISR_EOC);
    }

    uint32_t read_regular() {
        return _reg->DR;
    }

    void acknowledge_regular_conversion() {
        set_bit<uint32_t>(_reg->ISR, ADC_ISR_EOC);
    }

    void acknowledge_regular_sequence() {
        set_bit<uint32_t>(_reg->ISR, ADC_ISR_EOS);
    }

//     void enable_dma(dma::StreamController& dma_stream) {
//         _handle.DMA_Handle = dma_stream.handle();
//         dma_stream.handle()->Parent = &_handle;
//     }

    // DrvStatus start_regular_conversion() {
    //     DrvStatus status = static_cast<DrvStatus>(HAL_ADC_Start(&_handle));
    //     if constexpr (strict_error_check) {
    //         if (status != DrvStatus::ok) {
    //             fatal_error("regular ADC conversion start failed");
    //         }
    //     }
    //     return status;
    // }

//     template <uint32_t DmaBufSize>
//     DrvStatus start_regular_conversion_with_dma(mcu::dma::Buffer<uint16_t, DmaBufSize>& buf) {
//         DrvStatus status = static_cast<DrvStatus>(
//                 HAL_ADC_Start_DMA(&_handle, reinterpret_cast<uint32_t*>(buf.data()), buf.size()));
//         if constexpr (strict_error_check) {
//             if (status != DrvStatus::ok) {
//                 fatal_error("regular ADC conversion with DMA start failed");
//             }
//         }
//         return status;
//     }

//     DrvStatus poll() { return static_cast<DrvStatus>(HAL_ADC_PollForConversion(&_handle, 0)); }
//     uint32_t read_regular_conversion() { return HAL_ADC_GetValue(&_handle); }
//     uint32_t read_injected_conversion(uint32_t injected_rank) { return HAL_ADCEx_InjectedGetValue(&_handle, injected_rank); }

//     /* INTERRUPTS */
// public:
//     void (*on_half_completed)() = [](){ fatal_error("uninitialized callback"); };
//     void (*on_completed)() = [](){ fatal_error("uninitialized callback"); };
//     void (*on_error)() = [](){ fatal_error("uninitialized callback"); };
protected:
    static void _enable_clk(Peripheral peripheral);
};


} // namespace adc


} // namespace mcu


#endif
#endif
