#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/stm32_h7_base.h>
#include <mcudrv/stm32/h7/system/system.h>
#include <mcudrv/stm32/h7/gpio/gpio.h>
#include <mcudrv/stm32/h7/dma/dma.h>
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


class Module : public emb::singleton_array<Module, peripheral_count>, private emb::noncopyable {
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
        return emb::singleton_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void enable() {
        set_bit<uint32_t>(_reg->CR, ADC_CR_ADEN);
        while (bit_is_clear<uint32_t>(_reg->ISR, ADC_ISR_ADRDY)) {
            // wait
        }
    }

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
        clear_bit<uint32_t, bit_type::rc_w1>(_reg->ISR, ADC_ISR_JEOC);
    }

    void acknowledge_injected_sequence() {
        clear_bit<uint32_t, bit_type::rc_w1>(_reg->ISR, ADC_ISR_JEOS);
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
        clear_bit<uint32_t, bit_type::rc_w1>(_reg->ISR, ADC_ISR_EOC);
    }

    void acknowledge_regular_sequence() {
        clear_bit<uint32_t, bit_type::rc_w1>(_reg->ISR, ADC_ISR_EOS);
    }

public:
    void initialize_interrupts(uint32_t interrupt_list, mcu::IrqPriority priority);
    void enable_interrupts() { enable_irq(ADC_IRQn); }
    void disable_interrupts() { disable_irq(ADC_IRQn); }

protected:
    void _calibrate();
    static void _enable_clk(Peripheral peripheral);
};


} // namespace adc


} // namespace mcu


#endif
#endif
