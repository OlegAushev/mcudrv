#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "../system/system.h"
#include "../gpio/gpio.h"
#include "../dma/dma.h"
#include <apm32f4xx_adc.h>
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
    GPIO_T* port;
    uint16_t pin;
};


struct Config {
    ADC_CommonConfig_T hal_common_config;
    ADC_Config_T hal_config;
    bool dma_continuous_requests;
};


struct RegularChannelConfig {
    ADC_CHANNEL_T channel;
    std::initializer_list<uint8_t> ranks;
    ADC_SAMPLETIME_T sampletime;
};


struct InjectedChannelConfig {
    ADC_CHANNEL_T channel;
    uint8_t rank;
    ADC_SAMPLETIME_T sampletime;
    uint16_t offset;
    uint8_t conv_nbr;
    bool discontinuous_conv_mode;
    bool auto_conv;
    ADC_EXT_TRIG_INJEC_CONV_T ext_trigger;
    ADC_EXT_TRIG_INJEC_EDGE_T ext_trigger_edge;
};


namespace impl {


inline const std::array<ADC_T*, peripheral_count> adc_instances = {ADC1, ADC2, ADC3};


inline Peripheral to_peripheral(const ADC_T* instance) {
    return static_cast<Peripheral>(
        std::distance(adc_instances.begin(), std::find(adc_instances.begin(), adc_instances.end(), instance))
    );
}


inline std::array<void(*)(void), peripheral_count> adc_clk_enable_funcs = {
    [](){ RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC1); },
    [](){ RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC2); },
    [](){ RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC3); }
};


} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    ADC_T* _reg;
    static inline ADC_Common_T* _reg_common{ADC};
    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    Module(Peripheral peripheral, Config config, dma::Stream* dma = nullptr);

    Peripheral peripheral() const { return _peripheral; }
    ADC_T* reg() { return _reg; }
    static ADC_Common_T* reg_common() { return _reg_common; }

    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void init_injected_channel(const PinConfig& pin_config, const InjectedChannelConfig& channel_config);
    void init_regular_channel(const PinConfig& pin_config, const RegularChannelConfig& channel_config);

    void start_injected() {
        if (_reg->STS_B.INJCSFLG == 1) {
            return; // there is ongoing injected channel conversion
        }
        _reg->CTRL2_B.INJSWSC = 1;
    }

    // TODO uint32_t read_injected(InjectedChannelRank rank) {
    //     switch (rank) {
    //     case InjectedChannelRank::rank1:
    //         return _reg->INJDATA1;
    //     case InjectedChannelRank::rank2:
    //         return _reg->INJDATA2;
    //     case InjectedChannelRank::rank3:
    //         return _reg->INJDATA3;
    //     case InjectedChannelRank::rank4:
    //         return _reg->INJDATA4;
    //     }
    //     return 0xFFFFFFFF;
    // }

    void acknowledge_injected() {
        _reg->STS_B.INJCSFLG = 0;
        _reg->STS_B.INJEOCFLG = 0;
    }

    void start_regular() {
        if (_reg->STS_B.REGCSFLG == 1) {
            return; // there is ongoing regular channel conversion
        }
        _reg->CTRL2_B.REGSWSC = 1;
    }

    bool busy() const {
        return (_reg->STS_B.REGCSFLG == 1);
    }

    bool regular_ready() const {
        return (_reg->STS_B.EOCFLG == 1);
    }

    uint32_t read_regular() {
        return _reg->REGDATA;
    }

    void acknowledge_regular() {
        _reg->STS_B.REGCSFLG = 0;
        _reg->STS_B.EOCFLG = 0;
    }

public:
    void init_interrupts(uint32_t interrupt_list, mcu::IrqPriority priority);
    void enable_interrupts() { enable_irq(ADC_IRQn); }
    void disable_interrupts() { disable_irq(ADC_IRQn); }
    
protected:
    static void _enable_clk(Peripheral peripheral);
};


}


} // namespace mcu


#endif
#endif
