#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/system/system.h>
#include <mcudrv/apm32/f4/gpio/gpio.h>
#include <mcudrv/apm32/f4/dma/dma.h>
#include <apm32f4xx_adc.h>
#include <optional>
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
    gpio::Port port;
    gpio::Pin pin;
};


struct InjectedConfig {
    uint8_t conv_num;
    bool auto_conv;
    bool discontinuous_mode;
    ADC_EXT_TRIG_INJEC_EDGE_T ext_trigger_edge;
    ADC_EXT_TRIG_INJEC_CONV_T ext_trigger;
};


struct Config {
    ADC_CommonConfig_T  hal_common_config;
    ADC_RESOLUTION_T    resolution;
    bool                scan_mode;
    bool                continuous_mode;
    ADC_EXT_TRIG_EDGE_T ext_trigger_edge;
    ADC_EXT_TRIG_CONV_T ext_trigger;
    ADC_DATA_ALIGN_T    data_align;
    uint8_t             conv_num;
    bool                eoc_on_each_conv;
    bool                dma_continuous_requests;
    bool                discontinuous_mode;
    uint8_t             discontinuous_conv_num;
    std::optional<InjectedConfig> injected;
};


struct RegularChannelConfig {
    ADC_CHANNEL_T channel;
    ADC_SAMPLETIME_T sampletime;
    std::initializer_list<uint8_t> ranks;
};


enum class InjectedChannelRank {
    rank1 = ADC_INJEC_CHANNEL_1,
    rank2 = ADC_INJEC_CHANNEL_2,
    rank3 = ADC_INJEC_CHANNEL_3,
    rank4 = ADC_INJEC_CHANNEL_4
};


struct InjectedChannelConfig {
    ADC_CHANNEL_T channel;
    ADC_SAMPLETIME_T sampletime;
    std::initializer_list<InjectedChannelRank> ranks;
    uint16_t offset;
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


class Module : public emb::singleton_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    ADC_T* _reg;
    static inline ADC_Common_T* _reg_common{ADC};
    static inline std::array<bool, peripheral_count> _clk_enabled{};
    static inline bool _common_initialized{false};
public:
    Module(Peripheral peripheral, Config config, dma::Stream* dma = nullptr);

    Peripheral peripheral() const { return _peripheral; }
    ADC_T* reg() { return _reg; }
    static ADC_Common_T* reg_common() { return _reg_common; }

    static Module* instance(Peripheral peripheral) {
        return emb::singleton_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void init_injected(const PinConfig& pin_config, const InjectedChannelConfig& channel_config);
    void init_regular(const PinConfig& pin_config, const RegularChannelConfig& channel_config);

    void init_internal_injected(const InjectedChannelConfig& channel_config);
    void init_internal_regular(const RegularChannelConfig& channel_config);

    void start_injected() {
        if (_reg->STS_B.INJCSFLG == 1) {
            return; // there is ongoing injected channel conversion
        }
        _reg->CTRL2_B.INJSWSC = 1;
    }

    bool injected_busy() const { return (_reg->STS_B.INJCSFLG == 1); }
    bool injected_ready() const { return (_reg->STS_B.INJEOCFLG == 1); }

    uint32_t read_injected(InjectedChannelRank rank) {
        switch (rank) {
        case InjectedChannelRank::rank1:
            return _reg->INJDATA1;
        case InjectedChannelRank::rank2:
            return _reg->INJDATA2;
        case InjectedChannelRank::rank3:
            return _reg->INJDATA3;
        case InjectedChannelRank::rank4:
            return _reg->INJDATA4;
        }
        return 0xFFFFFFFF;
    }

    void ack_injected() {
        _reg->STS_B.INJCSFLG = 0;
        _reg->STS_B.INJEOCFLG = 0;
    }

    void start_regular() {
        if (_reg->STS_B.REGCSFLG == 1) {
            return; // there is ongoing regular channel conversion
        }
        _reg->CTRL2_B.REGSWSC = 1;
    }

    bool regular_busy() const { return (_reg->STS_B.REGCSFLG == 1); }
    bool regular_ready() const { return (_reg->STS_B.EOCFLG == 1); }

    uint32_t read_regular() { return _reg->REGDATA; }

    void ack_regular() {
        _reg->STS_B.REGCSFLG = 0;
        _reg->STS_B.EOCFLG = 0;
    }

public:
    void init_interrupts(uint32_t interrupt_bitset);
    static void set_interrupt_priority(mcu::IrqPriority priority) { mcu::set_irq_priority(ADC_IRQn, priority); }
    static void enable_interrupts() { enable_irq(ADC_IRQn); }
    static void disable_interrupts() { disable_irq(ADC_IRQn); }
    bool check_interrupt(ADC_INT_T interrupt) const {
        auto sts = _reg->STS_B;
        auto cr1 = _reg->CTRL1_B;

        switch (interrupt) {
        case ADC_INT_EOC:
            return sts.EOCFLG && cr1.EOCIEN;
            break;
        case ADC_INT_AWD:
            return sts.AWDFLG && cr1.AWDIEN;
            break;
        case ADC_INT_INJEOC:
            return sts.INJEOCFLG && cr1.INJEOCIEN;
            break;
        case ADC_INT_OVR:
            return sts.OVREFLG && cr1.OVRIEN;
            break;
        }
        return false;
    }
protected:
    static void _enable_clk(Peripheral peripheral);
};


}


} // namespace mcu


#endif
#endif
