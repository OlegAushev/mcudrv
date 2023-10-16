#pragma once

#ifdef STM32F4xx

#include "../mcudef.h"
#include "../gpio/gpio.h"
#include "../system/system.h"
#include <utility>


namespace mcu {
namespace dac {


enum class Peripheral {
    dac1
};


enum class Channel {
    channel1 = DAC_CHANNEL_1,
    channel2 = DAC_CHANNEL_2
};


enum class DataAlignment {
    right_12bit = DAC_ALIGN_12B_R,
    left_12bit = DAC_ALIGN_12B_L,
    right_8bit = DAC_ALIGN_8B_R
};


constexpr int peripheral_count = 1;


struct PinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
};


struct ChannelConfig {
    DAC_ChannelConfTypeDef hal_config;
};


namespace impl {


inline constexpr std::array<DAC_TypeDef*, peripheral_count> dac_instances = {DAC1};


inline Peripheral to_peripheral(const DAC_TypeDef* instance) {
    return static_cast<Peripheral>(
        std::distance(dac_instances.begin(), std::find(dac_instances.begin(), dac_instances.end(), instance))
    );
}


inline std::array<void(*)(void), peripheral_count> dac_clk_enable_funcs = {
    [](){ __HAL_RCC_DAC_CLK_ENABLE(); },
};


} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    DAC_HandleTypeDef _handle{};
    DAC_TypeDef* _reg;
    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    Module(Peripheral peripheral);
    void init_channel(Channel channel, const PinConfig& pin_config, ChannelConfig config);
    
    Peripheral peripheral() const { return _peripheral; }
    DAC_HandleTypeDef* handle() { return &_handle; }
    DAC_TypeDef* reg() { return _reg; }

    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void convert(Channel channel, DataAlignment alignment, uint32_t value) {
        uint32_t tmp = reinterpret_cast<uint32_t>(_handle.Instance);
        if (channel == Channel::channel1) {
            tmp += DAC_DHR12R1_ALIGNMENT(std::to_underlying(alignment));
        } else {
            tmp += DAC_DHR12R2_ALIGNMENT(std::to_underlying(alignment));
        }

        *(reinterpret_cast<volatile uint32_t *>(tmp)) = value;
    }

protected:
    static void _enable_clk(Peripheral peripheral);
};


} // namespace dac
} // namespace mcu


#endif
