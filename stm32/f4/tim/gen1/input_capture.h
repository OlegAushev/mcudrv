#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/tim/gen1/base.h>
#include <initializer_list>
#include <optional>


namespace mcu {
namespace tim {
namespace gen1 {


struct InputCaptureConfig {
    TIM_Base_InitTypeDef hal_base_config;
};


struct InputCaptureChannelConfig {
    TIM_IC_InitTypeDef hal_ic_config;
};


class InputCaptureTimer : public impl::AbstractTimer {
private:

public:
    InputCaptureTimer(Peripheral peripheral, const InputCaptureConfig& config);
    void initialize_channel(Channel channel, ChPin* pin_ch, const InputCaptureChannelConfig& config);
    
    uint32_t read_captured(Channel channel) const {
        switch (channel) {
        case Channel::channel1:
            return read_reg(_reg->CCR1);
        case Channel::channel2:
            return read_reg(_reg->CCR2);
        case Channel::channel3:
            return read_reg(_reg->CCR3);
        case Channel::channel4:
            return read_reg(_reg->CCR4);
        }
    }

    std::array<std::optional<uint32_t>, channel_count> read_ack_captured() {
        std::array<std::optional<uint32_t>, channel_count> captured{};
        if (bit_is_set<uint32_t>(_reg->SR, TIM_SR_CC1IF)) {
            captured[0] = read_reg(_reg->CCR1);
            clear_bit<uint32_t>(_reg->SR, TIM_SR_CC1IF);
        }
        if (bit_is_set<uint32_t>(_reg->SR, TIM_SR_CC2IF)) {
            captured[1] = read_reg(_reg->CCR2);
            clear_bit<uint32_t>(_reg->SR, TIM_SR_CC2IF);
        }
        if (bit_is_set<uint32_t>(_reg->SR, TIM_SR_CC3IF)) {
            captured[2] = read_reg(_reg->CCR3);
            clear_bit<uint32_t>(_reg->SR, TIM_SR_CC3IF);
        }
        if (bit_is_set<uint32_t>(_reg->SR, TIM_SR_CC4IF)) {
            captured[3] = read_reg(_reg->CCR4);
            clear_bit<uint32_t>(_reg->SR, TIM_SR_CC4IF);
        }
        return captured;
    }

    void initialize_interrupts(std::initializer_list<InterruptSource> sources, IrqPriority priority);
    
    void enable_interrupts() {
        clear_bit<uint32_t>(_reg->SR, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF |
                                      TIM_SR_TIF | TIM_SR_CC1OF | TIM_SR_CC2OF | TIM_SR_CC3OF | TIM_SR_CC4OF);
        clear_pending_irq(impl::irq_nums[std::to_underlying(_peripheral)]);
        enable_irq(impl::irq_nums[std::to_underlying(_peripheral)]);
    }

    void disable_interrupts() {
        disable_irq(impl::irq_nums[std::to_underlying(_peripheral)]);
    }

    bool acknowledge_update_interrupt() {
        if (bit_is_set<uint32_t>(_reg->SR, TIM_SR_UIF)) {
            clear_bit<uint32_t>(_reg->SR, TIM_SR_UIF);
            return true;
        }
        return false;
    }

    bool acknowledge_overcapture() {
        if (bit_is_set<uint32_t>(_reg->SR, TIM_SR_CC1OF | TIM_SR_CC2OF | TIM_SR_CC3OF | TIM_SR_CC4OF)) {
            clear_bit<uint32_t>(_reg->SR, TIM_SR_CC1OF | TIM_SR_CC2OF | TIM_SR_CC3OF | TIM_SR_CC4OF);
            return true;
        }
        return false;
    }
};


} // namespace general
} // namespace timers
} // namepsace mcu


#endif
#endif
