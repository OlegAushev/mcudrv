#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv/apm32/f4/tim/timdef.hpp>

namespace mcu {
namespace apm32 {
namespace tim {
namespace adv {

constexpr size_t peripheral_count = 2;
enum class Peripheral : unsigned int { tim1, tim8 };

constexpr size_t channel_count = 4;
enum class Channel : unsigned int {
    channel1 = TMR_CHANNEL_1,
    channel2 = TMR_CHANNEL_2,
    channel3 = TMR_CHANNEL_3,
    channel4 = TMR_CHANNEL_4,
};

namespace impl {

inline const std::array<TMR_T*, peripheral_count> instances = {TMR1, TMR8};

inline Peripheral to_peripheral(const TMR_T* instance) {
    return static_cast<Peripheral>(std::distance(
            instances.begin(),
            std::find(instances.begin(), instances.end(), instance)));
}

inline std::array<void (*)(void), peripheral_count> clk_enable_funcs = {
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR1); },
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR8); },
};

inline constexpr std::array<IRQn_Type, peripheral_count> up_irq_nums = {
    TMR1_UP_TMR10_IRQn, TMR8_UP_TMR13_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> brk_irq_nums = {
    TMR1_BRK_TMR9_IRQn, TMR8_BRK_TMR12_IRQn};

class AbstractTimer
        : public emb::singleton_array<AbstractTimer, peripheral_count>,
          public emb::noncopyable {
protected:
    const Peripheral _peripheral;
    TMR_T* const _reg;
    const OpMode _mode;

    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    AbstractTimer(Peripheral peripheral, OpMode mode)
            : emb::singleton_array<AbstractTimer, peripheral_count>(
                      this, std::to_underlying(peripheral)),
              _peripheral(peripheral),
              _reg(impl::instances[std::to_underlying(peripheral)]),
              _mode(mode) {
        _enable_clk(peripheral);
    }

    Peripheral peripheral() const { return _peripheral; }
    TMR_T* reg() { return _reg; }
    OpMode mode() const { return _mode; }
    CountDir dir() const {
        if (_reg->CTRL1_B.CNTDIR == 1) {
            return CountDir::down;
        }
        return CountDir::up;
    }

    void enable() { _reg->CTRL1_B.CNTEN = 1; }

    void disable() { _reg->CTRL1_B.CNTEN = 0; }
private:
    static void _enable_clk(Peripheral peripheral) {
        auto timer_idx = std::to_underlying(peripheral);
        if (_clk_enabled[timer_idx]) {
            return;
        }

        impl::clk_enable_funcs[timer_idx]();
        _clk_enabled[timer_idx] = true;
    }
};

} // namespace impl

} // namespace adv
} // namespace tim
} // namespace apm32
} // namespace mcu

#endif
#endif
