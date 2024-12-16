#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/timers/timerdef.h>
#include <utility>


namespace mcu {


namespace timers {


enum class GeneralPurposePeripheral : unsigned int {
    tim2,
    tim3,
    tim4,
    tim5,
    tim12,
    tim13,
    tim14,
    tim15,
    tim16,
    tim17
};


constexpr size_t gp_timer_peripheral_count = 10;


namespace impl {


inline const std::array<TIM_TypeDef*, gp_timer_peripheral_count> gp_timer_instances = {TIM2, TIM3, TIM4, TIM5,
                                                                                       TIM12, TIM13, TIM14,
                                                                                       TIM15, TIM16, TIM17};


inline GeneralPurposePeripheral to_peripheral(const TIM_TypeDef* instance) {
    return static_cast<GeneralPurposePeripheral>(
        std::distance(gp_timer_instances.begin(),
                      std::find(gp_timer_instances.begin(),
                                gp_timer_instances.end(),
                                instance)));
}


inline std::array<void(*)(void), gp_timer_peripheral_count> gp_timer_clk_enable_funcs = {
    [](){ __HAL_RCC_TIM2_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM3_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM4_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM5_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM12_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM13_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM14_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM15_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM16_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM17_CLK_ENABLE(); },
};


inline std::array<bool, gp_timer_peripheral_count> gp_clk_enabled = {};


inline constexpr std::array<IRQn_Type, gp_timer_peripheral_count> gp_timer_irqn = {TIM2_IRQn, TIM3_IRQn, TIM4_IRQn, TIM5_IRQn,
                                                                                   TIM8_BRK_TIM12_IRQn, TIM8_UP_TIM13_IRQn, TIM8_TRG_COM_TIM14_IRQn,
                                                                                   TIM15_IRQn, TIM16_IRQn, TIM17_IRQn};


} // namespace impl


template <OperationMode OpMode>
class GeneralPurposeTimer : public emb::singleton_array<GeneralPurposeTimer<OpMode>, gp_timer_peripheral_count>, public emb::noncopyable {
private:
    const GeneralPurposePeripheral _peripheral;
    TIM_HandleTypeDef _handle{};
    TIM_TypeDef* _reg;

    float _freq{0};
    float _t_dts_ns{0};

    bool _brk_enabled{false};
public:
    GeneralPurposeTimer(GeneralPurposePeripheral peripheral, const Config& config)
            : emb::singleton_array<GeneralPurposeTimer<OpMode>, gp_timer_peripheral_count>(this, std::to_underlying(peripheral))
            , _peripheral(peripheral)
    {
        _enable_clk(peripheral);

        _reg = impl::gp_timer_instances[std::to_underlying(_peripheral)];
        _handle.Instance = _reg;
        _handle.Init = config.hal_base_config;

        if (config.hal_base_config.Period == 0 && config.freq > 0) {
            // period specified by freq
            _freq = config.freq;
            float timebase_freq = static_cast<float>(core_clk_freq()) / static_cast<float>(config.hal_base_config.Prescaler+1);

            if (peripheral != GeneralPurposePeripheral::tim15
                    && peripheral != GeneralPurposePeripheral::tim16
                    && peripheral != GeneralPurposePeripheral::tim17) {
                timebase_freq = timebase_freq / 2;
            }

            if (peripheral == GeneralPurposePeripheral::tim2
                    || peripheral == GeneralPurposePeripheral::tim3
                    || peripheral == GeneralPurposePeripheral::tim4
                    || peripheral == GeneralPurposePeripheral::tim5) {
                switch (config.hal_base_config.CounterMode) {
                case TIM_COUNTERMODE_UP:
                case TIM_COUNTERMODE_DOWN:
                    _handle.Init.Period = static_cast<uint32_t>((timebase_freq / config.freq) - 1);
                    break;
                case TIM_COUNTERMODE_CENTERALIGNED1:
                case TIM_COUNTERMODE_CENTERALIGNED2:
                case TIM_COUNTERMODE_CENTERALIGNED3:
                    _handle.Init.Period = static_cast<uint32_t>((timebase_freq / config.freq) / 2);
                    break;
                }
            } else {
                _handle.Init.Period = static_cast<uint32_t>((timebase_freq / config.freq) - 1);
            }
        }

        switch (config.hal_base_config.ClockDivision) {
        case TIM_CLOCKDIVISION_DIV1:
            _t_dts_ns = static_cast<float>(config.hal_base_config.Prescaler+1) * 1000000000.f / static_cast<float>(core_clk_freq());
            break;
        case TIM_CLOCKDIVISION_DIV2:
            _t_dts_ns = 2 * static_cast<float>(config.hal_base_config.Prescaler+1) * 1000000000.f / static_cast<float>(core_clk_freq());
            break;
        case TIM_CLOCKDIVISION_DIV4:
            _t_dts_ns = 4 * static_cast<float>(config.hal_base_config.Prescaler+1) * 1000000000.f / static_cast<float>(core_clk_freq());
            break;
        default:
            fatal_error("timer initialization failed");
            break;
        }

        switch (OpMode) {
            case OperationMode::timebase:
                if (HAL_TIM_Base_Init(&_handle) != HAL_OK) {
                    fatal_error("timer timebase initialization failed");
                }
                break;
            case OperationMode::pwm:
                if (HAL_TIM_PWM_Init(&_handle) != HAL_OK) {
                    fatal_error("timer pwm initialization failed");
                }
                break;
        }
    }

    GeneralPurposePeripheral peripheral() const { return _peripheral; }
    TIM_HandleTypeDef* handle() { return &_handle; }
    TIM_TypeDef* reg() { return _reg; }
    
    static GeneralPurposeTimer* instance(GeneralPurposePeripheral peripheral) {
        return emb::singleton_array<GeneralPurposeTimer, gp_timer_peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void start() {
        set_bit<uint32_t>(_reg->CR1, TIM_CR1_CEN);
    }

    float freq() const { return _freq; }

    void initialize_update_interrupts(IrqPriority priority) {
        set_bit<uint32_t>(_reg->DIER, TIM_DIER_UIE);
        set_irq_priority(impl::gp_timer_irqn[std::to_underlying(_peripheral)], priority);
    }

    void enable_update_interrupts() {
        clear_bit<uint32_t>(_reg->SR, TIM_SR_UIF);
        clear_pending_irq(impl::gp_timer_irqn[std::to_underlying(_peripheral)]);
        enable_irq(impl::gp_timer_irqn[std::to_underlying(_peripheral)]);
    }

    void disable_update_interrupts() {
        disable_irq(impl::gp_timer_irqn[std::to_underlying(_peripheral)]);
    }

private:
    static void _enable_clk(GeneralPurposePeripheral peripheral) {
        auto timer_idx = std::to_underlying(peripheral);
        if (impl::gp_clk_enabled[timer_idx]) {
            return;
        }

        impl::gp_timer_clk_enable_funcs[timer_idx]();
        impl::gp_clk_enabled[timer_idx] = true;
    }
};


} // namespace mcu


} // namespace timers


#endif
#endif
