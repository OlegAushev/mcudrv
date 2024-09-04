#pragma once


#ifdef MCUDRV_C28X


#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib/array.h>
#include <emblib/core.h>
#include <emblib/math.h>


namespace mcu {


namespace pwm {


SCOPED_ENUM_DECLARE_BEGIN(State) {
    off,
    on
} SCOPED_ENUM_DECLARE_END(State)


SCOPED_ENUM_DECLARE_BEGIN(CountDirection) {
    up = EPWM_TIME_BASE_STATUS_COUNT_UP,
    down = EPWM_TIME_BASE_STATUS_COUNT_DOWN
} SCOPED_ENUM_DECLARE_END(CountDirection)


SCOPED_ENUM_DECLARE_BEGIN(Peripheral) {
    pwm1,
    pwm2,
    pwm3,
    pwm4,
    pwm5,
    pwm6,
    pwm7,
    pwm8,
    pwm9,
    pwm10,
    pwm11,
    pwm12
} SCOPED_ENUM_DECLARE_END(Peripheral)


SCOPED_ENUM_DECLARE_BEGIN(PhaseCount) {
    one = 1,
    two = 2,
    three = 3,
    six = 6,
    nine = 9,
} SCOPED_ENUM_DECLARE_END(PhaseCount)


SCOPED_ENUM_DECLARE_BEGIN(ClockDivider) {
    divider1 = EPWM_CLOCK_DIVIDER_1,
    divider2 = EPWM_CLOCK_DIVIDER_2,
    divider4 = EPWM_CLOCK_DIVIDER_4,
    divider8 = EPWM_CLOCK_DIVIDER_8,
    divider16 = EPWM_CLOCK_DIVIDER_16,
    divider32 = EPWM_CLOCK_DIVIDER_32,
    divider64 = EPWM_CLOCK_DIVIDER_64,
    divider128 = EPWM_CLOCK_DIVIDER_128
} SCOPED_ENUM_DECLARE_END(ClockDivider)


SCOPED_ENUM_DECLARE_BEGIN(HsClockDivider) {
    divider1 = EPWM_HSCLOCK_DIVIDER_1,
    divider2 = EPWM_HSCLOCK_DIVIDER_2,
    divider4 = EPWM_HSCLOCK_DIVIDER_4,
    divider6 = EPWM_HSCLOCK_DIVIDER_6,
    divider8 = EPWM_HSCLOCK_DIVIDER_8,
    divider10 = EPWM_HSCLOCK_DIVIDER_10,
    divider12 = EPWM_HSCLOCK_DIVIDER_12,
    divider14 = EPWM_HSCLOCK_DIVIDER_14
} SCOPED_ENUM_DECLARE_END(HsClockDivider)


SCOPED_ENUM_DECLARE_BEGIN(OperatingMode) {
    active_high_complementary,
    active_low_complementary,
    pass_through
} SCOPED_ENUM_DECLARE_END(OperatingMode)


SCOPED_ENUM_DECLARE_BEGIN(CounterMode) {
    up = EPWM_COUNTER_MODE_UP,
    down = EPWM_COUNTER_MODE_DOWN,
    updown = EPWM_COUNTER_MODE_UP_DOWN
} SCOPED_ENUM_DECLARE_END(CounterMode)


SCOPED_ENUM_DECLARE_BEGIN(OutputSwap) {
    no,
    yes
} SCOPED_ENUM_DECLARE_END(OutputSwap)


SCOPED_ENUM_DECLARE_BEGIN(CounterCompareModule) {
    a = EPWM_COUNTER_COMPARE_A,
    b = EPWM_COUNTER_COMPARE_B
} SCOPED_ENUM_DECLARE_END(CounterCompareModule)


struct PinConfig { uint32_t pin; uint32_t mux; };


struct Config {
    float switching_freq;
    float deadtime_ns;
    uint32_t clock_prescaler;   // must be the product of clkDivider and hsclkDivider
    ClockDivider clk_divider;
    HsClockDivider hsclk_divider;
    OperatingMode operating_mode;
    CounterMode counter_mode;
    OutputSwap output_swap;
    uint16_t event_interrupt_source;
    bool enable_adc_trigger[2];
    EPWM_ADCStartOfConversionSource adc_trigger_source[2];
};


template <PhaseCount::enum_type Phases>
struct SyncConfig {
    uint16_t sync_delay[Phases];    // delays from internal master module to slave modules, p.1876 of RM
    EPWM_SyncOutPulseMode sync_out_mode[Phases];
    bool enable_phase_shift[Phases];

};


namespace impl {

template <PhaseCount::enum_type Phases>
struct Module {
    uint32_t base[Phases];
    uint32_t pie_event_int_num;
    uint32_t pie_trip_int_num;
};


extern const uint32_t pwm_bases[12];
extern const uint32_t pwm_pie_event_int_nums[12];
extern const uint32_t pwm_pie_trip_int_nums[12];

} // namespace impl


template <PhaseCount::enum_type Phases>
class Module : private emb::noncopyable {
private:
    // there is a divider ( EPWMCLKDIV ) of the system clock
    // which defaults to EPWMCLK = SYSCLKOUT/2, fclk(epwm)max = 100 MHz
    static const float pwm_clk_freq = DEVICE_SYSCLK_FREQ / 2;
    static const float pwm_clk_cycle_ns = 1000000000.f / pwm_clk_freq;
    const float _timebase_clk_freq;
    const float _timebase_cycle_ns;

    Peripheral _peripheral[Phases];
    impl::Module<Phases> _module;
    CounterMode _counter_mode;
    float _switching_freq;
    uint16_t _deadtime_cycles;

    uint16_t _period;               // TBPRD register value
    uint16_t _phase_shift[Phases];	// TBPHS registers values
    uint16_t _sync_delay[Phases];   // delay from internal master module to slave modules, p.1876

    State _state;
public:
    Module(const emb::array<Peripheral, Phases>& peripherals,
            const emb::array<PinConfig, 2*Phases>& pins,
            const pwm::Config& config, const pwm::SyncConfig<Phases> sync_config)
            : _timebase_clk_freq(pwm_clk_freq / config.clock_prescaler)
            , _timebase_cycle_ns(pwm_clk_cycle_ns * config.clock_prescaler)
            , _counter_mode(config.counter_mode)
            , _switching_freq(config.switching_freq)
            , _deadtime_cycles(config.deadtime_ns / _timebase_cycle_ns)
            , _state(State::off) {
        for (size_t i = 0; i < Phases; ++i) {
            _peripheral[i] = peripherals[i];
            _module.base[i] = impl::pwm_bases[peripherals[i].underlying_value()];
        }
        _module.pie_event_int_num = impl::pwm_pie_event_int_nums[peripherals[0].underlying_value()];
        _module.pie_trip_int_num = impl::pwm_pie_trip_int_nums[peripherals[0].underlying_value()];

        for (size_t i = 0; i < Phases; ++i) {
            _phase_shift[i] = 0;
            _sync_delay[i] = sync_config.sync_delay[i];
        }

#ifdef CPU1
        _init_pins(pins);
#endif

        // Disable sync, freeze clock to PWM
        SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

        /* ========================================================================== */
        // Calculate TBPRD value
        switch (config.counter_mode.native_value()) {
        case CounterMode::up:
        case CounterMode::down:
            _period = (_timebase_clk_freq / _switching_freq) - 1;
            break;
        case CounterMode::updown:
            _period = (_timebase_clk_freq / _switching_freq) / 2;
            break;
        }

        for (size_t i = 0; i < Phases; ++i) {
            EPWM_setTimeBasePeriod(_module.base[i], _period);
            EPWM_setTimeBaseCounter(_module.base[i], 0);

            /* ========================================================================== */
            // Clock prescaler
            EPWM_setClockPrescaler(_module.base[i],
                                   static_cast<EPWM_ClockDivider>(config.clk_divider.underlying_value()),
                                   static_cast<EPWM_HSClockDivider>(config.hsclk_divider.underlying_value()));

            /* ========================================================================== */
            // Compare values
            EPWM_setCounterCompareValue(_module.base[i], EPWM_COUNTER_COMPARE_A, 0);

            /* ========================================================================== */
            // Counter mode
            EPWM_setTimeBaseCounterMode(_module.base[i],
                                        static_cast<EPWM_TimeBaseCountMode>(config.counter_mode.underlying_value()));

#ifdef CPU1
            /* ========================================================================== */
            // Sync input source for the EPWM signals
            switch (_module.base[i]) {
            case EPWM4_BASE:
                SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
                break;
            case EPWM7_BASE:
                SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM7, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
                break;
            case EPWM10_BASE:
                SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM10, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
                break;
            default:
                break;
            }
#endif

            /* ========================================================================== */
            // Sync out pulse event
            EPWM_setSyncOutPulseMode(_module.base[i], sync_config.sync_out_mode[i]);

            /* ========================================================================== */
            // Time-base counter synchronization and phase shift
            if (!sync_config.enable_phase_shift[i]) {
                EPWM_disablePhaseShiftLoad(_module.base[i]);
                EPWM_setPhaseShift(_module.base[i], 0);
            } else {
                EPWM_enablePhaseShiftLoad(_module.base[i]);
                switch (config.counter_mode.native_value()) {
                case CounterMode::up:
                    EPWM_setPhaseShift(_module.base[i], _sync_delay[i]);
                    break;
                case CounterMode::down:
                    EPWM_setPhaseShift(_module.base[i], _period - _sync_delay[i]);
                    break;
                case CounterMode::updown:
                    EPWM_setCountModeAfterSync(_module.base[i], EPWM_COUNT_MODE_UP_AFTER_SYNC);
                    EPWM_setPhaseShift(_module.base[i], _sync_delay[i]);
                    break;
                }
            }

            /* ========================================================================== */
            // Shadowing
            EPWM_selectPeriodLoadEvent(_module.base[i], EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);
            EPWM_setCounterCompareShadowLoadMode(_module.base[i], EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
            EPWM_setCounterCompareShadowLoadMode(_module.base[i], EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
            EPWM_setActionQualifierContSWForceShadowMode(_module.base[i], EPWM_AQ_SW_IMMEDIATE_LOAD);

            /* ========================================================================== */
            // PWMxA(CMPA) actions
                // PWMxA configuration for typical waveforms
                // Typically only PWMxA is used by dead-band submodule
            switch (config.counter_mode.native_value()) {
            case CounterMode::up:
                EPWM_setActionQualifierAction(_module.base[i],	EPWM_AQ_OUTPUT_A,
                                              EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
                EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_A,
                                              EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                break;
            case CounterMode::down:
                EPWM_setActionQualifierAction(_module.base[i],	EPWM_AQ_OUTPUT_A,
                                              EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
                EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_A,
                                              EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                break;
            case CounterMode::updown:
                EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_A,
                                              EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
                EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_A,
                                              EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
                break;
            }

            // PWMxB(CMPB) actions
            switch (config.counter_mode.native_value()) {
            case CounterMode::up:
                EPWM_setActionQualifierAction(_module.base[i],	EPWM_AQ_OUTPUT_B,
                                              EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
                EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_B,
                                              EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
                break;
            case CounterMode::down:
                EPWM_setActionQualifierAction(_module.base[i],	EPWM_AQ_OUTPUT_B,
                                              EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
                EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_B,
                                              EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
                break;
            case CounterMode::updown:
                EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_B,
                        EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
                EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_B,
                                              EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
                break;
            }

            /* ========================================================================== */
            // Dead-Band
            EPWM_setDeadBandControlShadowLoadMode(_module.base[i], EPWM_DB_LOAD_ON_CNTR_ZERO);

            switch (config.operating_mode.native_value()) {
            case OperatingMode::active_high_complementary:
                EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_FED, true);
                EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_RED, true);
                EPWM_setDeadBandDelayPolarity(_module.base[i], EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
                EPWM_setDeadBandDelayPolarity(_module.base[i], EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
                break;
            case OperatingMode::active_low_complementary:
                EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_FED, true);
                EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_RED, true);
                EPWM_setDeadBandDelayPolarity(_module.base[i], EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_LOW);
                EPWM_setDeadBandDelayPolarity(_module.base[i], EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
                break;
            case OperatingMode::pass_through:
                EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_FED, false);
                EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_RED, false);
                break;
            }

            EPWM_setRisingEdgeDeadBandDelayInput(_module.base[i], EPWM_DB_INPUT_EPWMA);
            EPWM_setFallingEdgeDeadBandDelayInput(_module.base[i], EPWM_DB_INPUT_EPWMA);
            EPWM_setRisingEdgeDelayCount(_module.base[i], _deadtime_cycles);
            EPWM_setFallingEdgeDelayCount(_module.base[i], _deadtime_cycles);
            EPWM_setDeadBandCounterClock(_module.base[i], EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

            switch (config.output_swap.native_value()) {
            case OutputSwap::no:
                EPWM_setDeadBandOutputSwapMode(_module.base[i], EPWM_DB_OUTPUT_A, false);
                EPWM_setDeadBandOutputSwapMode(_module.base[i], EPWM_DB_OUTPUT_B, false);
                break;
            case OutputSwap::yes:
                EPWM_setDeadBandOutputSwapMode(_module.base[i], EPWM_DB_OUTPUT_A, true);
                EPWM_setDeadBandOutputSwapMode(_module.base[i], EPWM_DB_OUTPUT_B, true);
                break;
            }

            /* ========================================================================== */
            // Trip-Zone actions
            switch (config.operating_mode.native_value()) {
            case OperatingMode::active_high_complementary:
                EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
                EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
                break;
            case OperatingMode::active_low_complementary:
                EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH);
                EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH);
                break;
            case OperatingMode::pass_through:
                EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
                EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
                break;
            }

            EPWM_clearOneShotTripZoneFlag(_module.base[i], EPWM_TZ_OST_FLAG_OST1);
            EPWM_clearTripZoneFlag(_module.base[i], EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
        }

        /* ========================================================================== */
        // ADC Trigger configuration, only first module triggers ADC
        if (config.enable_adc_trigger[0]) {
            EPWM_setADCTriggerSource(_module.base[0], EPWM_SOC_A, config.adc_trigger_source[0]);
            EPWM_setADCTriggerEventPrescale(_module.base[0], EPWM_SOC_A, 1);
            EPWM_enableADCTrigger(_module.base[0], EPWM_SOC_A);
        }

        if (config.enable_adc_trigger[1]) {
            EPWM_setADCTriggerSource(_module.base[0], EPWM_SOC_B, config.adc_trigger_source[1]);
            EPWM_setADCTriggerEventPrescale(_module.base[0], EPWM_SOC_B, 1);
            EPWM_enableADCTrigger(_module.base[0], EPWM_SOC_B);
        }

        /* ========================================================================== */
        // Interrupts, only interrupt on first module is required
        EPWM_setInterruptSource(_module.base[0], config.event_interrupt_source);
        EPWM_setInterruptEventCount(_module.base[0], 1U);

        _init_custom_options();

        stop();

        // Enable sync and clock to PWM
        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    }

    void init_tripzone(const gpio::InputPin& pin, XBAR_InputNum xbar_input) {
#ifdef CPU2
        assert(false);
#else
        assert(static_cast<uint32_t>(xbar_input) <= static_cast<uint32_t>(XBAR_INPUT3));

        switch (pin.config().actstate.native_value()) {
        case emb::gpio::active_pin_state::low:
            GPIO_setPadConfig(pin.config().pin, GPIO_PIN_TYPE_PULLUP);
            break;
        case emb::gpio::active_pin_state::high:
            GPIO_setPadConfig(pin.config().pin, GPIO_PIN_TYPE_INVERT);
            break;
        }

        GPIO_setPinConfig(pin.config().mux);
        GPIO_setDirectionMode(pin.config().pin, GPIO_DIR_MODE_IN);
        GPIO_setQualificationMode(pin.config().pin, GPIO_QUAL_ASYNC);

        XBAR_setInputPin(xbar_input, pin.config().pin);
        uint16_t tripzone_signal;
        switch (xbar_input) {
        case XBAR_INPUT1:
            tripzone_signal = EPWM_TZ_SIGNAL_OSHT1;
            break;
        case XBAR_INPUT2:
            tripzone_signal = EPWM_TZ_SIGNAL_OSHT2;
            break;

        case XBAR_INPUT3:
            tripzone_signal = EPWM_TZ_SIGNAL_OSHT3;
            break;
        default:
            tripzone_signal = EPWM_TZ_SIGNAL_OSHT3;
            break;
        }

        for (size_t i = 0; i < Phases; ++i) {
            // Enable tzSignal as one shot trip source
            EPWM_enableTripZoneSignals(_module.base[i], tripzone_signal);
        }
#endif
    }

#ifdef CPU1
    static void transfer_control_to_cpu2(const emb::array<Peripheral, Phases>& peripherals,
                                         const emb::array<PinConfig, 2*Phases> pins) {
        SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);  // Disable sync(Freeze clock to PWM as well)
        _init_pins(pins);
        for (size_t i = 0; i < pins.size(); ++i) {
            GPIO_setMasterCore(pins[i].pin, GPIO_CORE_CPU2);
        }

        for (size_t i = 0; i < peripherals.size(); ++i) {
            SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL0_EPWM, peripherals[i].underlying_value()+1, SYSCTL_CPUSEL_CPU2);
        }

        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);   // Enable sync and clock to PWM
    }
#endif

    uint32_t base() const { return _module.base[0]; }
    uint16_t period() const { return _period; }
    float freq() const { return _switching_freq; }

    void set_freq(float freq) {
        _switching_freq = freq;
        switch (_counter_mode.native_value()) {
        case CounterMode::up:
        case CounterMode::down:
            _period = (_timebase_clk_freq / _switching_freq) - 1;
            break;
        case CounterMode::updown:
            _period = (_timebase_clk_freq / _switching_freq) / 2;
            break;
        }

        for (size_t i = 0; i < Phases; ++i) {
            EPWM_setTimeBasePeriod(_module.base[i], _period);
        }
    }

    void set_compare_value(const emb::array<uint16_t, Phases>& cmp_value,
                           CounterCompareModule cmp_module = CounterCompareModule::a) {
        for (size_t i = 0; i < Phases; ++i) {
            EPWM_setCounterCompareValue(_module.base[i],
                                        static_cast<EPWM_CounterCompareModule>(cmp_module.underlying_value()),
                                        cmp_value[i]);
        }
    }

    void set_compare_value(uint16_t cmp_value, CounterCompareModule cmp_module = CounterCompareModule::a) {
        for (size_t i = 0; i < Phases; ++i) {
            EPWM_setCounterCompareValue(_module.base[i],
                                        static_cast<EPWM_CounterCompareModule>(cmp_module.underlying_value()),
                                        cmp_value);
        }
    }

    void set_duty_cycle(const emb::array<emb::unsigned_perunit, Phases>& duty_cycle, CounterCompareModule cmp_module = CounterCompareModule::a) {
        for (size_t i = 0; i < Phases; ++i) {
            EPWM_setCounterCompareValue(_module.base[i],
                                        static_cast<EPWM_CounterCompareModule>(cmp_module.underlying_value()),
                                        static_cast<uint16_t>(duty_cycle[i].get() * _period));
        }
    }

    void set_duty_cycle(emb::unsigned_perunit duty_cycle, CounterCompareModule cmp_module = CounterCompareModule::a) {
        for (size_t i = 0; i < Phases; ++i) {
            EPWM_setCounterCompareValue(_module.base[i],
                                        static_cast<EPWM_CounterCompareModule>(cmp_module.underlying_value()),
                                        static_cast<uint16_t>(duty_cycle.get() * _period));
        }
    }

    void set_phase_shift(const emb::array<uint16_t, Phases>& phase_shift) {
        for (size_t i = 0; i < Phases; ++i) {
            EPWM_setPhaseShift(_module.base[i], phase_shift[i]);
        }
    }

    void start() {
        _state = State::on;
        for (size_t i = 0; i < Phases; ++i) {
            EPWM_clearTripZoneFlag(_module.base[i], EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
        }
    }

    void stop() {
        for (size_t i = 0; i < Phases; ++i) {
            EPWM_forceTripZoneEvent(_module.base[i], EPWM_TZ_FORCE_EVENT_OST);
        }
        _state = State::off;
    }

    State state() const { return _state; }
    CountDirection count_direction() const { return CountDirection(EPWM_getTimeBaseCounterDirection(_module.base[0])); }

    void enable_event_interrupts() { EPWM_enableInterrupt(_module.base[0]); }
    void enable_trip_interrupts() { EPWM_enableTripZoneInterrupt(_module.base[0], EPWM_TZ_INTERRUPT_OST); }
    void disable_event_interrupts() { EPWM_disableInterrupt(_module.base[0]); }
    void disable_trip_interrupts() { EPWM_disableTripZoneInterrupt(_module.base[0], EPWM_TZ_INTERRUPT_OST); }

    void register_event_interrupt_handler(void (*handler)(void)) {
        Interrupt_register(_module.pie_event_int_num, handler);
        Interrupt_enable(_module.pie_event_int_num);
    }

    void register_trip_interrupt_handler(void (*handler)(void)) {
        Interrupt_register(_module.pie_trip_int_num, handler);
        Interrupt_enable(_module.pie_trip_int_num);
    }

    void acknowledge_event_interrupt() {
        EPWM_clearEventTriggerInterruptFlag(_module.base[0]);
        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
    }

    void acknowledge_trip_interrupt() { Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2); }
protected:
#ifdef CPU1
    static void _init_pins(const emb::array<PinConfig, 2*Phases> pins) {
        for (size_t i = 0; i < pins.size(); ++i) {
            GPIO_setPadConfig(pins[i].pin, GPIO_PIN_TYPE_STD);
            GPIO_setPinConfig(pins[i].mux);
        }
    }
#endif
    void _init_custom_options();

public:
#ifdef CPU1
    static void preset_pins(const emb::array<PinConfig, 2*Phases>& pins, emb::gpio::active_pin_state actstate) {
        for (size_t i = 0; i < pins.size(); ++i) {
            mcu::gpio::PinConfig cfg = {pins[i].pin, pins[i].mux, mcu::gpio::Direction::output, actstate,
                                     mcu::gpio::Type::std, mcu::gpio::QualMode::sync, 1, mcu::gpio::MasterCore::cpu1};
            mcu::gpio::OutputPin pin(cfg);
            pin.reset();
        }
    }
#endif
};


} // namespace pwm


} // namespace mcu


#endif
